// ============================================================
//  AudioEffectSpectralNR - reducere spectrala de zgomot
//  varianta Ephraim-Malah / Wiener cu SNR "decision-directed"
//  (dupa ideile din UHSDR si DD4WH Teensy-ConvolutionSDR)
//
//  Procesare: FFT 256, hop 128 (50% suprapunere), fereastra
//  sqrt-Hann la analiza si sinteza, overlap-add. Un hop per
//  bloc audio => latenta adaugata ~2.9 ms, CPU ~12% pe T3.6.
//  Rezolutie: 172 Hz/bin la 44.1 kHz - suficient pt fonie.
//
//  Algoritm per bin (0..128):
//   - PSD netezita, zgomot estimat prin urmarirea minimelor
//     (coboara instant, urca lent) * factor de supraestimare
//   - SNR aposteriori gamma = P/N
//   - SNR apriori xi prin decision-directed (Ephraim-Malah)
//   - gain Wiener G = xi/(1+xi), cu prag inferior (floor)
//     reglabil = intensitatea NR; netezire pe frecventa
//     contra "musical noise"
// ============================================================
#pragma once

#include <Arduino.h>
#include <AudioStream.h>
#include <arm_math.h>

#define NR_FFT   256
#define NR_BINS  129            // 0..128 (jumatatea reala)

class AudioEffectSpectralNR : public AudioStream {
public:
  AudioEffectSpectralNR() : AudioStream(1, inputQueueArray) {
    arm_cfft_radix4_init_f32(&fftInst,  NR_FFT, 0, 1);
    arm_cfft_radix4_init_f32(&ifftInst, NR_FFT, 1, 1);
    for (int i = 0; i < NR_FFT; i++)
      win[i] = sqrtf(0.5f * (1.0f - cosf(2.0f * PI * i / NR_FFT)));
    setLevel(0);
    resetState();
  }

  // 0 = oprit, 1..3 = slab / mediu / agresiv
  void setLevel(int lvl) {
    level = constrain(lvl, 0, 3);
    const float floors[4] = {1.0f, 0.45f, 0.25f, 0.12f};
    gainFloor = floors[level];
    if (level > 0 && !wasOn) resetState();
    wasOn = (level > 0);
  }
  int getLevel() { return level; }

  virtual void update(void) {
    audio_block_t* in = receiveReadOnly();
    if (!in) return;
    if (level == 0) { transmit(in); release(in); return; }

    audio_block_t* out = allocate();
    if (!out) { transmit(in); release(in); return; }

    // fereastra de analiza: 128 mostre vechi + 128 noi
    float t[NR_FFT];
    for (int i = 0; i < 128; i++) {
      t[i]       = prevIn[i];
      t[i + 128] = (float)in->data[i] / 32768.0f;
      prevIn[i]  = t[i + 128];
    }
    for (int i = 0; i < NR_FFT; i++) {
      fftBuf[2 * i]     = t[i] * win[i];
      fftBuf[2 * i + 1] = 0.0f;
    }
    arm_cfft_radix4_f32(&fftInst, fftBuf);

    // gain per bin
    float G[NR_BINS];
    for (int k = 0; k < NR_BINS; k++) {
      float re = fftBuf[2 * k], im = fftBuf[2 * k + 1];
      float P = re * re + im * im;

      psd[k] = 0.85f * psd[k] + 0.15f * P;

      // urmarirea minimelor: zgomotul coboara instant, urca lent
      if (warmup > 0)            nest[k] = psd[k];
      else if (psd[k] < nest[k]) nest[k] = psd[k];
      else                       nest[k] *= 1.0006f;   // ~x1.2/sec

      float noise = nest[k] * 2.0f + 1e-12f;  // supraestimare 2x
      float gamma = P / noise;
      float xi = 0.95f * prevG2g[k] + 0.05f * max(gamma - 1.0f, 0.0f);
      float g = xi / (1.0f + xi);
      prevG2g[k] = g * g * gamma;
      G[k] = constrain(g, gainFloor, 1.0f);
    }
    if (warmup > 0) warmup--;

    // netezire pe frecventa contra musical noise
    float Gs[NR_BINS];
    Gs[0] = G[0]; Gs[NR_BINS - 1] = G[NR_BINS - 1];
    for (int k = 1; k < NR_BINS - 1; k++)
      Gs[k] = 0.25f * G[k - 1] + 0.5f * G[k] + 0.25f * G[k + 1];

    // aplica gain-ul (si pe jumatatea-oglinda a spectrului)
    for (int k = 0; k < NR_BINS; k++) {
      fftBuf[2 * k]     *= Gs[k];
      fftBuf[2 * k + 1] *= Gs[k];
      if (k > 0 && k < NR_FFT / 2) {
        fftBuf[2 * (NR_FFT - k)]     *= Gs[k];
        fftBuf[2 * (NR_FFT - k) + 1] *= Gs[k];
      }
    }
    arm_cfft_radix4_f32(&ifftInst, fftBuf);  // include scalarea 1/N

    // fereastra de sinteza + overlap-add
    for (int i = 0; i < 128; i++) {
      float s = ola[i] + fftBuf[2 * i] * win[i];
      out->data[i] = (int16_t)constrain(s * 32768.0f, -32768.0f, 32767.0f);
      ola[i] = fftBuf[2 * (i + 128)] * win[i + 128];
    }

    transmit(out);
    release(out);
    release(in);
  }

private:
  void resetState() {
    memset(prevIn, 0, sizeof(prevIn));
    memset(ola, 0, sizeof(ola));
    memset(prevG2g, 0, sizeof(prevG2g));
    for (int k = 0; k < NR_BINS; k++) { psd[k] = 1e-6f; nest[k] = 1e-6f; }
    warmup = 30;   // ~0.1s: invata zgomotul la pornire
  }

  audio_block_t* inputQueueArray[1];
  arm_cfft_radix4_instance_f32 fftInst, ifftInst;
  float win[NR_FFT];
  float fftBuf[2 * NR_FFT];
  float prevIn[128], ola[128];
  float psd[NR_BINS], nest[NR_BINS], prevG2g[NR_BINS];
  float gainFloor = 1.0f;
  int   level = 0, warmup = 0;
  bool  wasOn = false;
};
