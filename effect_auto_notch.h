// ============================================================
//  AudioEffectAutoNotch - notch automat adaptiv (ANF)
//
//  Principiu (clasic in DSP-urile de radioamatori, cf. DSPham/UHSDR):
//  un filtru LMS incearca sa prezica mostra curenta din mostre
//  intarziate cu ANF_DELAY. Componentele periodice (purtatoare,
//  fluieraturi) sunt predictibile -> filtrul le invata in <1s;
//  vocea nu e predictibila la distanta asta -> ramane in semnalul
//  de eroare, care e iesirea noastra. Rezultat: tonurile constante
//  dispar singure, oriunde ar fi in banda, oricate ar fi.
//
//  ATENTIE: pe CW notch-ul trebuie tinut OPRIT - tonul de telegrafie
//  e exact o "purtatoare" si ar fi taiat.
//
//  Implementare: NLMS din CMSIS (arm_lms_norm_f32), 64 taps,
//  intarziere de decorelare 32 mostre, per bloc de 128 la 44.1 kHz.
// ============================================================
#pragma once

#include <Arduino.h>
#include <AudioStream.h>
#include <arm_math.h>

// 32 taps ajung pentru tonuri pure si costa jumatate fata de 64
#define ANF_TAPS  32
#define ANF_DELAY 16
#define ANF_MU    0.04f

class AudioEffectAutoNotch : public AudioStream {
public:
  AudioEffectAutoNotch() : AudioStream(1, inputQueueArray) { reset(); }

  void setEnabled(bool e) {
    if (e && !en) reset();   // porneste cu filtrul "curat"
    en = e;
  }
  bool enabled() { return en; }

  virtual void update(void) {
    audio_block_t* in = receiveReadOnly();
    if (!in) return;
    if (!en) { transmit(in); release(in); return; }

    audio_block_t* out = allocate();
    if (!out) { transmit(in); release(in); return; }

    float x[128], ref[128], y[128], e[128];
    for (int i = 0; i < 128; i++)
      x[i] = (float)in->data[i] / 32768.0f;

    // referinta = semnalul intarziat cu ANF_DELAY mostre
    for (int i = 0; i < 128; i++)
      ref[i] = (i < ANF_DELAY) ? hist[i] : x[i - ANF_DELAY];
    memcpy(hist, &x[128 - ANF_DELAY], sizeof(hist));

    // y = partea predictibila (tonurile), e = restul (vocea) = iesirea
    arm_lms_norm_f32(&lms, ref, x, y, e, 128);

    for (int i = 0; i < 128; i++)
      out->data[i] = (int16_t)constrain(e[i] * 32768.0f, -32768.0f, 32767.0f);

    transmit(out);
    release(out);
    release(in);
  }

private:
  void reset() {
    memset(coeffs, 0, sizeof(coeffs));
    memset(state, 0, sizeof(state));
    memset(hist, 0, sizeof(hist));
    arm_lms_norm_init_f32(&lms, ANF_TAPS, coeffs, state, ANF_MU, 128);
  }

  audio_block_t* inputQueueArray[1];
  arm_lms_norm_instance_f32 lms;
  float coeffs[ANF_TAPS];
  float state[ANF_TAPS + 128 - 1];
  float hist[ANF_DELAY];
  bool  en = false;
};
