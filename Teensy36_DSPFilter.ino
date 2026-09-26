// ============================================================
//  Teensy36_DSPFilter - filtru audio DSP pentru transceiver clasic
//  Teensy 3.6 + Audio Shield Rev B (SGTL5000) + TFT 2.4" ILI9341
//
//  Lant DSP:  LINE IN -> FIR trece-banda -> biquad (ingustare CW)
//             -> LINE OUT / casti
//  Afisaj:    spectru 0..4.3 kHz + waterfall, banda filtrului marcata
//  Comenzi:   buton MODE (pin 2)  = preset urmator (CW/SSB/BYPASS)
//             buton NR   (pin 3)  = rezervat reducere zgomot (TODO)
//             serial: '1'..'5' preset, '+'/'-' volum
//
//  Cablare TFT (SPI alternativ, ca sa nu se bata cu audio shield):
//    modul 8 pini FARA CS (mereu selectat): CLK=14  MOSI=7  DC=20
//    RES->3.3V  BLK->3.3V  VCC=3.3V  MISO neconectat
//    pinul 21 (CS in cod) comuta in gol - nu se leaga nicaieri
//    ATENTIE: fara CS nu se poate folosi SD/flash de pe audio shield!
//  Audio shield: LINE IN <- iesire difuzor/casti transceiver (atenuata!)
//                casti / LINE OUT -> difuzor amplificat
//
//  TODO (etapele urmatoare):
//    [ ] reducere spectrala de zgomot Ephraim-Malah (cf. DD4WH)
//    [ ] auto-notch LMS pentru purtatoare (fonie)
//    [ ] noise blanker impulsuri
//    [ ] decodor CW cu text pe ecran
// ============================================================

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include <ILI9341_t3.h>
#include <Bounce.h>
#include <Encoder.h>
#include <EEPROM.h>
#include "effect_spectral_nr.h"
#include "effect_auto_notch.h"

// ---------------- Pini ----------------
#define PIN_BTN_MODE 2   // buton la GND
#define PIN_BTN_NR   3   // buton la GND (rezervat)
#define PIN_ENC_A    4   // encoder CLK
#define PIN_ENC_B    5   // encoder DT
#define PIN_ENC_SW   8   // encoder buton (apasare = alt parametru)
#define TFT_CS   21
#define TFT_DC   20
#define TFT_RST  16      // RES pe pin controlat: reset hardware la fiecare boot
#define TFT_MOSI 7
#define TFT_SCK  14
#define TFT_MISO 12

ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC, TFT_RST, TFT_MOSI, TFT_SCK, TFT_MISO);

// ---------------- Graf audio ----------------
AudioInputI2S        i2sIn;
AudioFilterFIR       fir;        // filtrul principal trece-banda
AudioFilterBiquad    biquad;     // ingustare suplimentara pt CW
AudioEffectAutoNotch  anf;       // notch automat (inaintea NR)
AudioEffectSpectralNR nr;        // reducere spectrala de zgomot
AudioAnalyzePeak     peakOut;    // nivel iesire
AudioAnalyzePeak     peakIn;     // nivel intrare (VU + control saturare)
AudioRecordQueue     rawQueue;   // diagnostic: acces la mostrele brute
AudioOutputI2S       i2sOut;
AudioControlSGTL5000 codec;

AudioConnection c1(i2sIn, 0, fir, 0);        // canalul stang de la transceiver
AudioConnection c2(fir, 0, biquad, 0);
AudioConnection c3(biquad, 0, anf, 0);
AudioConnection c3b(anf, 0, nr, 0);
AudioConnection c4(nr, 0, i2sOut, 0);
AudioConnection c4b(nr, 0, i2sOut, 1);
AudioConnection c6(nr, 0, peakOut, 0);
AudioConnection c7(i2sIn, 0, peakIn, 0);
AudioConnection c8(i2sIn, 0, rawQueue, 0);

// ---------------- Spectru calculat in bucla principala ----------------
// FFT-ul de afisaj NU ruleaza in ISR-ul audio (economie ~25% CPU audio):
// mostrele vin continuu prin rawQueue, FFT 1024 se face la ~10 Hz aici.
int16_t  specRing[1024];
int      specIdx = 0;
float    specMag[512];        // amplitudini normalizate 0..1, 43.07 Hz/bin
float    hann1024[1024];
float    specBuf[2048];       // buffer complex pt cfft
arm_cfft_radix4_instance_f32 specFFT;
uint32_t tSpec = 0;

void computeSpectrum() {
  for (int i = 0; i < 1024; i++) {
    float v = specRing[(specIdx + i) & 1023] / 32768.0f * hann1024[i];
    specBuf[2 * i] = v;
    specBuf[2 * i + 1] = 0.0f;
  }
  arm_cfft_radix4_f32(&specFFT, specBuf);
  for (int k = 0; k < 512; k++) {
    float re = specBuf[2 * k], im = specBuf[2 * k + 1];
    specMag[k] = 4.0f * sqrtf(re * re + im * im) / 1024.0f;  // ~compatibil cu AudioAnalyzeFFT1024
  }
}

// stare captura diagnostic ('c' pe serial)
bool     capOn = false;
uint32_t capN = 0, capClip = 0;
int16_t  capMin = 32767, capMax = -32768;

// ---------------- Preseturi filtru ----------------
struct Preset {
  const char* name;
  float lo, hi;      // banda de trecere, Hz (0/0 = bypass)
};
const Preset presets[] = {
  {"CW ingust", 550,  750},
  {"CW",        400,  900},
  {"SSB",       300, 2700},
  {"SSB larg",  200, 3100},
  {"BYPASS",      0,    0},
};
const int NUM_PRESETS = sizeof(presets) / sizeof(presets[0]);
int preset = 2;  // pornim pe SSB

// banda curenta (poate fi modificata manual din encoder, pas 50 Hz);
// biquad-ul de ingustare intra automat cand banda e <= 600 Hz
float curLo = 300, curHi = 2700;
bool  manualBand = false;
bool  filtDirty = false;      // recalculare FIR amanata (debounce encoder)
uint32_t tFiltChange = 0;

// FIR: max 200 taps la AudioFilterFIR; 160 = compromis CPU/pante
#define FIR_TAPS 160
int16_t firCoeffs[FIR_TAPS];

float volume = 0.7f;
int   lineIn = 0;    // gain intrare SGTL5000 0..15; 0 = cel mai mare semnal
                     // acceptat (3.12Vpp) - AF OUT al FT-840 e ~1Vpp, altfel satureaza

Bounce btnMode(PIN_BTN_MODE, 15);
Bounce btnNr(PIN_BTN_NR, 15);
Bounce btnEnc(PIN_ENC_SW, 15);
Encoder enc(PIN_ENC_A, PIN_ENC_B);

// meniu: rotire = valoare, apasare = parametrul urmator
enum MenuItem { M_VOL = 0, M_PRESET, M_FLO, M_FHI, M_NR, M_NOTCH, M_GAIN, NUM_MENU };
const char* menuName[NUM_MENU] = {"Volum", "Preset", "Filtru jos", "Filtru sus", "NR", "Notch", "Gain in"};
int menuSel = M_VOL;
long encLast = 0;

// ---------------- Setari persistente (EEPROM) ----------------
// scrise la 3s dupa ultima modificare, ca sa nu uzam memoria la fiecare click
#define CFG_MAGIC 0x44535032   // "DSP2" (v2: + notch)
struct Settings {
  uint32_t magic;
  float    volume;
  int8_t   preset;
  float    lo, hi;
  bool     manual;
  int8_t   nrLevel;
  int8_t   lineIn;
  bool     notch;
};
bool     cfgDirty = false;
uint32_t tCfgChange = 0;

void markDirty() { cfgDirty = true; tCfgChange = millis(); }

void saveSettings() {
  Settings s = {CFG_MAGIC, volume, (int8_t)preset, curLo, curHi,
                manualBand, (int8_t)nr.getLevel(), (int8_t)lineIn, anf.enabled()};
  EEPROM.put(0, s);
  Serial.println("Setari salvate in EEPROM");
}

// intoarce true daca a incarcat setari valide
bool loadSettings() {
  Settings s;
  EEPROM.get(0, s);
  if (s.magic != CFG_MAGIC) return false;
  if (s.preset < 0 || s.preset >= NUM_PRESETS) return false;
  if (s.lineIn < 0 || s.lineIn > 15) return false;
  volume = constrain(s.volume, 0.0f, 1.0f);
  preset = s.preset;
  lineIn = s.lineIn;
  nr.setLevel(constrain(s.nrLevel, 0, 3));
  anf.setEnabled(s.notch);
  manualBand = s.manual;
  if (manualBand) {
    curLo = constrain(s.lo, 50.0f, 4900.0f);
    curHi = constrain(s.hi, curLo + 100, 5000.0f);
  }
  return true;
}

// ---------------- Proiectare FIR (sinc ferestruit Hamming) ----------------
// Trece-banda = diferenta a doua trece-jos; normalizat la castig 1 in
// centrul benzii, apoi scalat la int16 (formatul cerut de AudioFilterFIR).
void designBandpassFIR(int16_t* dest, int taps, float fLo, float fHi) {
  const float fs = AUDIO_SAMPLE_RATE_EXACT;   // 44117.6 Hz pe Teensy 3.x
  static float h[FIR_TAPS];
  const float M = (taps - 1) / 2.0f;

  for (int n = 0; n < taps; n++) {
    float k = n - M;
    float lp2 = (k == 0) ? 2 * fHi / fs : sinf(2 * PI * fHi * k / fs) / (PI * k);
    float lp1 = (k == 0) ? 2 * fLo / fs : sinf(2 * PI * fLo * k / fs) / (PI * k);
    float wnd = 0.54f - 0.46f * cosf(2 * PI * n / (taps - 1));  // Hamming
    h[n] = (lp2 - lp1) * wnd;
  }

  // castigul la frecventa centrala -> normalizare
  float fc = (fLo + fHi) / 2;
  float gr = 0, gi = 0;
  for (int n = 0; n < taps; n++) {
    gr += h[n] * cosf(2 * PI * fc * (n - M) / fs);
    gi += h[n] * sinf(2 * PI * fc * (n - M) / fs);
  }
  float g = sqrtf(gr * gr + gi * gi);
  for (int n = 0; n < taps; n++)
    dest[n] = (int16_t)constrain(32767.0f * h[n] / g, -32768.0f, 32767.0f);
}

// reconstruieste FIR + biquad pentru banda curenta curLo..curHi
void rebuildFilter() {
  AudioNoInterrupts();
  const double ident[5] = {1.0, 0.0, 0.0, 0.0, 0.0};
  if (curLo <= 0 || curHi <= 0) {
    fir.begin(FIR_PASSTHRU, 0);
    biquad.setCoefficients(0, ident);
    biquad.setCoefficients(1, ident);
  } else {
    designBandpassFIR(firCoeffs, FIR_TAPS, curLo, curHi);
    fir.begin(firCoeffs, FIR_TAPS);
    if (curHi - curLo <= 600) {
      // banda ingusta (CW): biquad-ul aduna panta in jurul centrului
      float fc = (curLo + curHi) / 2;
      float q  = fc / (curHi - curLo);
      biquad.setBandpass(0, fc, q);
      biquad.setBandpass(1, fc, q);
    } else {
      biquad.setCoefficients(0, ident);
      biquad.setCoefficients(1, ident);
    }
  }
  AudioInterrupts();
  drawHeader();
  drawFreqScale();
}

void applyPreset(int p) {
  const Preset& pr = presets[p];
  curLo = pr.lo;
  curHi = pr.hi;
  manualBand = false;
  rebuildFilter();
  Serial.printf("Preset: %s  %.0f-%.0f Hz\n", pr.name, pr.lo, pr.hi);
}

// ---------------- Afisaj ----------------
// Layout landscape 320x240:
//   0..23   antet: preset + banda + CPU
//   26..145 spectru (bare), 100 coloane x 3px = 300px, 0..4.3 kHz
//   146..157 scala de frecventa
//   160..239 waterfall (cursor circular)
#define SPEC_X0   10
#define SPEC_Y0   26
#define SPEC_H    120
#define SPEC_COLS 100     // FFT1024: 43.07 Hz/bin -> 100 bins = 4.3 kHz
#define COL_W     3
#define WF_Y0     160
#define WF_H      66     // lasa loc barei de meniu jos

int wfRow = 0;
uint8_t specVal[SPEC_COLS];

uint16_t heatColor(uint8_t v) {
  // negru -> albastru -> galben -> rosu
  if (v < 64)  return tft.color565(0, 0, v * 3);
  if (v < 160) return tft.color565((v - 64) * 2.6f, (v - 64) * 2.6f, 191 - (v - 64) * 2);
  return tft.color565(255, 255 - (v - 160) * 2.6f, 0);
}

void drawHeader() {
  tft.fillRect(0, 0, 320, 24, ILI9341_NAVY);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.setCursor(6, 4);
  tft.print(manualBand ? "MANUAL" : presets[preset].name);
  tft.setTextSize(1);
  tft.setCursor(140, 8);
  if (curLo > 0) tft.printf("%.0f-%.0f Hz", curLo, curHi);
  else           tft.print("fara filtru");
  if (nr.getLevel() > 0) {
    tft.setTextColor(ILI9341_GREEN);
    tft.setCursor(210, 8);
    tft.printf("NR%d", nr.getLevel());
  }
  if (anf.enabled()) {
    tft.setTextColor(ILI9341_YELLOW);
    tft.setCursor(184, 8);
    tft.print("AN");
  }
}

void drawCpu() {
  tft.fillRect(240, 0, 80, 24, ILI9341_NAVY);
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(1);
  tft.setCursor(244, 8);
  tft.printf("CPU %2.0f%%", AudioProcessorUsageMax());
  AudioProcessorUsageMaxReset();
}

void drawFreqScale() {
  tft.fillRect(0, 146, 320, 12, ILI9341_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_DARKGREY);
  for (int khz = 0; khz <= 4; khz++) {
    int x = SPEC_X0 + (int)(khz * 1000.0f / 43.07f * COL_W);
    if (x > SPEC_X0 + SPEC_COLS * COL_W) break;
    tft.drawFastVLine(x, 146, 4, ILI9341_DARKGREY);
    tft.setCursor(x - 2, 151);
    tft.print(khz);
  }
  // marcheaza banda filtrului sub scala
  if (curLo > 0) {
    int x1 = SPEC_X0 + (int)(curLo / 43.07f * COL_W);
    int x2 = SPEC_X0 + (int)(curHi / 43.07f * COL_W);
    tft.drawFastHLine(x1, 147, x2 - x1, ILI9341_GREEN);
    tft.drawFastHLine(x1, 148, x2 - x1, ILI9341_GREEN);
  }
}

void drawSpectrum() {
  int bLo = (curLo > 0) ? (int)(curLo / 43.07f) : -1;
  int bHi = (curLo > 0) ? (int)(curHi / 43.07f) : -1;

  for (int i = 0; i < SPEC_COLS; i++) {
    float v = specMag[i];
    // scala log: -70..0 dBFS -> 0..SPEC_H
    float db = 20.0f * log10f(v + 1e-6f);
    int h = constrain((int)((db + 70.0f) * SPEC_H / 70.0f), 0, SPEC_H);
    specVal[i] = (uint8_t)constrain((db + 70.0f) * 255.0f / 70.0f, 0.0f, 255.0f);

    int x = SPEC_X0 + i * COL_W;
    bool inBand = (i >= bLo && i <= bHi);
    uint16_t cBar = inBand ? ILI9341_YELLOW : ILI9341_DARKCYAN;
    uint16_t cBg  = inBand ? tft.color565(30, 30, 0) : ILI9341_BLACK;
    tft.fillRect(x, SPEC_Y0, COL_W - 1, SPEC_H - h, cBg);
    tft.fillRect(x, SPEC_Y0 + SPEC_H - h, COL_W - 1, h, cBar);
  }
}

// bara de meniu (jos): parametrul selectat + valoarea
void drawMenu() {
  tft.fillRect(0, 228, 320, 12, tft.color565(40, 40, 40));
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_YELLOW);
  tft.setCursor(4, 230);
  tft.print(menuName[menuSel]);
  tft.setTextColor(ILI9341_WHITE);
  tft.setCursor(70, 230);
  switch (menuSel) {
    case M_VOL:    tft.printf("%2.0f%%", volume * 100); break;
    case M_PRESET: tft.print(manualBand ? "MANUAL" : presets[preset].name); break;
    case M_FLO:    tft.printf("%.0f Hz", curLo); break;
    case M_FHI:    tft.printf("%.0f Hz", curHi); break;
    case M_NR:     if (nr.getLevel()) tft.printf("%d", nr.getLevel()); else tft.print("oprit"); break;
    case M_NOTCH:  tft.print(anf.enabled() ? "pornit" : "oprit"); break;
    case M_GAIN:   tft.printf("%d  (0=semnal mare)", lineIn); break;
  }
  tft.setTextColor(ILI9341_DARKGREY);
  tft.setCursor(200, 230);
  tft.print("apasa=alt parametru");
}

// VU-metru vertical pe marginea dreapta a spectrului (nivel INTRARE)
void drawVU() {
  if (!peakIn.available()) return;
  float p = peakIn.read();          // 0..1 din full scale
  int h = (int)(p * SPEC_H);
  uint16_t c = (p > 0.9f) ? ILI9341_RED : (p > 0.6f) ? ILI9341_YELLOW : ILI9341_GREEN;
  tft.fillRect(313, SPEC_Y0, 5, SPEC_H - h, ILI9341_BLACK);
  tft.fillRect(313, SPEC_Y0 + SPEC_H - h, 5, h, c);
}

void drawWaterfall() {
  int y = WF_Y0 + wfRow;
  for (int i = 0; i < SPEC_COLS; i++)
    tft.drawFastHLine(SPEC_X0 + i * COL_W, y, COL_W - 1, heatColor(specVal[i]));
  wfRow = (wfRow + 1) % WF_H;
  tft.drawFastHLine(SPEC_X0, WF_Y0 + wfRow, SPEC_COLS * COL_W, ILI9341_GREEN); // cursor
}

// ---------------- Setup / Loop ----------------
void setup() {
  Serial.begin(115200);
  pinMode(PIN_BTN_MODE, INPUT_PULLUP);
  pinMode(PIN_BTN_NR, INPUT_PULLUP);
  pinMode(PIN_ENC_SW, INPUT_PULLUP);

  bool loaded = loadSettings();   // inainte de configurarea codecului

  AudioMemory(24);
  codec.enable();
  codec.inputSelect(AUDIO_INPUT_LINEIN);
  codec.lineInLevel(lineIn);
  codec.volume(volume);        // casti
  codec.lineOutLevel(29);      // ~1.3 Vpp pe line out

  // spectrul de afisaj: FFT in bucla principala, nu in ISR audio
  arm_cfft_radix4_init_f32(&specFFT, 1024, 0, 1);
  for (int i = 0; i < 1024; i++)
    hann1024[i] = 0.5f * (1.0f - cosf(2.0f * PI * i / 1024));
  memset(specRing, 0, sizeof(specRing));
  rawQueue.begin();            // captura continua pt spectru + diagnostic

  delay(150);                  // panoul sa fie alimentat stabil inainte de reset
  tft.begin();
  tft.setRotation(1);          // landscape 320x240
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_CYAN);
  tft.setTextSize(2);
  tft.setCursor(40, 100);
  tft.print("Teensy 3.6 DSP Filter");
  delay(800);
  tft.fillScreen(ILI9341_BLACK);

  if (loaded && manualBand) {
    rebuildFilter();
    Serial.printf("Setari incarcate: MANUAL %.0f-%.0f Hz, NR %d\n", curLo, curHi, nr.getLevel());
  } else {
    applyPreset(preset);
    if (loaded) Serial.printf("Setari incarcate: %s, NR %d\n", presets[preset].name, nr.getLevel());
  }
  drawMenu();
}

// rotire encoder: aplica pasul pe parametrul selectat
void menuChange(int dir) {
  switch (menuSel) {
    case M_VOL:
      volume = constrain(volume + dir * 0.05f, 0.0f, 1.0f);
      codec.volume(volume);
      break;
    case M_PRESET:
      preset = (preset + dir + NUM_PRESETS) % NUM_PRESETS;
      applyPreset(preset);
      break;
    case M_FLO:
    case M_FHI:
      if (curHi <= 0) { curLo = 300; curHi = 2700; }  // iesire din bypass
      if (menuSel == M_FLO) curLo = constrain(curLo + dir * 50, 50.0f, curHi - 100);
      else                  curHi = constrain(curHi + dir * 50, curLo + 100, 5000.0f);
      manualBand = true;
      filtDirty = true;             // recalculam dupa ce se opreste rotirea
      tFiltChange = millis();
      drawHeader();
      break;
    case M_NR:
      nr.setLevel(nr.getLevel() + dir);
      drawHeader();
      break;
    case M_NOTCH:
      anf.setEnabled(dir > 0);
      drawHeader();
      break;
    case M_GAIN:
      lineIn = constrain(lineIn + dir, 0, 15);
      codec.lineInLevel(lineIn);
      break;
  }
  markDirty();
  drawMenu();
}

uint32_t tCpu = 0;

void loop() {
  // butoane
  if (btnMode.update() && btnMode.fallingEdge()) {
    preset = (preset + 1) % NUM_PRESETS;
    applyPreset(preset);
    markDirty();
  }
  if (btnNr.update() && btnNr.fallingEdge()) {
    nr.setLevel((nr.getLevel() + 1) % 4);   // OFF -> 1 -> 2 -> 3 -> OFF
    Serial.printf("NR: %d\n", nr.getLevel());
    drawHeader();
    if (menuSel == M_NR) drawMenu();
    markDirty();
  }

  // encoder: 4 pasi/detent la KY-040
  long ep = enc.read() / 4;
  if (ep != encLast) {
    menuChange(ep > encLast ? 1 : -1);
    encLast = ep;
  }
  if (btnEnc.update() && btnEnc.fallingEdge()) {
    menuSel = (menuSel + 1) % NUM_MENU;
    drawMenu();
  }

  // comenzi seriale pt teste fara butoane
  if (Serial.available()) {
    char c = Serial.read();
    if (c >= '1' && c <= '0' + NUM_PRESETS) { preset = c - '1'; applyPreset(preset); }
    if (c == 'n') { nr.setLevel((nr.getLevel() + 1) % 4); Serial.printf("NR: %d\n", nr.getLevel()); drawHeader(); }
    if (c == '+') { volume = min(volume + 0.05f, 1.0f); codec.volume(volume); Serial.printf("Volum: %.2f\n", volume); }
    if (c == '-') { volume = max(volume - 0.05f, 0.0f); codec.volume(volume); Serial.printf("Volum: %.2f\n", volume); }
    if (c == ']') { lineIn = min(lineIn + 1, 15); codec.lineInLevel(lineIn); Serial.printf("LineIn gain: %d\n", lineIn); }
    if (c == '[') { lineIn = max(lineIn - 1, 0);  codec.lineInLevel(lineIn); Serial.printf("LineIn gain: %d\n", lineIn); }
    if (c == 'a') { anf.setEnabled(!anf.enabled()); Serial.printf("Notch: %s\n", anf.enabled() ? "pornit" : "oprit"); drawHeader(); }
    if (c && strchr("12345+-[]na", c)) markDirty();
    if (c == 's') Serial.printf("Setari: %s %.0f-%.0f Hz, vol %.2f, NR %d, notch %d, gain %d%s\n",
                                manualBand ? "MANUAL" : presets[preset].name, curLo, curHi,
                                volume, nr.getLevel(), anf.enabled(), lineIn, cfgDirty ? " (nesalvate inca)" : "");
    if (c == 'R') { Serial.println("Reboot..."); Serial.flush(); delay(50); SCB_AIRCR = 0x05FA0004; }
    if (c == 'l') Serial.printf("Nivel in: %.2f  out: %.2f\n", peakIn.read(), peakOut.read());
    if (c == 'u') {  // repartitia CPU pe obiecte audio (max de la ultima citire)
      Serial.printf("CPU total: %.1f%% (max %.1f%%)\n", AudioProcessorUsage(), AudioProcessorUsageMax());
      Serial.printf("  fir=%d%%  biquad=%d%%  anf=%d%%  nr=%d%%\n",
                    fir.processorUsageMax(), biquad.processorUsageMax(),
                    anf.processorUsageMax(), nr.processorUsageMax());
      fir.processorUsageMaxReset(); biquad.processorUsageMaxReset();
      anf.processorUsageMaxReset(); nr.processorUsageMaxReset();
      AudioProcessorUsageMaxReset();
    }
    if (c == 'f') {  // diagnostic: primele 30 bin-uri FFT (0..1290 Hz, pas 43 Hz)
      for (int i = 0; i < 30; i++) Serial.printf("%4.0fHz %.3f\n", i * 43.07f, specMag[i]);
    }
    if (c == 'F') {  // diagnostic: top 5 bin-uri din tot spectrul (0..22kHz)
      float top[5] = {0}; int topI[5] = {0};
      for (int i = 0; i < 512; i++) {
        float v = specMag[i];
        for (int k = 0; k < 5; k++)
          if (v > top[k]) {
            for (int m = 4; m > k; m--) { top[m] = top[m-1]; topI[m] = topI[m-1]; }
            top[k] = v; topI[k] = i;
            break;
          }
      }
      for (int k = 0; k < 5; k++)
        Serial.printf("%5.0f Hz  %.3f\n", topI[k] * 43.07f, top[k]);
    }
    if (c == 'c') {  // diagnostic: analiza 1s de mostre brute
      capN = capClip = 0; capMin = 32767; capMax = -32768;
      capOn = true;
    }
  }

  // spectru + waterfall la ~10 Hz, calculat aici, nu in ISR audio
  if (millis() - tSpec >= 100) {
    tSpec = millis();
    computeSpectrum();
    drawSpectrum();
    drawWaterfall();
    drawVU();
  }

  // salvare setari, amanata 3s dupa ultima modificare (protejam EEPROM-ul)
  if (cfgDirty && millis() - tCfgChange > 3000) {
    cfgDirty = false;
    saveSettings();
  }

  // recalcularea filtrului, amanata pana se opreste rotirea encoderului
  if (filtDirty && millis() - tFiltChange > 150) {
    filtDirty = false;
    rebuildFilter();
    Serial.printf("Filtru manual: %.0f-%.0f Hz\n", curLo, curHi);
  }

  // dreneaza mostrele capturate: alimenteaza inelul de spectru
  // (+ diagnosticul de saturare cand e activ)
  while (rawQueue.available()) {
    int16_t* b = rawQueue.readBuffer();
    memcpy(&specRing[specIdx], b, 128 * sizeof(int16_t));
    specIdx = (specIdx + 128) & 1023;
    if (capOn) {
      for (int i = 0; i < 128; i++) {
        if (b[i] < capMin) capMin = b[i];
        if (b[i] > capMax) capMax = b[i];
        if (b[i] > 32000 || b[i] < -32000) capClip++;
      }
      capN += 128;
      if (capN >= 44100) {
        capOn = false;
        Serial.printf("1s raw: min=%d max=%d saturate=%lu din %lu (%.2f%%)\n",
                      capMin, capMax, capClip, capN, 100.0f * capClip / capN);
      }
    }
    rawQueue.freeBuffer();
  }

  if (millis() - tCpu > 2000) { tCpu = millis(); drawCpu(); }
}
