# Teensy 3.6 DSP Audio Filter

Filtru audio DSP pentru recepție pe transceiver clasic (telegrafie + fonie):
Teensy 3.6 + PJRC Audio Shield **Rev B** (SGTL5000) + TFT 2.4" ILI9341 SPI.

Lanț DSP: `LINE IN → FIR trece-bandă (160 taps) → biquad (îngustare CW) → LINE OUT/căști`,
cu spectru 0–4.3 kHz + waterfall pe display și banda filtrului marcată.

## Stare

- [x] Schelet funcțional: filtrare CW/SSB/bypass, spectru, waterfall, butoane + comenzi serial
- [ ] Reducere spectrală de zgomot Ephraim-Malah (referință: DD4WH Teensy-ConvolutionSDR)
- [ ] Auto-notch LMS pentru purtătoare (fonie)
- [ ] Noise blanker pentru impulsuri
- [ ] Decodor CW cu text pe ecran

## Cablare

### TFT ILI9341 (SPI alternativ — nu se bate cu audio shield-ul)

Modulul folosit e varianta cu 8 pini, **fără CS** (chip-select legat intern la
masă → display-ul e mereu selectat). Merge fără modificări de cod: pinul 21
declarat ca CS comută în gol, nu se leagă nicăieri.

| Pin display | Semnificație | Teensy 3.6 |
|-------------|-------------|-----------|
| 1 GND  | masă        | GND  |
| 2 VCC  | alimentare  | 3.3V |
| 3 CLK  | ceas SPI    | 14   |
| 4 MOSI | date intrare| 7    |
| 5 RES  | reset       | 16   |
| 6 DC   | date/comandă| 20   |
| 7 BLK  | backlight   | 3.3V |
| 8 MISO | date ieșire | nu se leagă |

MISO rămâne în aer (biblioteca nu citește nimic de pe display). Consecința
lipsei CS: **nu se poate folosi simultan SD-ul sau flash-ul de pe audio
shield** — orice trafic SPI ar ajunge și la display. Sketch-ul nostru nu le
folosește, deci nu ne afectează.

Fire scurte! SPI-ul lung induce zgomot în audio.

### Butoane (la GND, pull-up intern)

| Funcție | Pin |
|---------|-----|
| MODE — preset următor | 2 |
| NR — rezervat reducere zgomot | 3 |

### Encoder rotativ (KY-040)

| Encoder | Teensy |
|---------|--------|
| CLK (A) | 4 |
| DT (B)  | 5 |
| SW      | 8 |
| +       | 3.3V |
| GND     | GND |

Rotire = modifică parametrul selectat; apăsare = următorul parametru
(Volum → Preset → Gain intrare). Bara de meniu e jos pe ecran.

### Audio

- **LINE IN (L)** ← ieșirea de difuzor/căști a transceiverului. Atenție la nivel:
  de la ieșirea de difuzor folosește un divizor (ex. 10k/1k) ca să nu depășești
  ~1.5 Vpp. Ideal și un transformator de izolare 1:1 (600Ω) contra buclelor de masă.
- **Căști / LINE OUT** → căști sau difuzor amplificat.
- Semnalul se procesează pe canalul stâng, ieșirea e dusă pe ambele canale.

## Preseturi

| # | Nume | Bandă | Observații |
|---|------|-------|-----------|
| 1 | CW îngust | 550–750 Hz | FIR + 2×biquad bandpass |
| 2 | CW | 400–900 Hz | FIR + 2×biquad bandpass |
| 3 | SSB | 300–2700 Hz | FIR |
| 4 | SSB larg | 200–3100 Hz | FIR |
| 5 | BYPASS | — | passthrough |

Comenzi pe serial (115200): `1`–`5` = preset, `+`/`-` = volum căști,
`[`/`]` = gain intrare, `l` = nivele in/out, `f` = spectru 0–1.3 kHz,
`F` = top 5 frecvențe, `c` = analiză saturare 1 s.

Nivel intrare: gain implicit 0 (semnal mare) — AF OUT al FT-840 dă ~1 Vpp.
Ținta: VU-metrul din dreapta spectrului verde/galben, fără roșu.

## Compilare

```
arduino-cli compile --fqbn teensy:avr:teensy36 Teensy36_DSPFilter
arduino-cli upload -p <PORT> --fqbn teensy:avr:teensy36 Teensy36_DSPFilter
```

Necesită core-ul `teensy:avr` (Teensyduino ≥1.60); bibliotecile Audio, ILI9341_t3
și Bounce sunt incluse în core.

## Referințe

- DD4WH Teensy-ConvolutionSDR — reducere zgomot Ephraim-Malah:
  https://github.com/DD4WH/Teensy-ConvolutionSDR/wiki/Spectral--noise-reduction
- DSPham (LMS, notch, decodoare CW): https://github.com/grahamwhaley/DSPham
- GI1MIC $19 DSP filter (FIR runtime): https://gi1mic.github.io/
