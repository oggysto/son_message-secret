# Référence du Cours - Audio Embarqué sur Teensy

Ce document résume **uniquement les connaissances théoriques et pratiques enseignées durant le cours** (basé sur `contexte.txt`). Il sert de référence pour comprendre les concepts audio embarqués et guider la réalisation de votre projet.

> ⚠️ **Note importante** : Ce guide ne contient que les informations du cours magistral. Les exemples de code de projets spécifiques ne sont pas inclus ici.

## 🎯 Méthodologie de Travail Recommandée

**Workflow de développement conseillé** :

1. **Prototypage en Faust** 🧪
   - Coder et tester rapidement vos algorithmes DSP en Faust
   - Utiliser l'IDE en ligne pour itérations rapides
   - Tester différentes configurations sans recompiler

2. **Export vers C++** ⚙️
   - Une fois l'algorithme validé, compiler en C++ pour Teensy
   - Intégrer le code généré dans votre projet PlatformIO
   - Optimiser si nécessaire

3. **Déploiement sur Teensy** 🎛️
   - Upload du code C++ sur le matériel
   - Tests en conditions réelles avec capteurs/contrôles

**Avantage** : Faust permet de tester et itérer rapidement sans cycle upload/test sur hardware.

---

## 📋 Table des matières

1. [Plateforme Matérielle](#plateforme-matérielle)
2. [Environnement de Développement](#environnement-de-développement)
3. [Fondamentaux Audio Numérique](#fondamentaux-audio-numérique)
4. [Architecture Système Audio](#architecture-système-audio)
5. [Synthèse Sonore](#synthèse-sonore)
6. [Traitement Audio](#traitement-audio)
7. [Contrôle Matériel](#contrôle-matériel)
8. [Configuration Audio Codec](#configuration-audio-codec)
9. [Faust](#faust)
10. [Ressources Avancées](#ressources-avancées)

---

## 🔧 Plateforme Matérielle

### Le Teensy 4.0

Le **Teensy 4.0** est une carte de développement basée sur un microcontrôleur développée par PJRC (environnement open source).

#### Spécifications Techniques

- **Processeur**: ARM Cortex-M7 @ 600 MHz
- **Mémoire Flash**: 2 MB (stockage du code)
- **RAM**: 1 MB total
  - **RAM1** (512 KB): Mémoire ultra-rapide (tightly coupled memory)
  - **RAM2** (512 KB): Optimisée pour DMA (Direct Memory Access)
- **FPU**: Unité flottante matérielle (32-bit float, 64-bit double)
- **DSP Extensions**: Instructions accélérées pour traitement de signal (filtres, FFT, etc.)
- **Performances**: Plusieurs fois plus rapide que les microcontrôleurs 32-bit classiques

#### ⚠️ Contraintes Électriques CRITIQUES

- **Tension de fonctionnement**: 3.3V UNIQUEMENT
- **Pins NON tolérants 5V**: Ne JAMAIS appliquer plus de 3.3V sur les pins digitales
- **Risque**: Destruction du microcontrôleur si 5V appliqué

#### Mappage Mémoire

Les variables peuvent être placées dans différentes zones mémoire via directives compilateur:

- `DMAMEM`: Place en RAM2 (pour gros tableaux/buffers)
- `FASTRUN`: Place en RAM1 (pour variables critiques en temps)

#### Pinout

- **Total**: 40 pins I/O
- **Accessibles sur breadboard**: 24 pins
- **Types de pins**:
  - GPIO (digital ou analog)
  - Protocoles intégrés: I2C, I2S, CAN, SPI, UART
  - Pins analogiques: Notés A(N) sur le schéma

#### Carte de Référence Pinout

**Important**: Une carte pinout est fournie avec le Teensy - **ne pas la perdre !**

- Coin supérieur droit: Pin 3.3V (alimentation)
- Coin supérieur gauche: Pin GND (masse)
- Pin Vin (5.5V): **NE JAMAIS CONNECTER** aux autres pins !

---

### Audio Shield (Rev. D)

Le shield audio PJRC intègre un codec audio stéréo de qualité professionnelle et un lecteur carte SD.

#### Codec SGTL5000 (NXP Semiconductors)

- **Type**: Codec audio stéréo basse consommation
- **Résolution**: 24-bit
- **Fréquences d'échantillonnage**: 8 kHz à 96 kHz
- **Architecture**: ADC + DAC stéréo intégrés

#### Schéma de Connexion

**Protocoles utilisés**:

1. **I2C** (Inter-Integrated Circuit)
   - Pins: SDA (18), SCL (19)
   - Usage: **Configuration** du codec (sample rate, routing, gains, etc.)
2. **I2S** (Inter-IC Sound)
   - Pins: TX (transmission vers codec), RX (réception depuis codec)
   - Clocks:
     - LRCLK: 44.1 kHz (Left/Right Clock)
     - BCLK: 1.41 MHz (Bit Clock)
     - MCLK: 11.29 MHz (Master Clock)
   - Usage: **Transfert audio** bit-à-bit bidirectionnel
   - Mode: SGTL5000 en "slave mode" (Teensy génère les clocks)

#### Entrées Audio

- **Line In** (stéréo): Entrée directe vers ADC (pins soudables)
- **Mic In** (mono): Microphone avec préampli intégré (pins soudables)
  - Gain préampli ajustable indépendamment

#### Sorties Audio

- **Line Out**: Sortie DAC (pins soudables)
- **Headphone Out**: Sortie amplifiée (jack 3.5mm)

#### Pins Réservés par le Shield

⚠️ **Ces pins NE peuvent PAS être utilisées pour des capteurs externes**:

- SDA (18), SCL (19): I2C
- TX, RX, LRCLK, BCLK, MCLK: I2S
- GND, 3.3V: Alimentation

**Pins disponibles pour capteurs**: Tous les autres (notamment A0-A13 pour analogique)

---

## 🖥️ Environnement de Développement

---

## 🏗️ Architecture Logicielle

### Environnement de Développement

- **PlatformIO** ou Arduino IDE + Teensyduino
- **Teensy Audio Library**: Framework audio de PJRC
- **Faust**: Langage DSP de haut niveau (optionnel)

### Anatomie d'un Programme Audio Teensy

#### 1. Setup de Base (main.cpp)

```cpp
#include <Audio.h>
#include "MyDsp.h"

// Déclaration des objets audio
MyDsp myDsp;                          // Objet DSP custom
AudioOutputI2S out;                   // Sortie I2S
AudioControlSGTL5000 audioShield;     // Contrôleur codec

// Connections audio (graph)
AudioConnection patchCord0(myDsp, 0, out, 0);  // Left
AudioConnection patchCord1(myDsp, 1, out, 1);  // Right

void setup() {
  Serial.begin(9600);
  AudioMemory(20);              // ⚠️ Allouer suffisamment de blocs (minimum 10-20)
  audioShield.enable();         // Initialiser le codec
  audioShield.volume(0.5);      // Volume (0.0-1.0)
}

void loop() {
  // Contrôle à faible fréquence (UI, capteurs, paramètres)
  float sensorValue = analogRead(A0);
  myDsp.setFreq(sensorValue);
  delay(10);
}
```

#### 2. Classe Audio DSP (MyDsp.h)

```cpp
#include <Arduino.h>
#include <AudioStream.h>

class MyDsp : public AudioStream {
public:
  MyDsp();
  void setFreq(float freq);
  void setGain(float gain);
  virtual void update(void);    // ⚠️ CALLBACK AUDIO (appelé ~344 fois/sec)

private:
  // Objets DSP
  // Paramètres
};
```

#### 3. Audio Callback (MyDsp.cpp)

```cpp
#define MULT_16 32767

void MyDsp::update(void) {
  // Allocation de blocs audio pour chaque canal
  audio_block_t* outBlock[AUDIO_OUTPUTS];
  for (int channel = 0; channel < AUDIO_OUTPUTS; channel++) {
    outBlock[channel] = allocate();
    if (!outBlock[channel]) return;  // Échec allocation
  }

  // BOUCLE AUDIO RATE (128 samples)
  for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
    // Génération/traitement audio
    float sample = /* algorithme DSP */;

    // Anti-clipping: signal DOIT rester dans [-1, 1]
    sample = max(-1.0f, min(1.0f, sample));

    // Conversion float → int16 pour I2S
    int16_t val = (int16_t)(sample * MULT_16);

    outBlock[0]->data[i] = val;  // Left
    outBlock[1]->data[i] = val;  // Right
  }

  // Transmission des blocs
  for (int channel = 0; channel < AUDIO_OUTPUTS; channel++) {
    transmit(outBlock[channel], channel);
    release(outBlock[channel]);  // ⚠️ Libérer la mémoire
  }
}
```

### Concepts Clés

#### Audio Rate vs Control Rate

- **Audio Rate**: À l'intérieur de `for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++)`
  - 44100 samples/sec
  - Code ultra-optimisé
  - Pas d'allocations dynamiques
- **Control Rate**: À l'extérieur de la boucle (ou dans `loop()`)
  - ~344 fois/sec (44100/128)
  - Mise à jour des paramètres
  - Lecture capteurs, UI

#### Taille de Bloc et Latence

- **Block Size** (`AUDIO_BLOCK_SAMPLES`): 128 samples par défaut
- **Fréquence d'appel**: fs / BS = 44100 / 128 ≈ 344 Hz
- **Latence**: BS / fs = 128 / 44100 ≈ 2.9 ms

#### Gestion Mémoire Audio

```cpp
AudioMemory(20);  // Alloue 20 blocs de 128 samples chacun
```

- **Trop peu**: Coupures audio (glitches)
- **Trop**: Gaspillage de RAM
- **Recommandation**: 10-20 blocs minimum

---

## 🎵 Fondamentaux Audio Numérique

### Échantillonnage (Sampling)

#### Théorème de Shannon-Nyquist

- **Fréquence de Nyquist**: fn = fs/2
- Pour capturer une fréquence fo, il faut fs ≥ 2×fo
- **Audition humaine**: 20 Hz - 20 kHz → fs ≥ 40 kHz
- **Standard**: 44.1 kHz (CD) ou 48 kHz (pro)

#### Sur Teensy

```cpp
AUDIO_SAMPLE_RATE_EXACT  // 44117.64706 Hz (réel)
```

#### Aliasing

- **Problème**: Fréquences > fn "replient" dans le spectre
- **Formule**: f_repliée = fn - (fo - fn)
- **Solution**: Filtre anti-aliasing avant ADC

### Bit Depth et Plage Dynamique

- **Teensy Audio Library**: 16-bit signés (compatibilité I2S)
- **DSP interne**: `float` (32-bit) pour précision
- **Plage standard**: **[-1.0, 1.0]** pour les floats

#### Conversion Float ↔ Int16

```cpp
// Float [-1, 1] → Int16 [-32767, 32767]
int16_t val = (int16_t)(floatSample * 32767);

// Int16 → Float
float floatSample = (float)val / 32767.0f;
```

---

## 🎹 Synthèse Sonore

### 1. Oscillateurs de Base

#### Phasor (Rampe/Sawtooth)

```cpp
class Phasor {
  float phase = 0.0f;
  float increment;

  void setFrequency(float freq) {
    increment = freq / AUDIO_SAMPLE_RATE_EXACT;
  }

  float tick() {
    phase += increment;
    if (phase >= 1.0f) phase -= 1.0f;
    return phase;  // [0, 1]
  }
};
```

**Usage**: Oscillateur saw, lecture de wavetable

#### Sine Wave (avec Wavetable)

```cpp
#define SINE_TABLE_SIZE 16384  // 2^14

float sineTable[SINE_TABLE_SIZE];

// Initialisation (une seule fois)
for (int i = 0; i < SINE_TABLE_SIZE; i++) {
  sineTable[i] = sin(2.0 * PI * i / SINE_TABLE_SIZE);
}

// Lecture
float tick() {
  int index = phasor.tick() * SINE_TABLE_SIZE;
  return sineTable[index];  // [-1, 1]
}
```

**Avantage**: Pas de `sin()` coûteux à chaque sample

#### Formes d'Ondes Classiques

- **Sine**: Son pur (1 harmonique)
- **Sawtooth**: `phasor.tick() * 2 - 1` → [-1, 1]
- **Square**: Basculement entre -1 et 1
- **Triangle**: Variation linéaire

### 2. Synthèse Additive

**Principe**: Somme de plusieurs sinusoïdes

```cpp
float tick() {
  float out = 0.0f;
  for (int i = 0; i < numOsc; i++) {
    out += oscillators[i].tick() * gains[i];
  }
  return out / numOsc;  // Normalisation anti-clipping
}
```

**Exemple 2 oscillateurs (harmonie à la quinte)**:

```cpp
int index1 = phasor.tick() * SINE_TABLE_SIZE;
int index2 = (index1 * 1.5) % SINE_TABLE_SIZE;  // Fréquence × 1.5
return (sineTable[index1] + sineTable[index2] * 0.5) * 0.5;
```

### 3. Modulation d'Amplitude (AM)

**Équation**: `carrier × (1 + modulator × index)`

```cpp
float tick() {
  float mod = phasor_mod.tick() * SINE_TABLE_SIZE;
  float modSig = (sineTable[(int)mod] + 1.0f) * 0.5;  // [0, 1]

  float car = phasor_car.tick() * SINE_TABLE_SIZE;
  return sineTable[(int)car] * modSig;
}
```

**Effets**:

- **Mod < 20 Hz**: Tremolo (battements audibles)
- **Mod ≥ 20 Hz**: Sidebands à fc ± fm

### 4. Modulation de Fréquence (FM)

**Équation**: `carrier_freq = fc + sin(2πfm×t) × index`

```cpp
float tick() {
  // Modulateur
  float modSig = sineTable[(int)(phasor_mod.tick() * SINE_TABLE_SIZE)];

  // Modulation de la fréquence porteuse
  phasor_car.setFrequency(carrierFreq + modSig * modIndex);

  // Porteuse
  return sineTable[(int)(phasor_car.tick() * SINE_TABLE_SIZE)];
}
```

**Paramètres**:

- **Carrier freq (fc)**: Fréquence de base
- **Modulator freq (fm)**: Souvent ratio de fc (0.5×, 1×, 1.5×, 2×...)
- **Modulation Index**: Contrôle la richesse harmonique (peut > 1)

**FM à 3 oscillateurs (style DX7)**:

```
Osc3 → modifie Osc2 → modifie Osc1 (sortie)
```

### 5. Karplus-Strong (Modèle Physique Simplifié)

**Principe**: Modèle de corde vibrante

```cpp
// Excitation initiale
for (int i = 0; i < delayLength; i++) {
  buffer[i] = (rand() / (float)RAND_MAX) * 2 - 1;  // Bruit
}

// Boucle de feedback
float tick() {
  float output = buffer[readIndex];

  // Filtre passe-bas (moyenne)
  float avg = (output + prevOutput) * 0.5;
  buffer[writeIndex] = avg * feedback;

  prevOutput = output;
  readIndex = (readIndex + 1) % delayLength;
  writeIndex = (writeIndex + 1) % delayLength;

  return output;
}
```

**Fréquence**: `f = fs / delayLength`

---

## 🎛️ Traitement Audio

### 1. Bruit Blanc (White Noise)

```cpp
float tick() {
  return (rand() / (float)RAND_MAX) * 2.0f - 1.0f;  // [-1, 1]
}
```

**Spectre**: Plat (toutes fréquences à même niveau)

### 2. Filtre One Zero (FIR)

**Équation**: `y(n) = b0×x(n) + b1×x(n-1)`

```cpp
float tick(float input) {
  float output = (input + b1 * prev) * 0.5;
  prev = input;
  return output;
}
```

**Comportement**:

- **b1 > 0**: Lowpass (atténue aigus)
- **b1 < 0**: Highpass (atténue graves)

### 3. Echo / Delay

**Équation**: `y(n) = x(n) + feedback × y(n-delay)`

```cpp
class Echo {
  float* buffer;
  int delaySize, readIndex, writeIndex;
  float feedback;

  float tick(float input) {
    float delayed = buffer[readIndex];
    buffer[writeIndex] = input + delayed * feedback;

    readIndex = (readIndex + 1) % delaySize;
    writeIndex = (writeIndex + 1) % delaySize;

    return input + delayed;  // Wet + dry
  }
};
```

**Paramètres**:

- **Delay**: Temps en samples (ex: 44100 = 1 sec)
- **Feedback**: [0, 1] (attention instabilité si ≥ 1)

### 4. Filtre Comb (Résonateur)

- **Echo court** (< 50ms) → résonances spectrales
- **Usage**: Réverbération, modélisation corps résonants

### 5. Distortion (Harmonique)

**Polynôme cubique** (arrondit les clips):

```cpp
float cubic(float x) {
  if (x <= -1.0f) return -2.0f/3.0f;
  if (x >= 1.0f) return 2.0f/3.0f;
  return x - (x*x*x) / 3.0f;
}

float tick(float input) {
  float driven = input * drive;  // drive = [1, 100]
  driven = max(-1.0f, min(1.0f, driven + offset));
  return cubic(driven);
}
```

### 6. Smoothing (Leaky Integrator)

**Équation**: `y(n) = (1-s)×x(n) + s×y(n-1)`

```cpp
float smooth(float input) {
  output = (1.0f - smoothCoeff) * input + smoothCoeff * output;
  return output;
}
```

**Coefficient s**:

- **0.0**: Pas de lissage
- **0.999**: Très lisse, lent
- **Usage**: Éviter clicks lors de changements de paramètres

---

## 🕹️ Contrôle Matériel

### 1. Potentiomètre Rotatif

#### Montage

```
3.3V ----[POT]---- GND
           |
           A0 (Teensy)
```

#### Lecture

```cpp
void setup() {
  pinMode(A0, INPUT);
}

void loop() {
  int rawValue = analogRead(A0);  // [0, 1023] (10-bit ADC)
  float normalized = rawValue / 1023.0f;  // [0.0, 1.0]

  // Mapping vers paramètre DSP
  float freq = 100.0f + normalized * 900.0f;  // [100, 1000] Hz
  myDsp.setFreq(freq);

  delay(10);  // Éviter surcharge
}
```

### 2. Bouton avec Pulldown Resistor

#### Montage

```
3.3V ----[Bouton]---- Pin Digital (ex: 0)
                       |
                      [10kΩ] (pulldown)
                       |
                      GND
```

#### Debouncing Logiciel

```cpp
bool lastState = LOW;
bool currentState;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;  // ms

void loop() {
  int reading = digitalRead(0);

  if (reading != lastState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != currentState) {
      currentState = reading;

      if (currentState == HIGH) {
        // Action au press
        myDsp.toggleGate();
      }
    }
  }

  lastState = reading;
}
```

### 3. MIDI via USB

```cpp
void setup() {
  usbMIDI.setHandleNoteOn(OnNoteOn);
  usbMIDI.setHandleNoteOff(OnNoteOff);
}

void loop() {
  usbMIDI.read();  // Polling MIDI
}

void OnNoteOn(byte channel, byte note, byte velocity) {
  float freq = 440.0f * pow(2.0f, (note - 69) / 12.0f);
  myDsp.setFreq(freq);
  myDsp.setGain(velocity / 127.0f);
}

void OnNoteOff(byte channel, byte note, byte velocity) {
  myDsp.setGain(0.0f);
}
```

**Conversion MIDI → Fréquence**:

```cpp
float mtof(int midiNote) {
  return 440.0f * pow(2.0f, (midiNote - 69) / 12.0f);
}
```

### 4. Configuration Codec SGTL5000

#### Registres I2C

Le codec est configuré via I2C avec des paires registre/valeur (16-bit).

**Exemple** (voir `control_sgtl5000.cpp`):

```cpp
AudioControlSGTL5000 audioShield;

void setup() {
  audioShield.enable();          // Init par défaut
  audioShield.volume(0.5);       // Volume casque [0.0, 1.0]
  audioShield.inputSelect(AUDIO_INPUT_LINEIN);  // ou AUDIO_INPUT_MIC
  audioShield.lineInLevel(5);    // Gain entrée ligne [0-15]
  audioShield.micGain(20);       // Gain micro [0-63] dB
}
```

**Registres principaux** (datasheet SGTL5000):

- `0x0010`: Volume DAC L/R
- `0x0020`: Contrôle ADC
- `0x0024`: Sélection source entrée

---

## 🎼 Faust (Langage DSP)

> 💡 **Approche recommandée pour le projet** :  
> Faust est l'outil privilégié pour **prototyper et tester rapidement** vos algorithmes DSP. Une fois validés, vous exportez en C++ pour déploiement sur Teensy.

### Workflow Faust → Teensy

```
┌─────────────────────┐
│  1. CODER EN FAUST  │  ← Développement rapide
│   (FaustIDE online) │     Tests instantanés
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│  2. TESTER & ITÉRER │  ← Ajuster paramètres
│   (navigateur web)  │     Valider algorithme
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│  3. EXPORTER C++    │  ← Compilation Faust
│   (faust -i -a ...) │     Génération code
└──────────┬──────────┘
           │
           ▼
┌─────────────────────┐
│ 4. INTÉGRER TEENSY  │  ← MyDsp.cpp/h
│   (PlatformIO)      │     Upload hardware
└─────────────────────┘
```

**Avantages** :

- ✅ Pas besoin d'upload à chaque modification
- ✅ Tests audio instantanés dans le navigateur
- ✅ Bibliothèques DSP riches (filtres, effets, synthèse)
- ✅ Code C++ optimisé généré automatiquement

---

### Génération C++ depuis Faust

#### 1. Fichier Faust (exemple: `FaustSynth.dsp`)

```faust
import("stdfaust.lib");

freq = nentry("freq", 440, 50, 2000, 0.01);
gain = nentry("gain", 0.5, 0, 1, 0.01) : si.smoo;
gate = button("gate") : si.smoo;

process = os.sawtooth(freq) * gain * gate <: _,_;
```

#### 2. Compilation

```bash
faust -i -a faustMinimal.h FaustSynth.dsp -o FaustSynth.h
```

- `-i`: Inline les includes
- `-a`: Architecture file (wrapper C++)

#### 3. Utilisation dans Teensy

```cpp
#include "FaustSynth.h"

class MyDsp : public AudioStream {
  mydsp* fDSP;
  MapUI* fUI;
  float** outputs;

public:
  MyDsp() : AudioStream(2, new audio_block_t*[2]) {
    fDSP = new mydsp();
    fDSP->init(AUDIO_SAMPLE_RATE_EXACT);

    fUI = new MapUI();
    fDSP->buildUserInterface(fUI);

    outputs = new float*[2];
    for (int i = 0; i < 2; i++) {
      outputs[i] = new float[AUDIO_BLOCK_SAMPLES];
    }
  }

  void setFreq(float f) {
    fUI->setParamValue("freq", f);
  }

  void update() {
    fDSP->compute(AUDIO_BLOCK_SAMPLES, nullptr, outputs);

    // Conversion float → int16 et transmission...
  }
};
```

### Bibliothèques Faust Utiles

- `os.sawtooth(freq)`: Dent de scie
- `os.triangle(freq)`: Triangle
- `os.square(freq)`: Carrée
- `no.noise`: Bruit blanc
- `fi.lowpass(order, cutoff)`: Filtre passe-bas
- `pf.flanger_mono`: Flanger
- `si.smoo`: Lissage (smoothing)

---

## ✅ Bonnes Pratiques

### 1. Gestion de la Plage Audio

```cpp
// ⚠️ TOUJOURS clipper avant conversion int16
float sample = /* calcul DSP */;
sample = max(-1.0f, min(1.0f, sample));
```

### 2. Éviter les Clippings

- **Additive**: Diviser par nombre d'oscillateurs
- **Echo**: Vérifier feedback < 1
- **Mixing**: Multiplier par gains < 1

### 3. Optimisation Mémoire

- **Réutiliser** les wavetables (ne pas dupliquer)
- **Allouer** dans constructeur, **libérer** dans destructeur
- **Tableaux statiques** plutôt que dynamiques si taille connue

### 4. Debugging

```cpp
// Serial Monitor
void setup() {
  Serial.begin(9600);
  while (!Serial && millis() < 3000);  // Attente connexion
}

void loop() {
  Serial.print("Param: ");
  Serial.println(value);
}
```

**PlatformIO**: Ajouter dans `platformio.ini`

```ini
monitor_speed = 9600
```

### 5. Allocation Mémoire Audio

**Règle empirique**:

- Base: 10 blocs
- +2 par objet audio
- +5 si reverb/delay longs

```cpp
AudioMemory(20);  // Pour ~5 objets DSP
```

### 6. Éviter les Opérations Lourdes dans update()

**❌ À éviter**:

- `sin()`, `cos()`, `sqrt()` → Utiliser wavetables
- `Serial.print()` → Faire dans `loop()`
- Allocations dynamiques (`new`, `malloc`)

**✅ Bon**:

- Lookups dans tableaux
- Opérations arithmétiques simples
- Accès mémoire préallouée

### 7. Anti-Aliasing

Pour signaux à large bande (sawtooth, square), utiliser:

- **Oversampling** + downsampling
- **Band-limited** algorithms (BLIT, PolyBLEP)
- **Wavetables** précalculées

---

## 📚 Ressources Complémentaires

### Documentation

- [PJRC Teensy Audio Library](https://www.pjrc.com/teensy/td_libs_Audio.html)
- [SGTL5000 Datasheet](https://www.nxp.com/docs/en/data-sheet/SGTL5000.pdf)
- [Julius O. Smith - DSP Online](https://ccrma.stanford.edu/~jos/)
- [Faust Documentation](https://faustdoc.grame.fr)

### Exemples Teensy

Dans Arduino IDE: `File → Examples → Audio`

- WavFilePlayer (lecture SD)
- MemoryAndCpuUsage
- Effects (chorus, flanger, etc.)

### Outils

- [MIDI Note → Frequency Calculator](https://djip.co/blog/logic-studio-9-midi-note-numbers)
- [Hex ↔ Binary Converter](https://www.rapidtables.com/convert/number/hex-to-binary.html)

---

## 🎯 Checklist Projet

### Avant de Commencer

- [ ] Hardware assemblé (Teensy + Audio Shield)
- [ ] PlatformIO configuré (`platformio.ini`)
- [ ] `AudioMemory()` ajusté
- [ ] Serial Monitor fonctionnel
- [ ] Test audio basique (bip)

### Pendant le Développement

- [ ] Paramètres dans `[-1, 1]` avant output
- [ ] Pas d'opérations lourdes dans `update()`
- [ ] Mémoire libérée (`release()` des blocs)
- [ ] Capteurs/contrôles dans `loop()`
- [ ] Compilation sans warnings

### Tests

- [ ] Pas de clipping audible
- [ ] Latence acceptable
- [ ] CPU usage < 80% (voir `AudioProcessorUsageMax()`)
- [ ] Réponse fluide aux contrôles
- [ ] Comportement stable sur durée

---

## 💡 Concepts Clés à Retenir

### Architecture

1. **I2C configure**, **I2S transmet** l'audio
2. **Audio callback** = fonction critique temps-réel
3. **Control rate** ≠ **Audio rate**

### DSP

1. Tout signal audio en **float [-1, 1]**
2. **Wavetables** > calculs trigonométriques
3. **Smoothing** évite les clicks

### Hardware

1. Teensy = **3.3V UNIQUEMENT**
2. Pins audio shield = **réservés**
3. **Pulldown/pullup** pour boutons

### Synthèse

1. **FM** = richesse harmonique contrôlable
2. **AM** = tremolo ou sidebands
3. **Karplus-Strong** = cordes pincées réalistes

### Effets

1. **Echo** = delay + feedback
2. **Distortion** = saturation contrôlée
3. **Filtres** = sculpter le spectre

---

**Bonne chance pour votre projet ! 🎵🚀**
