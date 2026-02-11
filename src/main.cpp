/*
 * TEENSY (Version RAM)
 * Enregistrement et lecture en RAM (sans carte SD)
 */

#include <Arduino.h>
#include <Audio.h>
#include <Wire.h>

// Configuration :

// Pins des boutons
#define BTN_RECORD  0    // Bouton pour enregistrer
#define BTN_PLAY    1    // Bouton pour jouer en inversé

// Paramètres audio
#define RECORD_TIME_SEC 4  // Durée max: 4 secondes
#define SAMPLE_RATE 44100
#define MAX_SAMPLES (SAMPLE_RATE * RECORD_TIME_SEC)

// Entrée microphone
AudioInputI2S i2s_input;      // Entrée I2S depuis le codec

// FILTRES D'ENTRÉE (chaîne de traitement)
AudioFilterBiquad filterHP;   // Filtre passe-haut (100 Hz)
AudioFilterBiquad filterLP;   // Filtre passe-bas (8 kHz)
AudioEffectMultiply noiseGate; // Noise gate (via envelope)
AudioSynthWaveformDc gateControl; // Contrôle du gate

AudioRecordQueue recordQueue;    // Queue pour capturer l'audio

// Sortie: Playback
AudioPlayQueue playQueue;      // Queue pour jouer depuis RAM

AudioOutputI2S i2s_output;     // Sortie I2S vers le codec

// Mixeur pour combiner playQueue et testTone
AudioMixer4 mixerLeft;
AudioMixer4 mixerRight;

// Contrôle du codec
AudioControlSGTL5000 audioShield;

// Connexions (patch cords)
// CHAÎNE D'ENTRÉE : Mic → Passe-Haut → Passe-Bas → Record Queue
AudioConnection patchCord1(i2s_input, 1, filterHP, 0);        // Mic RIGHT → Filtre Passe-Haut
AudioConnection patchCord2(filterHP, 0, filterLP, 0);         // Passe-Haut → Passe-Bas
AudioConnection patchCord3(filterLP, 0, recordQueue, 0);      // Passe-Bas → Record Queue

// CHAÎNE DE SORTIE : PlayQueue → Mixer → Out (compresseur appliqué dans playRecordingReversed)
AudioConnection patchCord4(playQueue, 0, mixerLeft, 0);       // PlayQueue → Mixer L
AudioConnection patchCord5(playQueue, 0, mixerRight, 0);      // PlayQueue → Mixer R
AudioConnection patchCord6(mixerLeft, 0, i2s_output, 0);      // Mixer L → Left Out
AudioConnection patchCord7(mixerRight, 0, i2s_output, 1);     // Mixer R → Right Out

// Déclaration des fonctions

void setupFilters();
void startRecording();
void handleRecording();
void stopRecording();
void playRecordingReversed();
int16_t softClip(int16_t sample);
int16_t compressor(int16_t sample);
float analyzeNoiseLevel();

// Variables globales

// Buffer audio en RAM 
int16_t audioBuffer[MAX_SAMPLES];
unsigned int recordedSamples = 0;

bool isRecording = false;
bool canRecord = true;  // Cooldown entre enregistrements
unsigned long recordStartTime = 0;
unsigned long lastRecordEndTime = 0;

// Paramètres des filtres
float noiseThreshold = 0.02;  // Seuil du noise gate (ajustable)

// Paramètres du compresseur
float compThreshold = 0.5;    // Seuil de compression (50% du max)
float compRatio = 4.0;        // Ratio 4:1
float compGain = 0.0;         // Gain actuel du compresseur (envelope follower)

void setup() {
  // Initialisation Serial
  Serial.begin(9600);
  delay(1000);

  // Configuration des pins
  pinMode(BTN_RECORD, INPUT);
  pinMode(BTN_PLAY, INPUT);

  // Allocation mémoire audio (augmentée pour filtres)
  AudioMemory(150); 

  // Configuration Audio Shield
  audioShield.enable();
  audioShield.volume(0.7); //volume casque 
  
  // Configuration de l'entrée audio
  audioShield.inputSelect(AUDIO_INPUT_MIC);
  audioShield.micGain(20);

  // Configuration des FILTRES
  setupFilters();  

  // Configuration des mixers
  mixerLeft.gain(0, 1.0);   // PlayQueue sur canal 0 à gain 1.0
  mixerLeft.gain(1, 0);     // TestTone désactivé
  mixerLeft.gain(2, 0);     // Canaux inutilisés
  mixerLeft.gain(3, 0);
  
  mixerRight.gain(0, 1.0);  // PlayQueue sur canal 0 à gain 1.0
  mixerRight.gain(1, 0);    // TestTone désactivé
  mixerRight.gain(2, 0);    // Canaux inutilisés
  mixerRight.gain(3, 0);

  Serial.println("\n========================================");
  Serial.println("    SYSTÈME D'ENREGISTREMENT FILTRÉ");
  Serial.println("========================================");
  Serial.println("FILTRES ACTIFS:");
  Serial.println("  ✓ Passe-Haut: 100 Hz (élimine rumble)");
  Serial.println("  ✓ Passe-Bas: 8 kHz (élimine sifflements)");
  Serial.println("  ✓ Compresseur: Ratio 4:1 (égalise volume)");
  Serial.println("----------------------------------------");
  Serial.println("COMMANDES:");
  Serial.println("  Bouton 0: RECORD (4s max)");
  Serial.println("  Bouton 1: PLAY INVERSÉ");
  Serial.println("========================================\n");
}

void loop() {
  // Vérifier cooldown de 2 secondes entre enregistrements
  if (!canRecord && (millis() - lastRecordEndTime > 2000)) {
    canRecord = true;
    Serial.println("[INFO] Prêt pour un nouvel enregistrement\n");
  }

  // Bouton RECORD
  if (digitalRead(BTN_RECORD) == HIGH && !isRecording && canRecord) {
    delay(50);  // Anti-rebond simple
    if (digitalRead(BTN_RECORD) == HIGH) {
      startRecording();
    }
  }

  // Pendant l'enregistrement
  if (isRecording) {
    handleRecording();
  }

  // Bouton PLAY (lecture inversée pour transmission au Teensy 2)
  if (digitalRead(BTN_PLAY) == HIGH && !isRecording) {
    delay(50);  // Anti-rebond (OK car pas pendant enregistrement)
    if (digitalRead(BTN_PLAY) == HIGH) {
      Serial.println("[INFO] Bouton 1 pressé - LECTURE INVERSÉE");
      playRecordingReversed();
      while (digitalRead(BTN_PLAY) == HIGH) delay(10); 
    }
  }
}

// Fonctions

void setupFilters() {
  Serial.println("[SETUP] Configuration des filtres...");
  
  // 1. FILTRE PASSE-HAUT (100 Hz) - Élimine les basses fréquences
  // Butterworth 2nd order, Q=0.707 pour réponse plate
  filterHP.setHighpass(0, 100, 0.707);
  Serial.println("  ✓ Passe-Haut: 100 Hz, Q=0.707");
  
  // 2. FILTRE PASSE-BAS (8 kHz) - Élimine les hautes fréquences
  filterLP.setLowpass(0, 8000, 0.707);
  Serial.println("  ✓ Passe-Bas: 8 kHz, Q=0.707");
  
  Serial.println("  ✓ Compresseur logiciel: Ratio 4:1, Seuil 50%");
  
  Serial.println("[SETUP] Filtres configurés!\n");
}

void startRecording() {
  Serial.println("\nDÉMARRAGE ENREGISTREMENT");
  
  // Réinitialiser le buffer
  recordedSamples = 0;

  isRecording = true;
  recordStartTime = millis();
  recordQueue.begin();
  
  Serial.println("PARLEZ MAINTENANT...");
}

void handleRecording() {
  // Vérifier timeout (4 secondes)
  unsigned long elapsed = millis() - recordStartTime;
  if (elapsed >= (RECORD_TIME_SEC * 1000)) {
    Serial.println("Temps max atteint (4s)");
    stopRecording();
    return;
  }

  // Vérifier si on relâche le bouton pour arrêter
  if (digitalRead(BTN_RECORD) == LOW) {
    Serial.println("Arrêt manuel");
    stopRecording();
    return;
  }

  // Sauvegarder l'audio disponible dans le buffer RAM
  if (recordQueue.available() >= 1) {
    
    // Récupérer un bloc (128 samples) - readBuffer() retourne int16_t*
    int16_t* blockData = (int16_t*)recordQueue.readBuffer();
    
    if (blockData) {
      // Copier dans le buffer principal (AUDIO_BLOCK_SAMPLES = 128)
      for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
        if (recordedSamples < MAX_SAMPLES) {
          audioBuffer[recordedSamples++] = blockData[i];
        } else {
          // Buffer plein
          recordQueue.freeBuffer();
          Serial.println("Buffer RAM plein (4s max)");
          stopRecording();
          return;
        }
      }
      
      recordQueue.freeBuffer();
    }
  } 
}

void stopRecording() {
  recordQueue.end();
  isRecording = false;
  canRecord = false;  // Bloquer pendant 2 secondes
  lastRecordEndTime = millis();
  
  // Calcul de la durée RÉELLE basée sur le temps écoulé
  float realDuration = (millis() - recordStartTime) / 1000.0;
  float sampleDuration = recordedSamples / (float)SAMPLE_RATE;
  
  Serial.print(">>> ENREGISTREMENT TERMINÉ: ");
  Serial.print(realDuration, 2);
  Serial.print(" secondes (temps réel) / ");
  Serial.print(sampleDuration, 2);
  Serial.println(" secondes (samples)");
  Serial.print("    Samples enregistrés: ");
  Serial.println(recordedSamples);
  Serial.print("    Samples attendus pour ");
  Serial.print(realDuration, 1);
  Serial.print("s: ");
  Serial.println((unsigned long)(realDuration * SAMPLE_RATE));

  Serial.println("    ⏸️ Attendre 2 secondes avant nouvel enregistrement...\n");
}

void playRecordingReversed() {
  if (recordedSamples == 0) {
    Serial.println("[ERREUR] Aucun enregistrement disponible!\n");
    Serial.println("Appuyez sur Bouton 0 pour enregistrer d'abord.\n");
    return;
  }

  Serial.println("\n>>> LECTURE INVERSÉE...\n");
  Serial.print("Durée enregistrement: ");
  Serial.print(recordedSamples / (float)SAMPLE_RATE, 2);
  Serial.println(" secondes");

  // Envoyer les données par blocs de 128 samples EN SENS INVERSE
  int sampleIndex = recordedSamples - 1;  // Commencer à la fin
  unsigned int blocksPlayed = 0;
  
  while (sampleIndex >= 0) {
    // Attendre qu'un buffer soit disponible
    int16_t* txBuffer = playQueue.getBuffer();
    if (txBuffer) {
      // Copier jusqu'à 128 samples EN SENS INVERSE avec compresseur
      for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
        if (sampleIndex >= 0) {
          // Appliquer compresseur puis soft clipping
          int16_t sample = audioBuffer[sampleIndex--];
          sample = compressor(sample);
          txBuffer[i] = softClip(sample);
        } else {
          txBuffer[i] = 0; 
        }
      }
      playQueue.playBuffer();  // Envoyer le buffer
      blocksPlayed++;
    } else {
      // Attendre qu'un buffer se libère
      delay(3);
    }
  }
  
  // Attendre que tous les buffers soient joués
  unsigned int totalBlocks = (recordedSamples + AUDIO_BLOCK_SAMPLES - 1) / AUDIO_BLOCK_SAMPLES;
  delay(totalBlocks * 3);
  
  Serial.println("LECTURE INVERSÉE TERMINÉE\n");
}

// Anti saturation avec VRAI soft clipping

int16_t softClip(int16_t sample) {
  // Convertir en float normalisé [-1.0, 1.0]
  float normalized = sample / 32767.0f;
  
  // Soft clipping avec fonction tanh (saturation douce)
  // tanh donne une courbe S qui évite la distortion brutale
  float clipped = tanh(normalized * 1.5) / 1.5; // Gain pré-saturation
  
  // Sécurité finale
  if (clipped > 1.0f) clipped = 1.0f;
  if (clipped < -1.0f) clipped = -1.0f;
  
  // Reconvertir en int16
  return (int16_t)(clipped * 32767.0f);
}

// Compresseur dynamique simple (envelope follower)
int16_t compressor(int16_t sample) {
  // Convertir en float absolu normalisé
  float input = abs(sample) / 32767.0f;
  
  // Envelope follower (attack/release)
  float attackCoeff = 0.01;   // Réaction rapide
  float releaseCoeff = 0.001; // Relâchement lent
  
  if (input > compGain) {
    compGain = compGain + attackCoeff * (input - compGain);
  } else {
    compGain = compGain + releaseCoeff * (input - compGain);
  }
  
  // Calculer le gain de réduction
  float gainReduction = 1.0;
  if (compGain > compThreshold) {
    // Au-dessus du seuil: appliquer la compression
    float over = compGain - compThreshold;
    gainReduction = compThreshold + (over / compRatio);
    gainReduction = gainReduction / compGain; // Normaliser
  }
  
  // Appliquer la réduction de gain
  float output = (sample / 32767.0f) * gainReduction;
  
  // Makeup gain (compenser la perte de volume)
  output *= 1.5;
  
  // Limiter
  if (output > 1.0f) output = 1.0f;
  if (output < -1.0f) output = -1.0f;
  
  return (int16_t)(output * 32767.0f);
}

// Analyse du niveau de bruit ambiant (pour ajuster le gate si besoin)
float analyzeNoiseLevel() {
  // Cette fonction pourrait être appelée avant l'enregistrement
  // pour calibrer automatiquement le noise gate
  // (Non implémentée dans cette version)
  return noiseThreshold;
}