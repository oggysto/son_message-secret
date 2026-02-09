/*
 * TEENSY 1 - ÉTAPE 1 (Version RAM)
 * Enregistrement et lecture en RAM (sans SD Card ni LED)
 * 
 * Matériel:
 * - Teensy 4.0 + Audio Shield
 * - Micro (Mic In du shield)
 * - Casque (Headphone Out)
 * - 2 boutons avec résistances pulldown 10kΩ
 * 
 * Limitations:
 * - Max 2 secondes d'audio (stockage RAM)
 * - Pas de LED visuelle (utiliser Serial Monitor)
 */

#include <Arduino.h>
#include <Audio.h>
#include <Wire.h>

// ============================================================
// CONFIGURATION
// ============================================================

// Pins des boutons
#define BTN_RECORD  0    // Bouton pour enregistrer
#define BTN_PLAY    1    // Bouton pour jouer en inversé

// Paramètres audio
#define RECORD_TIME_SEC  4                    // Durée max: 4 secondes
#define SAMPLE_RATE      44100                // Hz
#define MAX_SAMPLES      (SAMPLE_RATE * RECORD_TIME_SEC)  // 176400 samples

// ============================================================
// OBJETS AUDIO
// ============================================================

// Entrée: Microphone
AudioInputI2S            i2s_input;      // Entrée I2S depuis le codec
AudioRecordQueue         recordQueue;    // Queue pour capturer l'audio

// Sortie: Playback
AudioPlayQueue           playQueue;      // Queue pour jouer depuis RAM
AudioOutputI2S           i2s_output;     // Sortie I2S vers le codec

// Test de tonalité
AudioSynthWaveformSine   testTone;       // Générateur de tonalité pour test

// Mixeur pour combiner playQueue et testTone
AudioMixer4              mixerLeft;
AudioMixer4              mixerRight;

// Contrôle du codec SGTL5000
AudioControlSGTL5000     audioShield;

// Connexions (patch cords)
AudioConnection patchCord1(i2s_input, 1, recordQueue, 0);     // Mic RIGHT → Record Queue
AudioConnection patchCord2(playQueue, 0, mixerLeft, 0);       // PlayQueue → Mixer L
AudioConnection patchCord3(playQueue, 0, mixerRight, 0);      // PlayQueue → Mixer R
AudioConnection patchCord4(testTone, 0, mixerLeft, 1);        // TestTone → Mixer L
AudioConnection patchCord5(testTone, 0, mixerRight, 1);       // TestTone → Mixer R
AudioConnection patchCord6(mixerLeft, 0, i2s_output, 0);      // Mixer L → Left Out
AudioConnection patchCord7(mixerRight, 0, i2s_output, 1);     // Mixer R → Right Out

// ============================================================
// DÉCLARATIONS DE FONCTIONS
// ============================================================

void startRecording();
void handleRecording();
void stopRecording();
void playRecordingReversed();
int16_t softClip(int16_t sample);

// ============================================================
// VARIABLES GLOBALES
// ============================================================

// Buffer audio en RAM (stockage 16-bit signed)
int16_t audioBuffer[MAX_SAMPLES];
unsigned int recordedSamples = 0;

bool isRecording = false;
bool canRecord = true;  // Cooldown entre enregistrements
unsigned long recordStartTime = 0;
unsigned long lastRecordEndTime = 0;

// Diagnostic de perte de blocs
unsigned int blocksReceived = 0;
unsigned int blocksSkipped = 0;

// ============================================================
// SETUP
// ============================================================

void setup() {
  // Initialisation Serial
  Serial.begin(9600);
  delay(1000);
  
  Serial.println("================================");
  Serial.println("TEENSY 1 - ÉTAPE 1 (RAM)");
  Serial.println("Record & Play - Sans SD Card");
  Serial.println("================================\n");

  // Configuration des pins
  pinMode(BTN_RECORD, INPUT);
  pinMode(BTN_PLAY, INPUT);

  // Allocation mémoire audio (augmentée pour éviter perte de samples)
  AudioMemory(120);  // Doublé de 60 à 120
  Serial.println("[OK] Audio Memory allouée (120 blocs)");

  // Configuration Audio Shield
  audioShield.enable();
  audioShield.volume(0.7);  // Volume casque augmenté pour meilleure audibilité
  
  // *** MICROPHONE ACTIVÉ ***
  audioShield.inputSelect(AUDIO_INPUT_MIC);
  audioShield.micGain(20);  // Gain réduit de 50 à 20 dB (recommandation cours)
  Serial.println("[OK] Audio Shield configuré (MIC - Gain 20)");

  // Configuration des mixers
  mixerLeft.gain(0, 1.0);   // PlayQueue sur canal 0 à gain 1.0
  mixerLeft.gain(1, 0);     // TestTone désactivé
  mixerLeft.gain(2, 0);     // Canaux inutilisés
  mixerLeft.gain(3, 0);
  
  mixerRight.gain(0, 1.0);  // PlayQueue sur canal 0 à gain 1.0
  mixerRight.gain(1, 0);    // TestTone désactivé
  mixerRight.gain(2, 0);    // Canaux inutilisés
  mixerRight.gain(3, 0);
  Serial.println("[OK] Mixers configurés");

  Serial.println("\n--- PRÊT ---");
  Serial.println("Bouton 0: RECORD (4s max)");
  Serial.println("Bouton 1: PLAY INVERSÉ (pour Teensy 2)");
  Serial.println("Suivez les messages dans le Serial Monitor");
  Serial.println("----------------\n");
}

// ============================================================
// LOOP
// ============================================================

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
      while (digitalRead(BTN_PLAY) == HIGH) delay(10);  // Attendre relâchement
    }
  }

  // PAS DE DELAY ICI ! Les blocs audio arrivent toutes les 2.9ms
  // Un delay(10) fait perdre 70% des samples
}

// ============================================================
// FONCTIONS
// ============================================================

void startRecording() {
  Serial.println("\n>>> DÉMARRAGE ENREGISTREMENT");
  
  // Réinitialiser le buffer
  recordedSamples = 0;
  blocksReceived = 0;
  blocksSkipped = 0;

  isRecording = true;
  recordStartTime = millis();
  recordQueue.begin();
  
  Serial.println("    🎤 PARLEZ MAINTENANT...");
}

void handleRecording() {
  // Vérifier timeout (2 secondes)
  unsigned long elapsed = millis() - recordStartTime;
  if (elapsed >= (RECORD_TIME_SEC * 1000)) {
    Serial.println("    ⏱️ Temps max atteint (2s)");
    stopRecording();
    return;
  }

  // Vérifier si on relâche le bouton pour arrêter
  if (digitalRead(BTN_RECORD) == LOW) {
    Serial.println("    ⏹️ Arrêt manuel");
    stopRecording();
    return;
  }

  // Sauvegarder l'audio disponible dans le buffer RAM
  if (recordQueue.available() >= 1) {
    blocksReceived++;
    
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
          Serial.println("    ⚠️ Buffer RAM plein (2s max)");
          stopRecording();
          return;
        }
      }
      
      recordQueue.freeBuffer();
    }
  } else {
    blocksSkipped++;  // Compter quand la queue est vide
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
  
  // Diagnostic de perte
  unsigned int expectedBlocks = (unsigned long)(realDuration * SAMPLE_RATE) / AUDIO_BLOCK_SAMPLES;
  Serial.print("    Blocs audio reçus: ");
  Serial.print(blocksReceived);
  Serial.print(" / ");
  Serial.print(expectedBlocks);
  Serial.print(" attendus (");
  Serial.print((blocksReceived * 100) / expectedBlocks);
  Serial.println("%)");
  Serial.print("    Iterations avec queue vide: ");
  Serial.println(blocksSkipped);
  
  Serial.print("    Mémoire utilisée: ");
  Serial.print((recordedSamples * 2) / 1024.0, 1);
  Serial.println(" KB");
  
  // DEBUG: Afficher quelques valeurs pour vérifier l'enregistrement
  if (recordedSamples > 100) {
    Serial.print("    DEBUG Samples [0-9]: ");
    for (int i = 0; i < 10; i++) {
      Serial.print(audioBuffer[i]);
      Serial.print(" ");
    }
    Serial.println();
    
    // Calculer min/max pour voir la variation
    int16_t minVal = audioBuffer[0];
    int16_t maxVal = audioBuffer[0];
    for (unsigned int i = 0; i < recordedSamples; i++) {
      if (audioBuffer[i] < minVal) minVal = audioBuffer[i];
      if (audioBuffer[i] > maxVal) maxVal = audioBuffer[i];
    }
    Serial.print("    DEBUG Min/Max: ");
    Serial.print(minVal);
    Serial.print(" / ");
    Serial.println(maxVal);
    Serial.print("    DEBUG Variation: ");
    Serial.println(maxVal - minVal);
  }
  
  Serial.println("    ⏸️ Attendre 2 secondes avant nouvel enregistrement...\n");
}

void playRecordingReversed() {
  if (recordedSamples == 0) {
    Serial.println("[ERREUR] Aucun enregistrement disponible!");
    Serial.println("         Appuyez sur Bouton 0 pour enregistrer d'abord.\n");
    return;
  }

  Serial.println("\n>>> LECTURE INVERSÉE...");
  Serial.print("    Durée enregistrement: ");
  Serial.print(recordedSamples / (float)SAMPLE_RATE, 2);
  Serial.println(" secondes");

  // Envoyer les données par blocs de 128 samples EN SENS INVERSE
  int sampleIndex = recordedSamples - 1;  // Commencer à la fin
  unsigned int blocksPlayed = 0;
  
  while (sampleIndex >= 0) {
    // Attendre qu'un buffer soit disponible
    int16_t* txBuffer = playQueue.getBuffer();
    if (txBuffer) {
      // Copier jusqu'à 128 samples avec soft clipping EN SENS INVERSE
      for (int i = 0; i < AUDIO_BLOCK_SAMPLES; i++) {
        if (sampleIndex >= 0) {
          txBuffer[i] = softClip(audioBuffer[sampleIndex--]);  // Index décrémente
        } else {
          txBuffer[i] = 0;  // Padding avec silence
        }
      }
      playQueue.playBuffer();  // Envoyer le buffer
      blocksPlayed++;
    } else {
      // Attendre qu'un buffer se libère (environ 2.9ms par bloc à 44.1kHz)
      delay(3);
    }
  }
  
  // Attendre que tous les buffers soient joués
  unsigned int totalBlocks = (recordedSamples + AUDIO_BLOCK_SAMPLES - 1) / AUDIO_BLOCK_SAMPLES;
  delay(totalBlocks * 3);
  
  Serial.println(">>> LECTURE INVERSÉE TERMINÉE\n");
}

// ============================================================
// SOFT CLIPPING (anti-saturation)
// ============================================================

int16_t softClip(int16_t sample) {
  // Convertir en float normalisé [-1.0, 1.0]
  float normalized = sample / 32767.0f;
  
  // Soft clipping: tanh donne une saturation douce
  // Pour un clipping plus dur, on peut utiliser max/min
  if (normalized > 1.0f) normalized = 1.0f;
  if (normalized < -1.0f) normalized = -1.0f;
  
  // Reconvertir en int16
  return (int16_t)(normalized * 32767.0f);
}