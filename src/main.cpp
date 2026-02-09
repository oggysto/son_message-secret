/*
 * TEENSY 1 - ÉTAPE 1
 * Enregistrement et lecture simple
 * 
 * Matériel:
 * - Teensy 4.0 + Audio Shield
 * - Carte SD insérée
 * - Micro (Mic In du shield)
 * - Casque (Headphone Out)
 * - 2 boutons avec résistances pulldown 10kΩ
 */

#include <Arduino.h>
#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>

// ============================================================
// CONFIGURATION
// ============================================================

// Pins des boutons
#define BTN_RECORD  0    // Bouton pour enregistrer
#define BTN_PLAY    1    // Bouton pour rejouer
#define LED_PIN     13   // LED intégrée du Teensy

// Configuration SD Card (pins du Audio Shield)
#define SDCARD_CS_PIN    10
#define SDCARD_MOSI_PIN  11
#define SDCARD_SCK_PIN   13

// Paramètres audio
#define RECORD_TIME_SEC  10              // Durée max: 10 secondes
const char* FILENAME = "voice.raw";      // Nom du fichier sur SD

// ============================================================
// OBJETS AUDIO
// ============================================================

// Entrée: Microphone
AudioInputI2S            i2s_input;      // Entrée I2S depuis le codec
AudioRecordQueue         queue;          // Queue pour capturer l'audio

// Sortie: Playback
AudioPlaySdRaw           playback;       // Lecteur de fichier SD
AudioOutputI2S           i2s_output;     // Sortie I2S vers le codec

// Contrôle du codec SGTL5000
AudioControlSGTL5000     audioShield;

// Connexions (patch cords)
AudioConnection patchCord1(i2s_input, 0, queue, 0);        // Mic → Queue
AudioConnection patchCord2(playback, 0, i2s_output, 0);    // Playback → Left
AudioConnection patchCord3(playback, 0, i2s_output, 1);    // Playback → Right

// ============================================================
// DÉCLARATIONS DE FONCTIONS
// ============================================================

void startRecording();
void handleRecording();
void stopRecording();
void playRecording();

// ============================================================
// VARIABLES GLOBALES
// ============================================================

bool isRecording = false;
unsigned long recordStartTime = 0;

// ============================================================
// SETUP
// ============================================================

void setup() {
  // Initialisation Serial
  Serial.begin(9600);
  delay(1000);
  
  Serial.println("================================");
  Serial.println("TEENSY 1 - ÉTAPE 1");
  Serial.println("Record & Play Simple");
  Serial.println("================================\n");

  // Configuration des pins
  pinMode(BTN_RECORD, INPUT);
  pinMode(BTN_PLAY, INPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Allocation mémoire audio
  AudioMemory(60);
  Serial.println("[OK] Audio Memory allouée");

  // Initialisation SD Card
  SPI.setMOSI(SDCARD_MOSI_PIN);
  SPI.setSCK(SDCARD_SCK_PIN);
  
  if (!SD.begin(SDCARD_CS_PIN)) {
    Serial.println("[ERREUR] Carte SD non détectée!");
    Serial.println("→ Vérifiez que la carte SD est insérée");
    
    // Blink LED d'erreur
    while (1) {
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(200);
    }
  }
  Serial.println("[OK] Carte SD initialisée");

  // Configuration Audio Shield
  audioShield.enable();
  audioShield.volume(0.6);                    // Volume casque
  audioShield.inputSelect(AUDIO_INPUT_MIC);   // Sélection micro
  audioShield.micGain(30);                    // Gain micro (ajustable 0-63)
  Serial.println("[OK] Audio Shield configuré");

  Serial.println("\n--- PRÊT ---");
  Serial.println("Bouton 0: RECORD (10s max)");
  Serial.println("Bouton 1: PLAY");
  Serial.println("----------------\n");

  // Signal prêt (3 blinks)
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
}

// ============================================================
// LOOP
// ============================================================

void loop() {
  // Bouton RECORD
  if (digitalRead(BTN_RECORD) == HIGH && !isRecording) {
    delay(50);  // Anti-rebond simple
    if (digitalRead(BTN_RECORD) == HIGH) {
      startRecording();
    }
  }

  // Pendant l'enregistrement
  if (isRecording) {
    handleRecording();
  }

  // Bouton PLAY
  if (digitalRead(BTN_PLAY) == HIGH && !isRecording) {
    delay(50);  // Anti-rebond
    if (digitalRead(BTN_PLAY) == HIGH) {
      playRecording();
      while (digitalRead(BTN_PLAY) == HIGH) delay(10);  // Attendre relâchement
    }
  }

  delay(10);
}

// ============================================================
// FONCTIONS
// ============================================================

void startRecording() {
  Serial.println("\n>>> DÉMARRAGE ENREGISTREMENT");
  
  // Supprimer ancien fichier
  if (SD.exists(FILENAME)) {
    SD.remove(FILENAME);
    Serial.println("    Ancien fichier supprimé");
  }

  isRecording = true;
  recordStartTime = millis();
  queue.begin();
  
  digitalWrite(LED_PIN, HIGH);  // LED ON
  Serial.println("    🎤 PARLEZ MAINTENANT...");
}

void handleRecording() {
  // Vérifier timeout (10 secondes)
  if (millis() - recordStartTime > (RECORD_TIME_SEC * 1000)) {
    stopRecording();
    Serial.println("    ⏱️ Temps max atteint");
    return;
  }

  // Vérifier si on relâche le bouton pour arrêter
  if (digitalRead(BTN_RECORD) == LOW) {
    stopRecording();
    Serial.println("    ⏹️ Arrêt manuel");
    return;
  }

  // Sauvegarder l'audio disponible
  if (queue.available() >= 2) {
    // Ouvrir fichier en mode append
    File audioFile = SD.open(FILENAME, FILE_WRITE);
    if (audioFile) {
      // Lire 2 blocs (256 samples = 512 bytes)
      byte buffer[512];
      memcpy(buffer, queue.readBuffer(), 256);
      queue.freeBuffer();
      memcpy(buffer + 256, queue.readBuffer(), 256);
      queue.freeBuffer();
      
      // Écrire sur SD
      audioFile.write(buffer, 512);
      audioFile.close();
    }
  }
}

void stopRecording() {
  queue.end();
  isRecording = false;
  digitalWrite(LED_PIN, LOW);
  
  float duration = (millis() - recordStartTime) / 1000.0;
  Serial.print(">>> ENREGISTREMENT TERMINÉ: ");
  Serial.print(duration, 1);
  Serial.println(" secondes\n");
}

void playRecording() {
  if (!SD.exists(FILENAME)) {
    Serial.println("[ERREUR] Aucun enregistrement disponible!");
    return;
  }

  Serial.println("\n>>> LECTURE EN COURS...");
  digitalWrite(LED_PIN, HIGH);

  playback.play(FILENAME);
  delay(50);  // Laisser le temps de démarrer

  // Attendre la fin de lecture
  while (playback.isPlaying()) {
    delay(10);
  }

  digitalWrite(LED_PIN, LOW);
  Serial.println(">>> LECTURE TERMINÉE\n");
}