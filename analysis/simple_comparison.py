#!/usr/bin/env python3
"""
Visualisation simple : Signal vocal normal vs inversé
Pour poster scientifique
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import signal

# Configuration
plt.rcParams['figure.figsize'] = (16, 8)
plt.rcParams['font.size'] = 12

# Paramètres audio
SAMPLE_RATE = 44100
DURATION = 2.0  # 2 secondes
NUM_SAMPLES = int(SAMPLE_RATE * DURATION)

def generate_speech_signal():
    """Génère un signal simulant de la parole"""
    t = np.linspace(0, DURATION, NUM_SAMPLES, endpoint=False)
    
    # Signal de parole réaliste
    f0 = 150  # Fréquence fondamentale (voix)
    speech = np.zeros(NUM_SAMPLES)
    
    # Harmoniques de la voix
    harmonics = [1, 2, 3, 4, 5, 7, 9]
    amplitudes = [1.0, 0.5, 0.3, 0.2, 0.15, 0.1, 0.05]
    
    for harm, amp in zip(harmonics, amplitudes):
        speech += amp * np.sin(2 * np.pi * f0 * harm * t)
    
    # Modulation d'amplitude (simulation de mots/syllabes)
    # Créer 3 "mots" avec des silences
    envelope = np.zeros(NUM_SAMPLES)
    
    # Mot 1 (0-0.5s)
    word1_samples = int(0.5 * SAMPLE_RATE)
    envelope[:word1_samples] = np.abs(np.sin(2 * np.pi * 5 * t[:word1_samples]))
    
    # Silence (0.5-0.7s)
    
    # Mot 2 (0.7-1.2s)
    word2_start = int(0.7 * SAMPLE_RATE)
    word2_end = int(1.2 * SAMPLE_RATE)
    word2_t = t[word2_start:word2_end] - t[word2_start]
    envelope[word2_start:word2_end] = np.abs(np.sin(2 * np.pi * 6 * word2_t))
    
    # Silence (1.2-1.4s)
    
    # Mot 3 (1.4-2.0s)
    word3_start = int(1.4 * SAMPLE_RATE)
    word3_t = t[word3_start:] - t[word3_start]
    envelope[word3_start:] = np.abs(np.sin(2 * np.pi * 4 * word3_t))
    
    speech *= envelope
    
    # Ajouter un peu de bruit réaliste
    noise = 0.02 * np.random.randn(NUM_SAMPLES)
    speech += noise
    
    # Normaliser
    speech = speech / np.max(np.abs(speech)) * 0.8
    
    return t, speech

def create_comparison_plot(t, normal_signal):
    """Crée le graphique de comparaison normal vs inversé"""
    
    # Signal inversé (reversed)
    reversed_signal = normal_signal[::-1]
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(16, 8))
    
    # Graphique 1: Signal NORMAL
    ax1.plot(t, normal_signal, 'b-', linewidth=1.2)
    ax1.set_title('SIGNAL VOCAL NORMAL (Enregistrement)', 
                  fontsize=16, fontweight='bold', color='blue')
    ax1.set_xlabel('Temps (secondes)', fontsize=12)
    ax1.set_ylabel('Amplitude', fontsize=12)
    ax1.grid(True, alpha=0.3)
    ax1.set_ylim([-1, 1])
    ax1.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    
    # Annotations pour montrer les "mots"
    ax1.axvspan(0, 0.5, alpha=0.1, color='green', label='Mot 1')
    ax1.axvspan(0.7, 1.2, alpha=0.1, color='green', label='Mot 2')
    ax1.axvspan(1.4, 2.0, alpha=0.1, color='green', label='Mot 3')
    
    # Graphique 2: Signal INVERSÉ
    ax2.plot(t, reversed_signal, 'r-', linewidth=1.2)
    ax2.set_title('SIGNAL VOCAL INVERSÉ (Lecture inversée pour transmission)', 
                  fontsize=16, fontweight='bold', color='red')
    ax2.set_xlabel('Temps (secondes)', fontsize=12)
    ax2.set_ylabel('Amplitude', fontsize=12)
    ax2.grid(True, alpha=0.3)
    ax2.set_ylim([-1, 1])
    ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    
    # Annotations inversées
    ax2.axvspan(0, 0.6, alpha=0.1, color='orange', label='Mot 3 (inversé)')
    ax2.axvspan(0.8, 1.3, alpha=0.1, color='orange', label='Mot 2 (inversé)')
    ax2.axvspan(1.5, 2.0, alpha=0.1, color='orange', label='Mot 1 (inversé)')
    
    # Titre principal
    fig.suptitle('COMPARAISON : Signal Normal vs Signal Inversé\n' +
                 'Système de Message Secret - Teensy Audio', 
                 fontsize=18, fontweight='bold', y=0.98)
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.savefig('signal_comparison.png', dpi=300, bbox_inches='tight')
    print("✅ Graphique sauvegardé: signal_comparison.png")

def create_zoomed_comparison(t, normal_signal):
    """Crée un zoom sur 0.3 secondes pour voir les détails"""
    
    # Signal inversé
    reversed_signal = normal_signal[::-1]
    
    # Zoom sur 0.3 secondes (au milieu du premier mot)
    zoom_start = int(0.2 * SAMPLE_RATE)
    zoom_end = int(0.5 * SAMPLE_RATE)
    
    t_zoom = t[zoom_start:zoom_end]
    normal_zoom = normal_signal[zoom_start:zoom_end]
    
    # Pour le signal inversé, prendre la portion correspondante
    reversed_zoom = reversed_signal[zoom_start:zoom_end]
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(16, 8))
    
    # Graphique 1: Signal NORMAL (zoom)
    ax1.plot(t_zoom, normal_zoom, 'b-', linewidth=1.5)
    ax1.set_title('SIGNAL NORMAL - Détail (0.3 secondes)', 
                  fontsize=16, fontweight='bold', color='blue')
    ax1.set_xlabel('Temps (secondes)', fontsize=12)
    ax1.set_ylabel('Amplitude', fontsize=12)
    ax1.grid(True, alpha=0.3)
    ax1.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    
    # Graphique 2: Signal INVERSÉ (zoom)
    ax2.plot(t_zoom, reversed_zoom, 'r-', linewidth=1.5)
    ax2.set_title('SIGNAL INVERSÉ - Détail (0.3 secondes)', 
                  fontsize=16, fontweight='bold', color='red')
    ax2.set_xlabel('Temps (secondes)', fontsize=12)
    ax2.set_ylabel('Amplitude', fontsize=12)
    ax2.grid(True, alpha=0.3)
    ax2.axhline(y=0, color='k', linestyle='--', alpha=0.3)
    
    fig.suptitle('COMPARAISON DÉTAILLÉE : Forme d\'Onde\n' +
                 'Observez l\'inversion temporelle du signal', 
                 fontsize=18, fontweight='bold', y=0.98)
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.savefig('signal_comparison_zoom.png', dpi=300, bbox_inches='tight')
    print("✅ Graphique zoom sauvegardé: signal_comparison_zoom.png")

def main():
    print("\n" + "="*60)
    print("  🎤 GÉNÉRATION DES GRAPHIQUES DE COMPARAISON")
    print("="*60 + "\n")
    
    # 1. Générer le signal de parole
    print("🎤 Génération du signal vocal simulé...")
    t, speech_signal = generate_speech_signal()
    
    # 2. Créer le graphique de comparaison
    print("📊 Création du graphique de comparaison...\n")
    create_comparison_plot(t, speech_signal)
    
    # 3. Créer un zoom pour voir les détails
    print("🔍 Création du graphique zoom...\n")
    create_zoomed_comparison(t, speech_signal)
    
    # Statistiques
    print("\n" + "="*60)
    print("  📈 STATISTIQUES")
    print("="*60)
    print(f"Durée du signal:       {DURATION} secondes")
    print(f"Taux d'échantillonnage: {SAMPLE_RATE} Hz")
    print(f"Nombre d'échantillons: {NUM_SAMPLES}")
    print(f"Amplitude max:         {np.max(np.abs(speech_signal)):.4f}")
    print("="*60 + "\n")
    
    print("✨ TERMINÉ! Graphiques prêts pour votre poster.\n")
    print("📁 Fichiers générés:")
    print("   - signal_comparison.png (comparaison complète)")
    print("   - signal_comparison_zoom.png (détail agrandi)")
    print("\n")

if __name__ == "__main__":
    main()
