#!/usr/bin/env python3
"""
Visualisation des effets des filtres audio pour poster scientifique
Simule un signal de parole avec bruit et montre l'effet des filtres
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from scipy.fft import fft, fftfreq
import matplotlib.gridspec as gridspec

# Configuration
plt.rcParams['figure.figsize'] = (16, 10)
plt.rcParams['font.size'] = 10
plt.rcParams['axes.grid'] = True
plt.rcParams['grid.alpha'] = 0.3

# Paramètres audio
SAMPLE_RATE = 44100
DURATION = 1.0  # secondes
NUM_SAMPLES = int(SAMPLE_RATE * DURATION)

def generate_speech_signal():
    """Génère un signal simulant de la parole avec bruit"""
    t = np.linspace(0, DURATION, NUM_SAMPLES, endpoint=False)
    
    # Signal de parole simulé (combinaison de fréquences vocales)
    # Fréquences fondamentales typiques de la voix (100-300 Hz)
    f0 = 150  # Fréquence fondamentale
    speech = np.zeros(NUM_SAMPLES)
    
    # Harmoniques de la voix (formants)
    harmonics = [1, 2, 3, 4, 5, 7, 9]
    amplitudes = [1.0, 0.5, 0.3, 0.2, 0.15, 0.1, 0.05]
    
    for harm, amp in zip(harmonics, amplitudes):
        speech += amp * np.sin(2 * np.pi * f0 * harm * t)
    
    # Modulation d'amplitude (simulation de phonèmes)
    envelope = np.abs(np.sin(2 * np.pi * 3 * t)) * 0.5 + 0.5
    speech *= envelope
    
    # Ajouter du bruit basse fréquence (rumble 20-80 Hz)
    rumble = 0.3 * np.sin(2 * np.pi * 30 * t) + 0.2 * np.sin(2 * np.pi * 50 * t)
    
    # Ajouter du bruit haute fréquence (sifflement > 8 kHz)
    hiss = 0.15 * np.random.randn(NUM_SAMPLES)
    hiss_filtered = signal.lfilter([1], [1, -0.95], hiss)  # Bruit rose
    
    # Ajouter bruit blanc général
    white_noise = 0.05 * np.random.randn(NUM_SAMPLES)
    
    # Signal brut = parole + tous les bruits
    raw_signal = speech + rumble + hiss_filtered + white_noise
    
    # Normaliser
    raw_signal = raw_signal / np.max(np.abs(raw_signal)) * 0.8
    
    return t, raw_signal, speech

def design_filters():
    """Conçoit les filtres utilisés dans le Teensy"""
    # Filtre passe-haut (100 Hz, Butterworth 2nd order, Q=0.707)
    sos_hp = signal.butter(2, 100, 'hp', fs=SAMPLE_RATE, output='sos')
    
    # Filtre passe-bas (8 kHz, Butterworth 2nd order, Q=0.707)
    sos_lp = signal.butter(2, 8000, 'lp', fs=SAMPLE_RATE, output='sos')
    
    return sos_hp, sos_lp

def apply_filters(raw_signal, sos_hp, sos_lp):
    """Applique les filtres comme sur le Teensy"""
    # Appliquer passe-haut
    filtered = signal.sosfilt(sos_hp, raw_signal)
    
    # Appliquer passe-bas
    filtered = signal.sosfilt(sos_lp, filtered)
    
    return filtered

def simple_compressor(signal_in, threshold=0.5, ratio=4.0, makeup_gain=1.5):
    """Compresseur dynamique simple"""
    output = np.zeros_like(signal_in)
    gain = 0.0
    attack = 0.01
    release = 0.001
    
    for i, sample in enumerate(signal_in):
        # Envelope follower
        input_level = abs(sample)
        if input_level > gain:
            gain = gain + attack * (input_level - gain)
        else:
            gain = gain + release * (input_level - gain)
        
        # Calcul de réduction de gain
        if gain > threshold:
            over = gain - threshold
            gain_reduction = threshold + (over / ratio)
            gain_reduction = gain_reduction / gain if gain > 0 else 1.0
        else:
            gain_reduction = 1.0
        
        # Appliquer
        output[i] = sample * gain_reduction * makeup_gain
    
    # Limiter
    output = np.clip(output, -1.0, 1.0)
    
    return output

def compute_fft(signal_data, sample_rate):
    """Calcule la FFT et retourne fréquences et magnitudes"""
    N = len(signal_data)
    yf = fft(signal_data)
    xf = fftfreq(N, 1/sample_rate)
    
    # Ne garder que les fréquences positives
    positive_freq_idx = xf > 0
    xf = xf[positive_freq_idx]
    yf = yf[positive_freq_idx]
    
    # Magnitude en dB
    magnitude_db = 20 * np.log10(np.abs(yf) + 1e-10)
    
    return xf, magnitude_db

def plot_filter_responses(sos_hp, sos_lp):
    """Trace la réponse en fréquence des filtres"""
    w_hp, h_hp = signal.sosfreqz(sos_hp, worN=2048, fs=SAMPLE_RATE)
    w_lp, h_lp = signal.sosfreqz(sos_lp, worN=2048, fs=SAMPLE_RATE)
    
    # Réponse combinée
    h_combined = h_hp * h_lp
    
    plt.figure(figsize=(12, 4))
    plt.semilogx(w_hp, 20 * np.log10(abs(h_hp)), 'b-', label='Passe-Haut (100 Hz)', linewidth=2)
    plt.semilogx(w_lp, 20 * np.log10(abs(h_lp)), 'r-', label='Passe-Bas (8 kHz)', linewidth=2)
    plt.semilogx(w_hp, 20 * np.log10(abs(h_combined)), 'g-', label='Combiné', linewidth=2.5)
    
    plt.axvline(100, color='b', linestyle='--', alpha=0.5, label='Coupure HP: 100 Hz')
    plt.axvline(8000, color='r', linestyle='--', alpha=0.5, label='Coupure LP: 8 kHz')
    
    plt.xlabel('Fréquence (Hz)')
    plt.ylabel('Gain (dB)')
    plt.title('Réponse en Fréquence des Filtres Biquad', fontsize=14, fontweight='bold')
    plt.grid(True, which='both', alpha=0.3)
    plt.legend(loc='best')
    plt.xlim([20, 20000])
    plt.ylim([-60, 5])
    plt.tight_layout()
    plt.savefig('filter_response.png', dpi=300, bbox_inches='tight')
    print("✅ Graphique sauvegardé: filter_response.png")

def create_comprehensive_plot(t, raw_signal, filtered_signal, compressed_signal):
    """Crée le graphique complet pour le poster scientifique"""
    
    fig = plt.figure(figsize=(18, 12))
    gs = gridspec.GridSpec(3, 2, figure=fig, hspace=0.3, wspace=0.3)
    
    # 1. Signal temporel - RAW
    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(t[:5000], raw_signal[:5000], 'b-', linewidth=0.5)
    ax1.set_title('1. Signal BRUT (avec bruit)', fontsize=12, fontweight='bold')
    ax1.set_xlabel('Temps (s)')
    ax1.set_ylabel('Amplitude')
    ax1.grid(True, alpha=0.3)
    ax1.set_ylim([-1, 1])
    
    # 2. Signal temporel - FILTRÉ
    ax2 = fig.add_subplot(gs[0, 1])
    ax2.plot(t[:5000], filtered_signal[:5000], 'g-', linewidth=0.5)
    ax2.set_title('2. Après Filtres (HP 100Hz + LP 8kHz)', fontsize=12, fontweight='bold', color='green')
    ax2.set_xlabel('Temps (s)')
    ax2.set_ylabel('Amplitude')
    ax2.grid(True, alpha=0.3)
    ax2.set_ylim([-1, 1])
    
    # 3. FFT - RAW
    ax3 = fig.add_subplot(gs[1, 0])
    freqs_raw, mag_raw = compute_fft(raw_signal, SAMPLE_RATE)
    ax3.semilogx(freqs_raw, mag_raw, 'b-', linewidth=1)
    ax3.set_title('3. Spectre Fréquentiel BRUT', fontsize=12, fontweight='bold')
    ax3.set_xlabel('Fréquence (Hz)')
    ax3.set_ylabel('Magnitude (dB)')
    ax3.grid(True, which='both', alpha=0.3)
    ax3.set_xlim([20, 20000])
    ax3.axvline(100, color='r', linestyle='--', alpha=0.5, label='Coupure HP')
    ax3.axvline(8000, color='r', linestyle='--', alpha=0.5, label='Coupure LP')
    ax3.legend()
    
    # 4. FFT - FILTRÉ
    ax4 = fig.add_subplot(gs[1, 1])
    freqs_filt, mag_filt = compute_fft(filtered_signal, SAMPLE_RATE)
    ax4.semilogx(freqs_filt, mag_filt, 'g-', linewidth=1)
    ax4.set_title('4. Spectre Fréquentiel FILTRÉ', fontsize=12, fontweight='bold', color='green')
    ax4.set_xlabel('Fréquence (Hz)')
    ax4.set_ylabel('Magnitude (dB)')
    ax4.grid(True, which='both', alpha=0.3)
    ax4.set_xlim([20, 20000])
    ax4.axvline(100, color='r', linestyle='--', alpha=0.5)
    ax4.axvline(8000, color='r', linestyle='--', alpha=0.5)
    
    # 5. Comparaison temporelle ZOOM
    ax5 = fig.add_subplot(gs[2, 0])
    zoom_start = 1000
    zoom_end = 2000
    ax5.plot(t[zoom_start:zoom_end], raw_signal[zoom_start:zoom_end], 'b-', 
             linewidth=1, alpha=0.7, label='Signal brut')
    ax5.plot(t[zoom_start:zoom_end], filtered_signal[zoom_start:zoom_end], 'g-', 
             linewidth=1.5, label='Signal filtré')
    ax5.set_title('5. Comparaison Détaillée (zoom)', fontsize=12, fontweight='bold')
    ax5.set_xlabel('Temps (s)')
    ax5.set_ylabel('Amplitude')
    ax5.grid(True, alpha=0.3)
    ax5.legend()
    
    # 6. Après compresseur
    ax6 = fig.add_subplot(gs[2, 1])
    ax6.plot(t[:5000], compressed_signal[:5000], 'purple', linewidth=0.5)
    ax6.set_title('6. Après Compresseur 4:1', fontsize=12, fontweight='bold', color='purple')
    ax6.set_xlabel('Temps (s)')
    ax6.set_ylabel('Amplitude')
    ax6.grid(True, alpha=0.3)
    ax6.set_ylim([-1, 1])
    
    # Titre principal
    fig.suptitle('ANALYSE DES FILTRES AUDIO - Système d\'Enregistrement Teensy', 
                 fontsize=16, fontweight='bold', y=0.98)
    
    plt.savefig('audio_analysis_complete.png', dpi=300, bbox_inches='tight')
    print("✅ Graphique principal sauvegardé: audio_analysis_complete.png")

def main():
    print("\n" + "="*60)
    print("  📊 GÉNÉRATION DES GRAPHIQUES SCIENTIFIQUES")
    print("="*60 + "\n")
    
    # 1. Générer le signal
    print("🎤 Génération du signal de parole simulé...")
    t, raw_signal, clean_speech = generate_speech_signal()
    
    # 2. Concevoir les filtres
    print("🎛️  Conception des filtres (Butterworth 2nd order)...")
    sos_hp, sos_lp = design_filters()
    
    # 3. Appliquer les filtres
    print("⚙️  Application des filtres...")
    filtered_signal = apply_filters(raw_signal, sos_hp, sos_lp)
    
    # 4. Appliquer le compresseur
    print("📊 Application du compresseur dynamique...")
    compressed_signal = simple_compressor(filtered_signal)
    
    # 5. Créer les graphiques
    print("📈 Création des visualisations...\n")
    
    # Graphique des réponses en fréquence
    plot_filter_responses(sos_hp, sos_lp)
    
    # Graphique complet
    create_comprehensive_plot(t, raw_signal, filtered_signal, compressed_signal)
    
    # Statistiques
    print("\n" + "="*60)
    print("  📈 STATISTIQUES")
    print("="*60)
    print(f"Signal brut RMS:       {np.sqrt(np.mean(raw_signal**2)):.4f}")
    print(f"Signal filtré RMS:     {np.sqrt(np.mean(filtered_signal**2)):.4f}")
    print(f"Signal compressé RMS:  {np.sqrt(np.mean(compressed_signal**2)):.4f}")
    print(f"Réduction de bruit:    {20*np.log10(np.sqrt(np.mean(filtered_signal**2)) / np.sqrt(np.mean(raw_signal**2))):.2f} dB")
    print("="*60 + "\n")
    
    print("✨ TERMINÉ! Graphiques prêts pour votre poster scientifique.\n")
    print("📁 Fichiers générés:")
    print("   - audio_analysis_complete.png (graphique principal)")
    print("   - filter_response.png (réponse en fréquence)")
    print("\n")

if __name__ == "__main__":
    main()
