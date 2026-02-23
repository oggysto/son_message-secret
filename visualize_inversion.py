"""
Script de visualisation pour le poster - Message Secret
Affiche un signal audio et sa version inversée temporellement
"""

import numpy as np
import matplotlib.pyplot as plt

# Configuration pour un graphique sobre et élégant
plt.style.use('default')
plt.rcParams['figure.figsize'] = (10, 6)
plt.rcParams['font.size'] = 10
plt.rcParams['font.family'] = 'sans-serif'

# ============================================================
# GÉNÉRATION DU SIGNAL AUDIO SIMULÉ
# ============================================================

# Paramètres (similaires au Teensy)
SAMPLE_RATE = 44100  # Hz
DURATION = 1.0       # secondes
t = np.linspace(0, DURATION, int(SAMPLE_RATE * DURATION))

# Création d'un signal qui ressemble à de la parole
# (combinaison de plusieurs fréquences + enveloppe)
def generate_speech_like_signal(t):
    """Génère un signal qui ressemble à une phrase parlée"""
    
    # Fréquences fondamentales (simulant des phonèmes)
    f1 = 150  # Hz (fréquence de base voix masculine)
    f2 = 300
    f3 = 450
    
    # Signal de base avec harmoniques
    signal = np.sin(2 * np.pi * f1 * t)
    signal += 0.5 * np.sin(2 * np.pi * f2 * t)
    signal += 0.3 * np.sin(2 * np.pi * f3 * t)
    
    # Ajout de modulation (comme des syllabes)
    modulation_freq = 4  # Hz (4 syllabes par seconde)
    modulation = 0.5 + 0.5 * np.sin(2 * np.pi * modulation_freq * t)
    signal = signal * modulation
    
    # Enveloppe d'amplitude (début et fin progressifs)
    envelope = np.ones_like(t)
    fade_samples = int(0.05 * len(t))  # 5% fade in/out
    envelope[:fade_samples] = np.linspace(0, 1, fade_samples)
    envelope[-fade_samples:] = np.linspace(1, 0, fade_samples)
    signal = signal * envelope
    
    # Ajout de bruit léger (réalisme)
    noise = 0.05 * np.random.randn(len(t))
    signal = signal + noise
    
    # Normalisation
    signal = signal / np.max(np.abs(signal))
    
    return signal

# Générer le signal
signal_normal = generate_speech_like_signal(t)

# Inversion temporelle (comme dans le Teensy)
signal_inverse = signal_normal[::-1]

# ============================================================
# VISUALISATION
# ============================================================

fig, axes = plt.subplots(2, 1, figsize=(10, 6))

# Limiter l'affichage à une portion pour mieux voir la forme
zoom_duration = 0.2  # secondes
zoom_samples = int(zoom_duration * SAMPLE_RATE)
t_zoom = t[:zoom_samples]

# Couleurs sobres
color_normal = '#2c3e50'    # Bleu foncé
color_inverse = '#e74c3c'   # Rouge sobre

# ---- Graphique 1: Signal Original ----
axes[0].plot(t_zoom * 1000, signal_normal[:zoom_samples], 
             linewidth=1.2, color=color_normal)
axes[0].set_title('Signal Original', fontsize=12, fontweight='bold', pad=10)
axes[0].set_ylabel('Amplitude', fontsize=10)
axes[0].grid(True, alpha=0.2, linestyle='--')
axes[0].set_ylim([-1.1, 1.1])
axes[0].axhline(y=0, color='gray', linewidth=0.5, alpha=0.5)
axes[0].spines['top'].set_visible(False)
axes[0].spines['right'].set_visible(False)

# ---- Graphique 2: Signal Inversé ----
axes[1].plot(t_zoom * 1000, signal_inverse[:zoom_samples], 
             linewidth=1.2, color=color_inverse)
axes[1].set_title('Signal Inversé (Crypté)', fontsize=12, fontweight='bold', pad=10)
axes[1].set_xlabel('Temps (ms)', fontsize=10)
axes[1].set_ylabel('Amplitude', fontsize=10)
axes[1].grid(True, alpha=0.2, linestyle='--')
axes[1].set_ylim([-1.1, 1.1])
axes[1].axhline(y=0, color='gray', linewidth=0.5, alpha=0.5)
axes[1].spines['top'].set_visible(False)
axes[1].spines['right'].set_visible(False)

# Titre général
fig.suptitle('Inversion Temporelle du Signal Audio', 
             fontsize=14, fontweight='bold', y=0.98)

# Ajuster l'espacement
plt.tight_layout(rect=[0, 0, 1, 0.96])

# ============================================================
# AFFICHAGE ET SAUVEGARDE
# ============================================================

# Sauvegarder en haute résolution pour le poster
output_file = '/Users/oggysto/Documents/PlatformIO/Projects/SON_message_secret/signal_inversion_poster.png'
plt.savefig(output_file, dpi=300, bbox_inches='tight')
print(f"✅ Graphique sauvegardé : {output_file}")

# Afficher
plt.show()

# ============================================================
# STATISTIQUES
# ============================================================

print("\n📊 STATISTIQUES DU SIGNAL")
print("=" * 50)
print(f"Durée totale : {DURATION} secondes")
print(f"Fréquence d'échantillonnage : {SAMPLE_RATE} Hz")
print(f"Nombre d'échantillons : {len(signal_normal)}")
print(f"Amplitude max (original) : {np.max(signal_normal):.3f}")
print(f"Amplitude min (original) : {np.min(signal_normal):.3f}")
print(f"\n✅ Double inversion = signal original")
