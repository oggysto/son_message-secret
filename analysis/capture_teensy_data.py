#!/usr/bin/env python3
"""
Script pour capturer les données audio depuis le Teensy via Serial
Usage: python capture_teensy_data.py [port]
Exemple: python capture_teensy_data.py /dev/cu.usbmodem*
"""

import serial
import serial.tools.list_ports
import sys
import time
import csv

def find_teensy_port():
    """Trouve automatiquement le port du Teensy"""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if 'usb' in port.device.lower() or 'teensy' in port.description.lower():
            return port.device
    return None

def capture_data(port_name, baud_rate=9600, output_file='teensy_audio_data.csv'):
    """Capture les données audio depuis le Teensy"""
    
    print(f"🔌 Connexion au port: {port_name}")
    print(f"📊 Fichier de sortie: {output_file}")
    print("=" * 50)
    
    try:
        ser = serial.Serial(port_name, baud_rate, timeout=1)
        time.sleep(2)  # Attendre stabilisation
        
        print("✅ Connecté au Teensy!")
        print("\n📝 Instructions:")
        print("  1. Appuyez sur Bouton 0 pour ENREGISTRER votre voix")
        print("  2. Appuyez sur Bouton 2 pour EXPORTER les données")
        print("  3. Les données seront sauvegardées automatiquement\n")
        print("⏳ En attente des données...\n")
        
        capturing = False
        data_lines = []
        
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                
                # Afficher les messages du Teensy
                if not capturing:
                    print(line)
                
                # Détection du début des données
                if line == "DATA_START":
                    print("\n🚀 DÉBUT DE LA CAPTURE...")
                    capturing = True
                    data_lines = []
                    continue
                
                # Détection de la fin des données
                if line == "DATA_END":
                    print(f"✅ CAPTURE TERMINÉE - {len(data_lines)-1} samples reçus")
                    capturing = False
                    
                    # Sauvegarder les données
                    with open(output_file, 'w', newline='') as f:
                        f.write('\n'.join(data_lines))
                    
                    print(f"💾 Données sauvegardées dans: {output_file}")
                    print("\n✨ Vous pouvez maintenant lancer le script de visualisation!")
                    break
                
                # Collecter les données
                if capturing:
                    data_lines.append(line)
                    # Indicateur de progression
                    if len(data_lines) % 1000 == 0:
                        print(f"  📈 {len(data_lines)} samples capturés...")
        
        ser.close()
        print("\n🔌 Connexion fermée")
        return True
        
    except serial.SerialException as e:
        print(f"❌ Erreur de connexion: {e}")
        return False
    except KeyboardInterrupt:
        print("\n\n⚠️  Interruption utilisateur")
        if 'ser' in locals() and ser.is_open:
            ser.close()
        return False

def main():
    print("\n" + "=" * 50)
    print("  📡 CAPTURE DES DONNÉES TEENSY")
    print("=" * 50 + "\n")
    
    # Déterminer le port
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        port = find_teensy_port()
        if port is None:
            print("❌ Aucun Teensy détecté!")
            print("\n📋 Ports disponibles:")
            for p in serial.tools.list_ports.comports():
                print(f"  - {p.device}: {p.description}")
            print("\nUsage: python capture_teensy_data.py [port]")
            sys.exit(1)
    
    # Capturer
    capture_data(port)

if __name__ == "__main__":
    main()
