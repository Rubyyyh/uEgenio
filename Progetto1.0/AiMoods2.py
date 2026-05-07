import os
import time
import re
import unicodedata
from typing import List, Optional

import sounddevice as sd
import numpy as np
import whisper
import serial
from google import genai
from google.genai import types

# ============================================================
# CONFIGURAZIONE
# ============================================================

MIC_INDEX = None              # None = microfono di default
SERIAL_PORT = "COM5"          # Cambia con la porta della tua STM32
BAUDRATE = 115200

FS = 16000                    # frequenza campionamento audio
DURATION = 4                  # secondi di ascolto

WHISPER_MODEL_NAME = "small"  # puoi mettere "base" se vuoi più velocità
GEMINI_MODEL_NAME = "gemini-2.5-flash-lite"

COMMAND_DELAY_SECONDS = 1.0

VALID_COMMANDS = {"0", "1", "2", "3", "4", "5", "6", "7"}
MOTION_COMMANDS = {"4", "5", "7"}
ACTIVE_STATES = {"PATROL", "ARM_WAVE_LOOP", "ARM_PICK_PLACE"}

# Devi impostare la variabile d'ambiente da terminale Windows:
# setx GEMINI_API_KEY "LA_TUA_API_KEY"
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")

# ============================================================
# STATO ROBOT
# ============================================================

robot_state = "IDLE"

# Comandi STM32:
# 0 = nessuna azione
# 1 = faccia felice / frase positiva verso robot
# 2 = faccia triste / frase negativa verso robot
# 3 = animazione preside / kiss + heart
# 4 = patrol mode / giro di controllo
# 5 = saluto continuo con il braccio
# 6 = stop / ritorno IDLE / ferma patrol o braccio
# 7 = prendi oggetto, spostalo e posalo una volta

# ============================================================
# AVVIO MODELLI
# ============================================================

print("🔊 Carico Whisper...")
whisper_model = whisper.load_model(WHISPER_MODEL_NAME)
print("✅ Whisper pronto.")

if GEMINI_API_KEY:
    gemini_client = genai.Client(api_key=GEMINI_API_KEY)
    print("✅ Gemini pronto.\n")
else:
    gemini_client = None
    print("⚠️ GEMINI_API_KEY non trovata. Userò solo le regole rapide.\n")

# ============================================================
# AUDIO
# ============================================================

def record_audio() -> np.ndarray:
    print("🎤 Sto ascoltando...")
    audio = sd.rec(
        int(DURATION * FS),
        samplerate=FS,
        channels=1,
        dtype="float32",
        device=MIC_INDEX
    )
    sd.wait()
    return audio.flatten()


def transcribe(audio: np.ndarray) -> str:
    print("📝 Trascrivo...")
    result = whisper_model.transcribe(
        audio,
        fp16=False,
        language="it"
    )
    return result.get("text", "").strip()

# ============================================================
# UTILITY TESTO
# ============================================================

def normalize_text(text: str) -> str:
    text = text.lower().strip()
    text = unicodedata.normalize("NFD", text)
    text = "".join(ch for ch in text if unicodedata.category(ch) != "Mn")
    return text


def contains_any(text: str, words: List[str]) -> bool:
    return any(word in text for word in words)

# ============================================================
# ANALISI COMANDI - REGOLE RAPIDE
# ============================================================

def heuristic_commands(text: str) -> List[str]:
    t = normalize_text(text)

    stop_words = [
        "fermati", "ferma", "stop", "basta", "smetti", "interrompi", "annulla",
        "blocca", "arresta", "torna al centro", "torna in idle",
        "esci dalla patrol", "esci dalla pattuglia", "non muovere",
        "basta muovere", "smetti di muovere", "non salutare",
        "smetti di salutare", "ferma il braccio", "ferma la pattuglia"
    ]

    patrol_words = [
        "patrol", "pattuglia", "pattugliare", "controlla la zona",
        "giro di controllo", "sorveglia", "ronda", "controlla l'area",
        "controlla area", "fai un giro"
    ]

    wave_words = [
        "saluta", "saluto", "fai ciao", "fai un ciao", "ciao con la mano",
        "muovi la mano", "saluta con il braccio", "fai un saluto", "wave"
    ]

    pick_place_words = [
        "prendi", "prendere", "afferra", "raccogli", "sposta", "posa",
        "posalo", "mettilo", "porta", "preleva", "oggetto", "pick",
        "place", "pick and place"
    ]

    positive_robot_words = [
        "bravo", "sei bravo", "sei bello", "sei bellissimo", "ottimo",
        "grande robot", "mi piaci", "sei fantastico", "sei forte", "sei simpatico"
    ]

    negative_robot_words = [
        "fai schifo", "sei inutile", "stupido", "cretino", "brutto",
        "non servi", "sei scarso", "sei pessimo"
    ]

    preside_words = ["preside", "dirigente"]
    positive_preside_words = [
        "brava", "bravissima", "grande", "gentile", "simpatica",
        "bella", "fantastica", "forte"
    ]

    wants_stop = contains_any(t, stop_words)
    wants_patrol = contains_any(t, patrol_words)
    wants_wave = contains_any(t, wave_words)
    wants_pick_place = contains_any(t, pick_place_words)
    wants_positive_robot = contains_any(t, positive_robot_words)
    wants_negative_robot = contains_any(t, negative_robot_words)
    mentions_preside = contains_any(t, preside_words)
    positive_preside = mentions_preside and contains_any(t, positive_preside_words)

    commands: List[str] = []

    if wants_stop:
        commands.append("6")

    # Priorità: oggetto > saluto > patrol
    if wants_pick_place:
        commands.append("7")
    elif wants_wave:
        commands.append("5")
    elif wants_patrol:
        commands.append("4")

    # Mood solo se non ci sono già azioni fisiche
    if not commands:
        if positive_preside:
            commands.append("3")
        elif wants_negative_robot:
            commands.append("2")
        elif wants_positive_robot:
            commands.append("1")

    return commands

# ============================================================
# ANALISI COMANDI - GEMINI
# ============================================================

def parse_commands(raw: str) -> List[str]:
    found = re.findall(r"[0-7]", raw)
    commands = [cmd for cmd in found if cmd in VALID_COMMANDS]

    if len(commands) > 1 and "0" in commands:
        commands = [cmd for cmd in commands if cmd != "0"]

    cleaned: List[str] = []
    for cmd in commands:
        if not cleaned or cleaned[-1] != cmd:
            cleaned.append(cmd)

    return cleaned


def ai_commands(text: str, current_state: str) -> List[str]:
    if gemini_client is None:
        return []

    print("🤖 Analizzo comando con Gemini...")

    prompt = f"""
Una persona sta parlando a un robot STM32.

Devi trasformare la frase in una SEQUENZA di comandi numerici.
Rispondi SOLO con numeri separati da virgola.
Non scrivere spiegazioni.
Non scrivere parole.
Non usare punti.

Comandi possibili:
0 = nessuna azione / frase non rilevante
1 = faccia felice, frase positiva verso il robot
2 = faccia triste, frase negativa/offensiva verso il robot
3 = animazione preside / kiss + heart / frase positiva sulla preside
4 = inizia patrol mode / pattuglia / giro di controllo
5 = saluto continuo con il braccio, cioe' il robot fa ciao con il braccio
6 = stop / ferma patrol / ferma saluto / ferma braccio / torna in idle / torna al centro
7 = prendi un oggetto, spostalo e posalo da un'altra parte

Stato stimato attuale del robot: {current_state}

Regole IMPORTANTI:
- Se la frase contiene sia uno stop sia una nuova azione, rispondi con 2 numeri.
- Esempio: "basta muovere il braccio e prendi un oggetto" -> 6,7
- Esempio: "ferma il saluto e fai la pattuglia" -> 6,4
- Esempio: "smetti di pattugliare e salutami" -> 6,5
- Se il robot deve solo fermarsi, rispondi 6.
- Se deve salutare con il braccio, rispondi 5.
- Se deve prendere/spostare/posare un oggetto, rispondi 7.
- Se deve iniziare il controllo della zona o la pattuglia, rispondi 4.
- Se parla bene della preside, rispondi 3.
- Se parla bene del robot, rispondi 1.
- Se insulta il robot, rispondi 2.
- Se non capisci o non e' rilevante, rispondi 0.

Frase da analizzare:
{text}
"""

    try:
        response = gemini_client.models.generate_content(
            model=GEMINI_MODEL_NAME,
            contents=prompt,
            config=types.GenerateContentConfig(
                temperature=0,
                max_output_tokens=10
            )
        )

        raw = response.text or ""
        print(f"🔎 Output grezzo Gemini: {repr(raw)}")
        return parse_commands(raw)

    except Exception as e:
        print(f"❌ Errore Gemini: {e}")
        return []


def normalize_command_sequence(commands: List[str], text: str, current_state: str) -> List[str]:
    commands = [cmd for cmd in commands if cmd in VALID_COMMANDS]

    if not commands:
        return []

    if commands == ["0"]:
        return ["0"]

    commands = [cmd for cmd in commands if cmd != "0"]

    t = normalize_text(text)
    stop_requested = contains_any(t, [
        "fermati", "ferma", "stop", "basta", "smetti", "interrompi", "annulla",
        "torna al centro", "torna in idle", "ferma il braccio", "ferma la pattuglia",
        "smetti di salutare", "basta muovere"
    ])

    has_motion_command = any(cmd in MOTION_COMMANDS for cmd in commands)

    # Stop + nuova azione nella stessa frase: forza 6 davanti
    if stop_requested and has_motion_command and commands[0] != "6":
        commands.insert(0, "6")

    # Se robot già in modalità attiva e arriva nuova azione fisica: stop prima
    if current_state != "IDLE" and has_motion_command and commands[0] != "6":
        commands.insert(0, "6")

    cleaned: List[str] = []
    for cmd in commands:
        if not cleaned or cleaned[-1] != cmd:
            cleaned.append(cmd)

    return cleaned


def analyze_commands(text: str, current_state: str) -> List[str]:
    print("🤖 Analizzo frase...")

    heuristic = heuristic_commands(text)

    # Risparmia API: Gemini viene usato solo se le regole non capiscono
    if heuristic:
        selected = heuristic
        print(f"🧠 Comandi da euristica: {selected}")
    else:
        selected = ai_commands(text, current_state)
        print(f"🧠 Comandi da Gemini: {selected}")

    final_commands = normalize_command_sequence(selected, text, current_state)

    if not final_commands:
        final_commands = ["0"]

    print(f"🎭 Comandi finali: {final_commands}")
    return final_commands

# ============================================================
# SERIALE STM32
# ============================================================

def connect_serial() -> Optional[serial.Serial]:
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
        time.sleep(2)
        print(f"✅ Seriale aperta su {SERIAL_PORT} a {BAUDRATE} baud\n")
        return ser
    except Exception as e:
        print(f"❌ Errore apertura seriale: {e}\n")
        return None


def update_robot_state_after_command(cmd: str) -> None:
    global robot_state

    if cmd == "4":
        robot_state = "PATROL"
    elif cmd == "5":
        robot_state = "ARM_WAVE_LOOP"
    elif cmd == "7":
        robot_state = "ARM_PICK_PLACE"
    elif cmd == "6":
        robot_state = "IDLE"
    elif cmd in {"1", "2", "3"}:
        if robot_state not in ACTIVE_STATES:
            robot_state = "IDLE"


def send_command(ser: serial.Serial, cmd: str) -> None:
    ser.write(f"{cmd}\n".encode())
    ser.flush()
    print(f"📡 Inviato comando {cmd} ad STM")
    update_robot_state_after_command(cmd)


def send_command_sequence(ser: Optional[serial.Serial], commands: List[str]) -> None:
    if not commands:
        print("⚠️ Nessun comando da inviare\n")
        return

    if commands == ["0"]:
        print("ℹ️ Comando 0: nessuna azione\n")
        return

    if ser is None or not ser.is_open:
        print("❌ Seriale non disponibile, comando non inviato\n")
        return

    try:
        for index, cmd in enumerate(commands):
            send_command(ser, cmd)

            if index < len(commands) - 1:
                print(f"⏳ Aspetto {COMMAND_DELAY_SECONDS:.1f}s prima del prossimo comando...")
                time.sleep(COMMAND_DELAY_SECONDS)

        print(f"🤖 Stato stimato robot: {robot_state}\n")

    except Exception as e:
        print(f"❌ Errore invio seriale: {e}\n")

# ============================================================
# LOOP PRINCIPALE
# ============================================================

def main() -> None:
    ser = connect_serial()

    print("➡️ Premi INVIO per iniziare...")
    input()

    while True:
        try:
            audio = record_audio()
            text = transcribe(audio)

            print(f"🗣️ Hai detto: {text}")

            if not text:
                print("⚠️ Nessuna frase riconosciuta\n")
                print("➡️ Premi INVIO per parlare di nuovo...")
                input()
                continue

            commands = analyze_commands(text, robot_state)
            send_command_sequence(ser, commands)

            print("➡️ Premi INVIO per parlare di nuovo...")
            input()

        except KeyboardInterrupt:
            print("\n👋 Uscita dal programma.")
            break

        except Exception as e:
            print(f"❌ Errore generale: {e}\n")
            print("➡️ Premi INVIO per riprovare...")
            input()

    if ser is not None and ser.is_open:
        ser.close()
        print("🔌 Seriale chiusa.")


if __name__ == "__main__":
    main()
