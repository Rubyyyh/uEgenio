import os
import time
import re
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

FS = 16000                    # Frequenza campionamento audio
DURATION = 4                  # Secondi di ascolto

WHISPER_MODEL_NAME = "small"  # Puoi mettere "base" se vuoi più velocità
GEMINI_MODEL_NAME = "gemini-2.5-flash-lite"

COMMAND_DELAY_SECONDS = 1.0

VALID_COMMANDS = {"1", "2", "3", "4", "5", "6", "7"}
MOTION_COMMANDS = {"4", "5", "7"}
ACTIVE_STATES = {"PATROL", "ARM_WAVE_LOOP", "ARM_PICK_PLACE"}

# Comando di sicurezza se Gemini risponde male o non dà numeri
DEFAULT_COMMAND = "1"

# Devi impostare la variabile d'ambiente da terminale Windows:
# setx GEMINI_API_KEY "LA_TUA_API_KEY"
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")

# ============================================================
# STATO ROBOT
# ============================================================

robot_state = "IDLE"

# Comandi STM32:
# 1 = faccia felice / frase positiva o neutra verso robot
# 2 = faccia triste / frase negativa/offensiva verso robot
# 3 = manda un bacio / kiss + heart
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
    print("⚠️ GEMINI_API_KEY non trovata. Userò il comando di fallback.\n")

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
# ANALISI COMANDI - GEMINI
# ============================================================

def parse_commands(raw: str) -> List[str]:
    """
    Estrae solo comandi validi da 1 a 7.
    Gemini deve rispondere sempre con numeri.
    Se non trova numeri validi, ritorna il comando di fallback.
    """

    if not raw:
        return [DEFAULT_COMMAND]

    raw = raw.strip()

    found = re.findall(r"\b[1-7]\b", raw)
    commands = [cmd for cmd in found if cmd in VALID_COMMANDS]

    if not commands:
        return [DEFAULT_COMMAND]

    cleaned: List[str] = []

    for cmd in commands:
        if not cleaned or cleaned[-1] != cmd:
            cleaned.append(cmd)

    return cleaned


def ai_commands(text: str, current_state: str) -> List[str]:
    if gemini_client is None:
        print(f"❌ Gemini non disponibile. Uso fallback: {DEFAULT_COMMAND}")
        return [DEFAULT_COMMAND]

    print("🤖 Analizzo comando con Gemini...")

    prompt = f"""
Una persona sta parlando a un robot STM32.

Devi trasformare la frase in una SEQUENZA di comandi numerici.

Rispondi SOLO con numeri da 1 a 7 separati da virgola.
Non scrivere spiegazioni.
Non scrivere parole.
Non usare NONE.
Non usare il comando 0.
Non usare punti.
Non aggiungere testo.

Comandi possibili:
1 = faccia felice, frase positiva verso il robot, frase neutra o interazione amichevole
2 = faccia triste, frase negativa/offensiva verso il robot
3 = manda un bacio / fai un bacio / dammi un bacio / fammi un bacio / bacino / kiss / manda un cuore / animazione kiss + heart
4 = inizia patrol mode / pattuglia / giro di controllo / controlla la zona
5 = saluto continuo con il braccio, cioe' il robot fa ciao con il braccio
6 = stop / ferma patrol / ferma saluto / ferma braccio / torna in idle / torna al centro
7 = prendi un oggetto, spostalo e posalo da un'altra parte

Stato stimato attuale del robot: {current_state}

Regole IMPORTANTI:
- Devi rispondere sempre almeno con un numero.
- Non esiste il comando 0.
- Non puoi rispondere NONE.
- Se la frase è poco chiara, scegli comunque il comando più probabile.
- Se la frase è generica, neutra o amichevole, rispondi 1.
- Se la frase sembra negativa, triste o offensiva, rispondi 2.
- Se la frase contiene sia uno stop sia una nuova azione, rispondi con 2 numeri.
- Esempio: "basta muovere il braccio e prendi un oggetto" -> 6,7
- Esempio: "ferma il saluto e fai la pattuglia" -> 6,4
- Esempio: "smetti di pattugliare e salutami" -> 6,5
- Se il robot deve solo fermarsi, rispondi 6.
- Se deve salutare con il braccio, rispondi 5.
- Se deve prendere, spostare o posare un oggetto, rispondi 7.
- Se deve iniziare il controllo della zona, la ronda o la pattuglia, rispondi 4.
- Se la frase chiede al robot di mandare un bacio, fare un bacio, dare un bacino, fare kiss, mandare un cuore o fare l'animazione del bacio, rispondi 3.
- Se parla bene del robot, rispondi 1.
- Se insulta il robot, rispondi 2.
- Se hai dubbio assoluto, rispondi 1.

Esempi:
"ciao robot" -> 1
"bravo robot" -> 1
"sei simpatico" -> 1
"sei brutto" -> 2
"fai schifo" -> 2
"manda un bacio" -> 3
"fammi un bacio" -> 3
"dammi un bacino" -> 3
"fai kiss" -> 3
"manda un cuore" -> 3
"controlla la zona" -> 4
"fai la pattuglia" -> 4
"saluta" -> 5
"fai ciao con il braccio" -> 5
"fermati" -> 6
"stop" -> 6
"prendi l'oggetto" -> 7
"sposta questo oggetto" -> 7

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
        print(f"⚠️ Uso fallback: {DEFAULT_COMMAND}")
        return [DEFAULT_COMMAND]


def normalize_command_sequence(commands: List[str], current_state: str) -> List[str]:
    """
    Normalizza la sequenza numerica.
    Non usa euristiche.
    Non analizza il testo.
    Non usa comando 0.
    Non usa NONE.
    """

    commands = [cmd for cmd in commands if cmd in VALID_COMMANDS]

    if not commands:
        commands = [DEFAULT_COMMAND]

    has_motion_command = any(cmd in MOTION_COMMANDS for cmd in commands)

    # Se il robot è già in una modalità attiva e arriva una nuova azione fisica,
    # prima manda stop per evitare conflitti.
    if current_state != "IDLE" and has_motion_command and commands[0] != "6":
        commands.insert(0, "6")

    cleaned: List[str] = []

    for cmd in commands:
        if not cleaned or cleaned[-1] != cmd:
            cleaned.append(cmd)

    return cleaned


def analyze_commands(text: str, current_state: str) -> List[str]:
    print("🤖 Analizzo frase...")

    selected = ai_commands(text, current_state)
    print(f"🧠 Comandi da Gemini: {selected}")

    final_commands = normalize_command_sequence(selected, current_state)

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
        commands = [DEFAULT_COMMAND]

    if ser is None or not ser.is_open:
        print("❌ Seriale non disponibile, comando non inviato.\n")
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
                print(f"⚠️ Nessuna frase riconosciuta. Uso fallback: {DEFAULT_COMMAND}")
                commands = [DEFAULT_COMMAND]
            else:
                commands = analyze_commands(text, robot_state)

            send_command_sequence(ser, commands)

            print("➡️ Premi INVIO per parlare di nuovo...")
            input()

        except KeyboardInterrupt:
            print("\n👋 Uscita dal programma.")
            break

        except Exception as e:
            print(f"❌ Errore generale: {e}\n")
            print(f"⚠️ Uso fallback: {DEFAULT_COMMAND}")

            send_command_sequence(ser, [DEFAULT_COMMAND])

            print("➡️ Premi INVIO per riprovare...")
            input()

    if ser is not None and ser.is_open:
        ser.close()
        print("🔌 Seriale chiusa.")


if __name__ == "__main__":
    main()
