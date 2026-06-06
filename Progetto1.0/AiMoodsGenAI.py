"""
Controller vocale GenAI per uEgenio.

Obiettivi principali:
- usare regole locali prima di Gemini, così i comandi chiari sono immediati;
- usare Gemini solo quando serve davvero;
- interpretare correttamente comandi come "mandami un bacio" / "manda un cuore" -> 3;
- evitare fallback pericolosi: se non capisce, non invia comandi casuali al robot.

Prerequisiti principali:
    pip install sounddevice numpy openai-whisper pyserial google-genai

Prima dell'avvio imposta la chiave Gemini:
    setx GEMINI_API_KEY "LA_TUA_API_KEY"

Avvio consigliato dalla cartella Progetto1.0:
    python AiMoodsGenAI.py

Test senza microfono/seriale:
    python AiMoodsGenAI.py --self-test
    python AiMoodsGenAI.py --text "mandami un bacio" --no-serial
"""

from __future__ import annotations

import argparse
import json
import os
import re
import time
import unicodedata
from dataclasses import dataclass
from typing import Any, Iterable, List, Optional, Sequence

# Le dipendenze pesanti audio/seriale vengono importate solo quando servono.
# Così `--self-test` e `--text ... --no-serial` funzionano anche senza microfono,
# porta seriale o Whisper già pronti.
genai: Any = None
types: Any = None
gemini_client_cache: Any = None
gemini_client_checked = False
np: Any = None
serial: Any = None
sd: Any = None
whisper: Any = None


# ============================================================
# CONFIGURAZIONE
# ============================================================

MIC_INDEX = None                  # None = microfono di default
SERIAL_PORT = "COM5"              # Cambia con la porta della tua STM32
BAUDRATE = 115200

FS = 16000                        # Frequenza campionamento audio per Whisper
DURATION = 4.0                    # Secondi di ascolto per ogni comando
MIN_AUDIO_RMS = 0.003             # Sotto questa soglia evita trascrizioni casuali

# True  = prova prima le regole locali veloci, poi Gemini se serve.
# False = salta completamente le regole locali e usa direttamente Gemini.
USE_HEURISTIC = True

WHISPER_MODEL_NAME = os.getenv("WHISPER_MODEL_NAME", "small")
GEMINI_MODEL_NAME = os.getenv("GEMINI_MODEL_NAME", "gemini-2.5-flash-lite")
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY")

COMMAND_DELAY_SECONDS = 1.0       # Pausa tra comandi multipli, es. 6 poi 7

VALID_COMMANDS = {"0", "1", "2", "3", "4", "5", "6", "7"}
REAL_COMMANDS = {"1", "2", "3", "4", "5", "6", "7"}
MOTION_COMMANDS = {"4", "5", "7"}
COMMANDS_REQUIRING_IDLE = {"1", "2", "3", "4", "5", "7"}
ACTIVE_STATES = {"PATROL", "ARM_WAVE_LOOP", "ARM_PICK_PLACE"}


# ============================================================
# STATO ROBOT STIMATO
# ============================================================

robot_state = "IDLE"


@dataclass(frozen=True)
class CommandMatch:
    command: str
    position: int


# ============================================================
# UTILITY TESTO
# ============================================================

def normalize_text(text: str) -> str:
    """Minuscolo, senza accenti, con punteggiatura semplificata."""
    text = text.lower().strip()
    text = unicodedata.normalize("NFD", text)
    text = "".join(ch for ch in text if unicodedata.category(ch) != "Mn")
    text = re.sub(r"[^a-z0-9\s']+", " ", text)
    text = re.sub(r"\s+", " ", text).strip()
    return text


def find_first(text: str, phrases: Iterable[str]) -> Optional[int]:
    """Ritorna la posizione della prima frase trovata, oppure None."""
    positions: List[int] = []
    for phrase in phrases:
        normalized_phrase = normalize_text(phrase)
        if not normalized_phrase:
            continue

        pattern = rf"(?<![a-z0-9]){re.escape(normalized_phrase)}(?![a-z0-9])"
        match = re.search(pattern, text)
        if match:
            positions.append(match.start())

    return min(positions) if positions else None


def find_all_spans(text: str, phrases: Iterable[str]) -> List[tuple[int, int]]:
    """Trova tutte le occorrenze delle frasi, come intervalli [inizio, fine)."""
    spans: List[tuple[int, int]] = []
    for phrase in phrases:
        normalized_phrase = normalize_text(phrase)
        if not normalized_phrase:
            continue

        pattern = rf"(?<![a-z0-9]){re.escape(normalized_phrase)}(?![a-z0-9])"
        for match in re.finditer(pattern, text):
            spans.append((match.start(), match.end()))

    return spans


def is_inside_spans(position: int, spans: Sequence[tuple[int, int]]) -> bool:
    return any(start <= position < end for start, end in spans)


def contains_any(text: str, phrases: Iterable[str]) -> bool:
    return find_first(text, phrases) is not None


def dedupe_consecutive(commands: Sequence[str]) -> List[str]:
    cleaned: List[str] = []
    for cmd in commands:
        if cmd in VALID_COMMANDS and (not cleaned or cleaned[-1] != cmd):
            cleaned.append(cmd)
    return cleaned


# ============================================================
# REGOLE LOCALI VELOCI
# ============================================================

STOP_PHRASES = [
    "fermati", "ferma", "stop", "basta", "smetti", "interrompi", "annulla",
    "blocca", "arresta", "torna al centro", "torna in idle", "torna fermo",
    "esci dalla patrol", "esci dalla pattuglia", "ferma la patrol", "ferma la pattuglia",
    "smetti di pattugliare", "basta pattugliare", "ferma il giro", "ferma il controllo",
    "non muovere", "basta muovere", "smetti di muovere", "ferma il braccio",
    "non salutare", "smetti di salutare", "basta salutare", "ferma il saluto",
]

KISS_PHRASES = [
    "bacio", "bacino", "baciami", "kiss", "cuore", "cuoricino",
    "manda un bacio", "mandami un bacio", "fai un bacio", "fammi un bacio",
    "dammi un bacio", "dammi un bacino", "manda un bacino", "mandami un bacino",
    "manda un cuore", "mandami un cuore", "fai un cuore", "fammi un cuore",
    "animazione bacio", "animazione kiss",
]

PATROL_PHRASES = [
    "patrol", "pattuglia", "pattugliare", "fai la pattuglia", "inizia la pattuglia",
    "controlla la zona", "controlla l area", "controlla area", "giro di controllo",
    "fai un giro", "sorveglia", "ronda", "perlustra", "controlla intorno",
]

WAVE_PHRASES = [
    "saluta", "salutami", "saluto", "fai ciao", "fai un ciao", "ciao con la mano",
    "muovi la mano", "saluta con il braccio", "fai un saluto", "wave",
]

PICK_PLACE_PHRASES = [
    "prendi", "prendere", "afferra", "raccogli", "sposta", "posa", "posalo",
    "mettilo", "porta", "preleva", "oggetto", "pick", "place", "pick and place",
]

POSITIVE_ROBOT_PHRASES = [
    "bravo", "sei bravo", "sei bello", "sei bellissimo", "ottimo", "grande robot",
    "mi piaci", "sei fantastico", "sei forte", "sei simpatico", "ben fatto",
]

NEGATIVE_ROBOT_PHRASES = [
    "fai schifo", "sei inutile", "stupido", "cretino", "brutto", "non servi",
    "sei scarso", "sei pessimo", "sei cattivo", "non mi piaci",
]

PRESIDE_PHRASES = ["preside", "dirigente"]

POSITIVE_PRESIDE_PHRASES = [
    "brava", "bravissima", "grande", "gentile", "simpatica", "bella", "fantastica", "forte",
]


def heuristic_commands(text: str) -> List[str]:
    """
    Riconosce localmente i comandi più frequenti.

    La priorità è intenzionale:
    1. stop, perché può precedere una nuova azione;
    2. azioni fisiche e bacio/cuore;
    3. mood positivi/negativi.
    """
    t = normalize_text(text)
    if not t:
        return []

    matches: List[CommandMatch] = []
    stop_spans = find_all_spans(t, STOP_PHRASES)

    phrase_groups = [
        ("6", STOP_PHRASES),
        ("3", KISS_PHRASES),
        ("7", PICK_PLACE_PHRASES),
        ("5", WAVE_PHRASES),
        ("4", PATROL_PHRASES),
    ]

    for command, phrases in phrase_groups:
        pos = find_first(t, phrases)
        if pos is not None:
            # Evita falsi positivi come "ferma il saluto" -> 6,5.
            # In quel caso "saluto" è dentro una frase di stop e non è una nuova azione.
            if command != "6" and is_inside_spans(pos, stop_spans):
                continue

            matches.append(CommandMatch(command, pos))

    if matches:
        # Se c'è stop + nuova azione, lo stop deve sempre essere inviato prima,
        # anche se nella frase appare dopo: "fai pattuglia e poi fermati" è raro,
        # ma è più sicuro non avviare una nuova azione senza prima fermare quella attuale.
        commands = [m.command for m in sorted(matches, key=lambda m: m.position)]
        if "6" in commands and len(commands) > 1:
            commands = ["6"] + [cmd for cmd in commands if cmd != "6"]

        # Se una frase contiene "saluta e manda un bacio", il robot non può fare due
        # animazioni fisiche contemporaneamente; teniamo l'ordine naturale della frase.
        return dedupe_consecutive(commands)

    positive_preside = contains_any(t, PRESIDE_PHRASES) and contains_any(t, POSITIVE_PRESIDE_PHRASES)
    if positive_preside:
        # Nel firmware il comando 3 è kiss+heart; è usato anche come animazione positiva speciale.
        return ["3"]

    if contains_any(t, NEGATIVE_ROBOT_PHRASES):
        return ["2"]

    if contains_any(t, POSITIVE_ROBOT_PHRASES):
        return ["1"]

    return []


# ============================================================
# GEMINI
# ============================================================

def ensure_gemini_dependency() -> bool:
    """Importa google-genai solo quando serve chiamare Gemini."""
    global genai, types

    if genai is not None and types is not None:
        return True

    try:
        from google import genai as _genai
        from google.genai import types as _types
    except ImportError:
        print("⚠️ Libreria google-genai non installata. Gemini disattivato.")
        print("   Per abilitarlo: pip install google-genai")
        return False

    genai = _genai
    types = _types
    return True


def create_gemini_client():
    if not GEMINI_API_KEY:
        print("⚠️ GEMINI_API_KEY non trovata. Userò solo le regole locali.")
        print("   Per abilitarla: setx GEMINI_API_KEY \"LA_TUA_API_KEY\"")
        return None

    if not ensure_gemini_dependency():
        return None

    return genai.Client(api_key=GEMINI_API_KEY)


def get_gemini_client():
    """Inizializza Gemini una sola volta e solo se le euristiche non bastano."""
    global gemini_client_cache, gemini_client_checked

    if gemini_client_checked:
        return gemini_client_cache

    gemini_client_checked = True
    gemini_client_cache = create_gemini_client()
    return gemini_client_cache


def build_prompt(text: str, current_state: str) -> str:
    return f"""
Sei il classificatore comandi di un robot STM32 chiamato uEgenio.

Devi convertire una frase italiana in una sequenza di comandi numerici.

Rispondi SOLO con JSON valido nel formato:
{{"commands":["NUMERO"]}}
oppure:
{{"commands":["6","7"]}}

Non aggiungere spiegazioni, markdown o testo fuori dal JSON.

Comandi validi:
"0" = nessuna azione, frase non rilevante o non chiara
"1" = faccia felice, complimento o frase positiva verso il robot
"2" = faccia triste, insulto o frase negativa verso il robot
"3" = bacio/cuore/kiss: "manda un bacio", "mandami un bacio", "fammi un bacio", "dammi un bacino", "manda un cuore"
"4" = patrol/pattuglia/controllo zona/giro di controllo
"5" = saluto con il braccio/fai ciao/salutami
"6" = stop/fermati/smetti/ferma braccio/ferma patrol/torna idle
"7" = prendi/afferra/raccogli/sposta/posa un oggetto

Stato stimato attuale del robot: {current_state}

Regole:
- Se la frase contiene uno stop e poi una nuova azione, metti prima "6".
- "bacio", "bacino", "kiss", "cuore" significano SEMPRE comando "3".
- Se non sei sicuro, usa "0" invece di inventare un comando.
- Usa solo stringhe numeriche tra "0" e "7".

Esempi:
"mandami un bacio" -> {{"commands":["3"]}}
"manda un cuore" -> {{"commands":["3"]}}
"dammi un bacino" -> {{"commands":["3"]}}
"fermati e manda un bacio" -> {{"commands":["6","3"]}}
"ferma il saluto e prendi l'oggetto" -> {{"commands":["6","7"]}}
"smetti di pattugliare e salutami" -> {{"commands":["6","5"]}}
"controlla la zona" -> {{"commands":["4"]}}
"sei brutto" -> {{"commands":["2"]}}
"che ore sono" -> {{"commands":["0"]}}

Frase da classificare:
{text}
""".strip()


def parse_ai_commands(raw: str) -> List[str]:
    """Parsing restrittivo dell'output Gemini."""
    if not raw:
        return []

    raw = raw.strip()

    # Caso ideale: JSON puro.
    try:
        data = json.loads(raw)
        commands = data.get("commands", []) if isinstance(data, dict) else []
        if isinstance(commands, list):
            parsed = [str(cmd).strip() for cmd in commands if str(cmd).strip() in VALID_COMMANDS]
            return dedupe_consecutive(parsed)
    except json.JSONDecodeError:
        pass

    # Caso Gemini aggiunga testo per errore: estrai solo il primo oggetto JSON.
    match = re.search(r"\{.*\}", raw, flags=re.DOTALL)
    if match:
        try:
            data = json.loads(match.group(0))
            commands = data.get("commands", []) if isinstance(data, dict) else []
            if isinstance(commands, list):
                parsed = [str(cmd).strip() for cmd in commands if str(cmd).strip() in VALID_COMMANDS]
                return dedupe_consecutive(parsed)
        except json.JSONDecodeError:
            pass

    # Fallback controllato per risposte tipo "6,7". Non prende cifre dentro numeri lunghi.
    if re.fullmatch(r"\s*[0-7](\s*,\s*[0-7])*\s*", raw):
        return dedupe_consecutive(re.findall(r"\b[0-7]\b", raw))

    return []


def ai_commands(client, text: str, current_state: str) -> List[str]:
    if client is None:
        return []

    print("🤖 Regole locali insufficienti: chiedo a Gemini...")
    prompt = build_prompt(text, current_state)

    try:
        response = client.models.generate_content(
            model=GEMINI_MODEL_NAME,
            contents=prompt,
            config=types.GenerateContentConfig(
                temperature=0,
                max_output_tokens=40,
                response_mime_type="application/json",
            ),
        )
        raw = response.text or ""
        print(f"🔎 Output grezzo Gemini: {raw!r}")
        return parse_ai_commands(raw)
    except Exception as exc:
        print(f"❌ Errore Gemini: {exc}")
        return []


# ============================================================
# NORMALIZZAZIONE SEQUENZA
# ============================================================

def normalize_command_sequence(commands: Sequence[str], text: str, current_state: str) -> List[str]:
    commands = [cmd for cmd in commands if cmd in VALID_COMMANDS]
    commands = dedupe_consecutive(commands)

    if not commands:
        return []

    # "0" è solo interno: non deve essere inviato se ci sono comandi reali.
    if len(commands) > 1 and "0" in commands:
        commands = [cmd for cmd in commands if cmd != "0"]

    if commands == ["0"]:
        return []

    t = normalize_text(text)
    stop_requested = contains_any(t, STOP_PHRASES)
    has_command_requiring_idle = any(cmd in COMMANDS_REQUIRING_IDLE for cmd in commands)

    # Se l'utente chiede stop + nuova azione, garantisce 6 davanti.
    if stop_requested and len(commands) > 1 and commands[0] != "6":
        commands.insert(0, "6")

    # Il firmware accetta 1/2/3/4/5/7 solo in IDLE; in PATROL/ARM ignora tutto tranne 6.
    # Quindi, se il robot è stimato attivo, manda prima 6 anche per bacio/cuore o mood.
    if current_state in ACTIVE_STATES and has_command_requiring_idle and commands[0] != "6":
        commands.insert(0, "6")

    return dedupe_consecutive(commands)


def analyze_commands(client, text: str, current_state: str) -> List[str]:
    print("🧠 Analizzo frase...")

    heuristic = heuristic_commands(text) if USE_HEURISTIC else []

    if heuristic:
        print(f"⚡ Comandi da regole locali: {heuristic}")
        selected = heuristic
    else:
        if not USE_HEURISTIC:
            print("⚙️ Euristica disattivata: uso direttamente Gemini.")

        if client is None:
            client = get_gemini_client()
        selected = ai_commands(client, text, current_state)
        print(f"🧠 Comandi da Gemini: {selected}")

    final_commands = normalize_command_sequence(selected, text, current_state)
    print(f"🎭 Comandi finali: {final_commands if final_commands else '[nessun invio]'}")
    return final_commands


# ============================================================
# AUDIO / WHISPER
# ============================================================

def ensure_audio_dependencies() -> None:
    """Importa le dipendenze audio solo quando servono davvero."""
    global np, sd, whisper

    if np is not None and sd is not None and whisper is not None:
        return

    try:
        import numpy as _np
        import sounddevice as _sd
        import whisper as _whisper
    except ImportError as exc:
        raise RuntimeError(
            "Dipendenza audio mancante. Installa con: "
            "pip install sounddevice numpy openai-whisper"
        ) from exc

    np = _np
    sd = _sd
    whisper = _whisper


def load_whisper_model():
    ensure_audio_dependencies()
    print(f"🔊 Carico Whisper ({WHISPER_MODEL_NAME})...")
    model = whisper.load_model(WHISPER_MODEL_NAME)
    print("✅ Whisper pronto.")
    return model


def record_audio() -> np.ndarray:
    ensure_audio_dependencies()
    print("🎤 Sto ascoltando...")
    audio = sd.rec(
        int(DURATION * FS),
        samplerate=FS,
        channels=1,
        dtype="float32",
        device=MIC_INDEX,
    )
    sd.wait()
    return audio.flatten()


def audio_rms(audio: np.ndarray) -> float:
    ensure_audio_dependencies()
    if audio.size == 0:
        return 0.0
    return float(np.sqrt(np.mean(np.square(audio))))


def transcribe(model, audio: np.ndarray) -> str:
    rms = audio_rms(audio)
    print(f"📈 Volume RMS: {rms:.5f}")
    if rms < MIN_AUDIO_RMS:
        print("⚠️ Audio troppo basso/silenzio: salto la trascrizione.")
        return ""

    print("📝 Trascrivo...")
    result = model.transcribe(audio, fp16=False, language="it")
    return result.get("text", "").strip()


# ============================================================
# SERIALE STM32
# ============================================================

def ensure_serial_dependency() -> bool:
    """Importa pyserial solo se bisogna davvero usare la seriale."""
    global serial

    if serial is not None:
        return True

    try:
        import serial as _serial
    except ImportError:
        print("⚠️ Dipendenza pyserial mancante: la seriale sarà disattivata.")
        print("   Installa con: pip install pyserial")
        return False

    serial = _serial
    return True


def connect_serial() -> Optional[serial.Serial]:
    if not ensure_serial_dependency():
        return None

    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
        time.sleep(2)  # tempo per eventuale reset STM quando si apre la seriale
        print(f"✅ Seriale aperta su {SERIAL_PORT} a {BAUDRATE} baud\n")
        return ser
    except Exception as exc:
        print(f"❌ Errore apertura seriale: {exc}")
        print("   Il programma continuerà: analizzerà i comandi ma non li invierà.\n")
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
    elif cmd in {"1", "2", "3"} and robot_state not in ACTIVE_STATES:
        robot_state = "IDLE"


def send_command(ser: serial.Serial, cmd: str) -> None:
    # Alla STM32 deve arrivare SOLO il numero del comando, nessun newline/testo/JSON.
    ser.write(cmd.encode("ascii"))
    ser.flush()
    print(f"📡 Inviato comando {cmd} ad STM")
    update_robot_state_after_command(cmd)


def send_command_sequence(ser: Optional[serial.Serial], commands: Sequence[str]) -> None:
    commands = [cmd for cmd in commands if cmd in REAL_COMMANDS]
    if not commands:
        print("ℹ️ Nessun comando inviato.\n")
        return

    if ser is None or not ser.is_open:
        print(f"❌ Seriale non disponibile, comandi non inviati: {commands}\n")
        return

    try:
        for index, cmd in enumerate(commands):
            send_command(ser, cmd)
            if index < len(commands) - 1:
                print(f"⏳ Aspetto {COMMAND_DELAY_SECONDS:.1f}s prima del prossimo comando...")
                time.sleep(COMMAND_DELAY_SECONDS)

        print(f"🤖 Stato stimato robot: {robot_state}\n")
    except Exception as exc:
        print(f"❌ Errore invio seriale: {exc}\n")


# ============================================================
# LOOP PRINCIPALE
# ============================================================

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Controller vocale GenAI/Gemini per uEgenio."
    )
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="Esegue test locali su euristiche e parser, senza microfono/seriale/Gemini.",
    )
    parser.add_argument(
        "--text",
        type=str,
        default=None,
        help="Analizza una frase singola da terminale invece di usare il microfono.",
    )
    parser.add_argument(
        "--no-serial",
        action="store_true",
        help="Non apre la porta seriale e stampa soltanto i comandi calcolati.",
    )
    parser.add_argument(
        "--serial-port",
        type=str,
        default=SERIAL_PORT,
        help=f"Porta seriale STM32, default: {SERIAL_PORT}.",
    )
    return parser.parse_args()


def run_self_test() -> int:
    tests = [
        ("mandami un bacio", ["3"]),
        ("manda un cuore", ["3"]),
        ("dammi un bacino", ["3"]),
        ("fermati e manda un bacio", ["6", "3"]),
        ("ferma il saluto e prendi l oggetto", ["6", "7"]),
        ("smetti di pattugliare e salutami", ["6", "5"]),
        ("controlla la zona", ["4"]),
        ("salutami", ["5"]),
        ("sei brutto", ["2"]),
        ("bravo robot", ["1"]),
        ("ferma il saluto", ["6"]),
        ("smetti di pattugliare", ["6"]),
        ("questa cosa è importante", []),
        ("che ore sono", []),
    ]

    parser_tests = [
        ('{"commands":["3"]}', ["3"]),
        ('{"commands":["6","7"]}', ["6", "7"]),
        ('{"commands":["0"]}', ["0"]),
        ("6,7", ["6", "7"]),
        ("comando 7", []),
    ]

    ok = True
    print("🧪 Self-test euristiche:")
    for text, expected in tests:
        got = normalize_command_sequence(heuristic_commands(text), text, "IDLE")
        passed = got == expected
        ok = ok and passed
        print(f"  {'✅' if passed else '❌'} {text!r} -> {got} | atteso {expected}")

    print("\n🧪 Self-test parser Gemini:")
    for raw, expected in parser_tests:
        got = parse_ai_commands(raw)
        passed = got == expected
        ok = ok and passed
        print(f"  {'✅' if passed else '❌'} {raw!r} -> {got} | atteso {expected}")

    print("\n✅ Self-test completato." if ok else "\n❌ Self-test fallito.")
    return 0 if ok else 1


def run_text_mode(text: str, no_serial: bool) -> int:
    ser = None if no_serial else connect_serial()

    print(f"🗣️ Frase test: {text}")
    commands = analyze_commands(None, text, robot_state)

    if no_serial:
        print(f"📋 Comandi calcolati: {commands if commands else '[nessun invio]'}")
    else:
        send_command_sequence(ser, commands)

    if ser is not None and ser.is_open:
        ser.close()
        print("🔌 Seriale chiusa.")

    return 0

def wait_enter(message: str = "➡️ Premi INVIO per parlare...") -> bool:
    try:
        print(message)
        input()
        return True
    except KeyboardInterrupt:
        return False


def run_voice_loop(no_serial: bool) -> int:
    whisper_model = load_whisper_model()
    ser = None if no_serial else connect_serial()

    print("Comandi principali:")
    print("  1=felice  2=triste  3=bacio/cuore  4=patrol  5=saluto  6=stop  7=pick&place")
    print("Esempi: 'mandami un bacio', 'manda un cuore', 'fermati e fai la pattuglia'.\n")

    if not wait_enter("➡️ Premi INVIO per iniziare..."):
        return 0

    try:
        while True:
            audio = record_audio()
            text = transcribe(whisper_model, audio)

            if not text:
                print("⚠️ Nessuna frase riconosciuta: non invio nulla.\n")
            else:
                print(f"🗣️ Hai detto: {text}")
                commands = analyze_commands(None, text, robot_state)
                send_command_sequence(ser, commands)

            if not wait_enter("➡️ Premi INVIO per parlare di nuovo..."):
                break

    except KeyboardInterrupt:
        print("\n👋 Uscita dal programma.")
    finally:
        if ser is not None and ser.is_open:
            ser.close()
            print("🔌 Seriale chiusa.")

    return 0


def main() -> int:
    global SERIAL_PORT

    args = parse_args()
    SERIAL_PORT = args.serial_port

    if args.self_test:
        return run_self_test()

    if args.text is not None:
        return run_text_mode(args.text, args.no_serial)

    return run_voice_loop(args.no_serial)


if __name__ == "__main__":
    raise SystemExit(main())