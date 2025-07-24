import serial
import re
import threading
import sys
import queue
import time

# Configurazione porta seriale
PORTA = "COM5"       
BAUD = 115200
FILE_LOG = "log.csv"

# Pattern per estrarre dati di temperatura
pattern = re.compile(r"\[TEMP\]\s+(\d+)\s+ms:\s+([\d\.]+)")

# Apro porta seriale con timeout basso per non bloccare troppo la lettura
ser = serial.Serial(PORTA, BAUD, timeout=0.1)

# Apro file CSV per logging
f = open(FILE_LOG, "w")
f.write("tempo_ms,temperatura_C\n")
f.flush()

print(f"[INFO] Logger avviato su {PORTA} (baud {BAUD}).")
print("[INFO] Premi CTRL+C per interrompere.")
print("[INFO] Puoi digitare comandi (es: r, t, m...) e premere Invio.")

# Coda thread-safe per gestire la stampa
print_queue = queue.Queue()

# Thread per leggere dati dalla seriale
def reader():
    while True:
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line:
                print_queue.put(line)  # metto la riga in coda per la stampa
                m = pattern.match(line)
                if m:
                    tempo_ms = m.group(1)
                    temperatura = m.group(2)
                    f.write(f"{tempo_ms},{temperatura}\n")
                    f.flush()
        except Exception as e:
            print_queue.put(f"[ERRORE lettura] {e}")
            break

# Thread dedicato alla stampa dalla coda
def printer():
    while True:
        try:
            line = print_queue.get()
            print(line)
            print_queue.task_done()
        except Exception as e:
            print(f"[ERRORE stampa] {e}")

# Thread per leggere comandi da tastiera e inviarli alla seriale
def writer():
    while True:
        try:
            cmd = sys.stdin.readline()
            if cmd:
                ser.write(cmd.encode("utf-8"))
        except Exception as e:
            print_queue.put(f"[ERRORE scrittura] {e}")
            break

# Avvio thread
t_reader = threading.Thread(target=reader, daemon=True)
t_writer = threading.Thread(target=writer, daemon=True)
t_printer = threading.Thread(target=printer, daemon=True)

t_reader.start()
t_writer.start()
t_printer.start()

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\n[INFO] Interruzione richiesta. Chiudo...")
    ser.close()
    f.close()
