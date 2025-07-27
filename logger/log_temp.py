import serial
import re
import threading
import sys
import queue
import time
import pandas as pd
import matplotlib.pyplot as plt

# =========================================================
# CONFIGURAZIONE
# =========================================================

# Porta seriale del tuo Pico
PORTA = "COM5"         # esempio su Windows; controlla la tua COM
# PORTA = "/dev/ttyACM0"  # esempio su Linux

BAUD = 115200          # baud rate uguale a quello impostato nel firmware
FILE_LOG = "log.csv"   # nome file CSV dove salvare i dati

# =========================================================
# ESPRESSIONE REGOLARE PER DATI TEMPERATURA
# =========================================================
# Le righe inviate dal Pico sono tipo: [TEMP] 1234 ms: 30.15°C
pattern = re.compile(r"\[TEMP\]\s+(\d+)\s+ms:\s+([\d\.]+)")

# =========================================================
# VARIABILI E STRUTTURE PER THREAD
# =========================================================

# Evento usato per segnalare ai thread di fermarsi
stop_event = threading.Event()

# Coda thread-safe per le righe da stampare
print_queue = queue.Queue()

# =========================================================
# INIZIALIZZAZIONE SERIAL E FILE CSV
# =========================================================
# Timeout basso per evitare blocchi lunghi durante la lettura
ser = serial.Serial(PORTA, BAUD, timeout=0.1)

# Apri file CSV e scrivi intestazione
f = open(FILE_LOG, "w")
f.write("tempo_ms,temperatura_C\n")
f.flush()

print(f"[INFO] Logger avviato su {PORTA} (baud {BAUD}).")
print("[INFO] Premi CTRL+C per interrompere.")
print("[INFO] Puoi digitare comandi (es: r, t, m...) e premere Invio.")

# =========================================================
# THREAD: LETTURA DALLA SERIALE
# =========================================================
def reader():
    """
    Legge continuamente dalla seriale:
      - Ogni riga letta viene messa nella coda di stampa.
      - Se è una riga [TEMP], estrae tempo e temperatura e li scrive nel CSV.
    """
    while not stop_event.is_set():
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line:
                # Mettiamo la riga in coda per la stampa asincrona
                print_queue.put(line)

                # Controlliamo se la riga contiene un dato temperatura
                m = pattern.match(line)
                if m:
                    tempo_ms = m.group(1)
                    temperatura = m.group(2)
                    f.write(f"{tempo_ms},{temperatura}\n")
                    f.flush()
        except Exception as e:
            # Se stop_event è già settato, ignoriamo errori di chiusura
            if not stop_event.is_set():
                print_queue.put(f"[ERRORE lettura] {e}")
            break

# =========================================================
# THREAD: STAMPA ASINCRONA
# =========================================================
def printer():
    """
    Prende righe dalla coda e le stampa.
    Così la lettura seriale non si blocca mai aspettando la console.
    """
    while not stop_event.is_set():
        try:
            # Usa timeout per uscire dal loop in caso di stop_event
            line = print_queue.get(timeout=0.1)
            print(line)
            print_queue.task_done()
        except queue.Empty:
            # Nessuna riga pronta, continua il loop
            continue
        except Exception as e:
            print(f"[ERRORE stampa] {e}")

# =========================================================
# THREAD: SCRITTURA COMANDI VERSO LA SERIALE
# =========================================================
def writer():
    """
    Legge comandi da tastiera e li invia al Pico.
    Esempi di comandi supportati dal firmware:
      r <Invio> → avvia rilevazione
      t <Invio> → ferma rilevazione
      m <Invio> → media ultime letture
    """
    while not stop_event.is_set():
        try:
            cmd = sys.stdin.readline() # legge una riga di input
            if cmd:
                ser.write(cmd.encode("utf-8")) # invia il comando al Pico
        except Exception as e:
            if not stop_event.is_set():
                print_queue.put(f"[ERRORE scrittura] {e}")
            break

# =========================================================
# AVVIO THREAD
# =========================================================
t_reader = threading.Thread(target=reader, daemon=True) # Daemon per chiudere automaticamente alla fine del programma
t_writer = threading.Thread(target=writer, daemon=True)
t_printer = threading.Thread(target=printer, daemon=True)

t_reader.start()
t_writer.start()
t_printer.start()

# =========================================================
# LOOP PRINCIPALE
# =========================================================
try:
    # Resta attivo finché non premi CTRL+C
    while True:
        time.sleep(1)

except KeyboardInterrupt:
    # Quando premi CTRL+C, segnaliamo lo stop a tutti i thread
    print("\n[INFO] Interruzione richiesta. Sto chiudendo i thread e la seriale...")
    stop_event.set()

    # Aspettiamo la terminazione dei thread con un timeout
    t_reader.join(timeout=1)
    t_writer.join(timeout=1)
    t_printer.join(timeout=1)

    # Chiudiamo la seriale e il file CSV
    ser.close()
    f.close()

    # =====================================================
    # GENERAZIONE GRAFICO AUTOMATICO
    # =====================================================
    print("[INFO] Generazione grafico dal CSV...")
    try:
        # Carichiamo i dati dal CSV
        df = pd.read_csv(FILE_LOG)

        # Creiamo la figura
        plt.figure(figsize=(10, 5))
        # Plot originale con punti piccoli e linea sottile
        plt.plot(
            df['tempo_ms'],
            df['temperatura_C'],
            marker='o',
            markersize=2,
            linestyle='-',
            linewidth=1,
            label='Temperatura'
        )

        # Calcoliamo media mobile (finestra di 10 letture)
        df['media_mobile'] = df['temperatura_C'].rolling(window=10, center=True).mean()
        plt.plot(
            df['tempo_ms'],
            df['media_mobile'],
            color='red',
            linewidth=2,
            label='Media mobile'
        )

        # Miglioriamo l’aspetto
        plt.title('Andamento temperatura nel tempo')
        plt.xlabel('Tempo (ms)')
        plt.ylabel('Temperatura (°C)')
        plt.grid(True)
        plt.legend()

        # Salviamo il grafico
        output_png = "grafico_temperatura.png"
        plt.savefig(output_png, dpi=150)
        print(f"[INFO] Grafico salvato come {output_png}")
    except Exception as e:
        print(f"[ERRORE] Non sono riuscito a generare il grafico: {e}")
