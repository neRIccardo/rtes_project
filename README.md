# 📌 Progetto FreeRTOS – Sistema di monitoraggio su Raspberry Pi Pico

Questo progetto realizza un **sistema di monitoraggio simulato** su **Raspberry Pi Pico** utilizzando **FreeRTOS**.  
L’applicazione legge la temperatura interna del microcontrollore, la elabora tramite più task concorrenti e ne gestisce la visualizzazione su interfaccia seriale.  

Non utilizza hardware aggiuntivo oltre al Pico stesso.

---

## ✨ Funzionalità principali

✅ Lettura periodica della temperatura interna tramite ADC del RP2040.  
✅ Gestione concorrente con **FreeRTOS** tramite:
- **Task di acquisizione**: legge la temperatura a intervalli configurabili
- **Task di elaborazione**: stampa la temperatura e lampeggia il LED se supera una soglia
- **Task menu**: riceve comandi via seriale per:
  - Avviare/sospendere la rilevazione
  - Modificare soglia e intervallo
  - Visualizzare media delle ultime letture
  - Visualizzare la soglia corrente e l’intervallo corrente
- **Task statistiche**: ogni 10 s mostra min, max e deviazione standard delle ultime letture
- **Task diagnostico**: ogni 10 s mostra memoria heap libera e stack residuo dei task

✅ Utilizzo di **mutex e code FreeRTOS** per evitare race condition e garantire thread safety.  
✅ Possibilità di modificare la soglia e l’intervallo in tempo reale senza interrompere il sistema.<br>
✅ **Script Python logger** che:
- Legge la seriale
- Salva i dati su CSV (`tempo_ms, temperatura_C`)
- Permette di inviare comandi come dal menu
- Alla chiusura (CTRL+C) genera un grafico PNG con:
  - Puntini dei valori misurati
  - Linea di media mobile per visualizzare la tendenza

***Nota script python***
- Lo script è opzionale nel caso in cui non si è interessati al logging/grafico: il sistema funzionerà comunque in modo corretto. E' quindi necessario ignorare le sezioni terminali di questo documento ("***Utilizzo script Python logger***"). 
---

## ⚡ Requisiti

- Raspberry Pi Pico
- Toolchain ARM e Pico SDK configurati
- FreeRTOS Kernel incluso nel progetto
- Un sistema Linux (testato su Ubuntu 22.04 con WSL)
- Python (per lo script logger): opzionale nel caso in cui non si è interessati al logging/grafico

---

## 🔧 Preparazione ambiente

### 0. Configurare i gruppi utente
```bash
sudo usermod -a -G plugdev $USER
sudo usermod -a -G dialout $USER

# Per montare PICO senza "sudo"
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="2e8a", ATTR{idProduct}=="0003", MODE="0660", GROUP="plugdev"' | sudo tee /etc/udev/rules.d/99-rpi-pico.rules > /dev/null

# Riavvia dopo queste modifiche
```
### 1. Installare Pico SDK
```bash
sudo apt install cmake g++ gcc-arm-none-eabi doxygen libnewlib-arm-none-eabi git python3
git clone --recurse-submodules https://github.com/raspberrypi/pico-sdk.git $HOME/pico-sdk

# Impostare variabile d'ambiente
echo "export PICO_SDK_PATH=$HOME/pico-sdk" >> ~/.bashrc
source ~/.bashrc
```
### 2. Clonare questo progetto
```bash
git clone https://github.com/neRIccardo/rtes_project.git
cd rtes_project
```
---
## ▶️ Compilazione e caricamento
Creare la cartella "build" e compilare:
```bash
mkdir build
cd build
cmake ..
make
```
Per caricare il firmware:
- Tenere premuto il tasto "BOOTSEL" sul Pico
- Collegarlo alla USB: apparirà come disco "RPI-RP2"
- Copiare il file ".uf2" generato:
```bash
cp apps/exam_project.uf2 $(findmnt -rn -o TARGET -S LABEL=RPI-RP2)/
```

***Nota per chi usa WSL***
- Su Windows Subsystem for Linux il comando "findmnt" potrebbe non funzionare e il disco "RPI-RP2" non viene montato automaticamente in WSL.
- In questo caso, dopo aver generato il file ".uf2" nella cartella "build/apps/", copia manualmente il file in "RPI-RP2" tramite l’esplora risorse di Windows: basta aprire la cartella "build/apps", trovare "exam_project.uf2" e trascinarlo sul disco "RPI-RP2"

---

## 📌 Utilizzo
Apri un terminale seriale (es. PuTTY) a ***115200 baud***. <br>
All’avvio il menu mostrerà i comandi disponibili:
```bash
[MENU] Comandi disponibili:
  r = avvia rilevazione (reset buffer)
  t = sospendi rilevazione
  p = mostra soglia attuale
  s<val> = imposta nuova soglia (es: s30)
  m = media ultime 10 letture
  i<ms> = imposta intervallo acquisizione (es: i1000)
  g = mostra intervallo acquisizione attuale
  ```
Mentre la rilevazione è attiva, ogni 10 s verranno stampate:
- [STATS] con ***min, max e deviazione standard*** delle ultime letture
- [DIAG] con ***memoria heap libera e stack residuo*** dei task
---
## 📄 Utilizzo script Python logger
Nella cartella "logger/" trovi "log_temp.py" e un "requirements.txt":
### 0. Installazione dipendenze Python
```bash
cd logger/
python3 -m pip install -r requirements.txt
```
### 1. Avvio logger
Chiudi qualsiasi programma stia usando la porta COM (es. PuTTY), poi esegui:
```bash
python3 log_temp.py
```
✅ Funzionalità:
- Mostra in tempo reale i messaggi generati dai task tramite seriale
- Puoi digitare i comandi del menu direttamente in questa console
- Salva i dati in "log.csv"
- Quando premi CTRL+C ferma la lettura e genera automaticamente "grafico_temperatura.png" con i dati letti
---
## 📊 Grafico generato
Il grafico salvato (grafico_temperatura.png) mostra:
- ***Linea blu***: valori di temperatura letti
- ***Linea rossa***: media mobile (finestra 10 letture) per evidenziare la tendenza
---
## ℹ️ Note implementative
- Usa ***mutex*** per proteggere accessi concorrenti alle variabili condivise

- Usa ***code FreeRTOS*** per passare le temperature dal task di acquisizione al task di elaborazione

- ***Deviazione standard***: misura quanto le letture si discostano dalla media; valori bassi indicano letture stabili, valori alti indicano maggiore variabilità
- Lo script Python usa una ***coda*** per la stampa asincrona, così la velocità della console non rallenta la lettura seriale
---
## 📖 Documentazione utile
- [Raspberry Pi Pico C/C++ SDK](https://datasheets.raspberrypi.org/pico/raspberry-pi-pico-c-sdk.pdf)
- [FreeRTOS API Reference](https://www.freertos.org/a00106.html)
---
## 📝 Licenza e utilizzo
Questo progetto è realizzato a scopo didattico per l’esame di Real-Time and Embedded Systems.
Puoi usarlo e modificarlo liberamente per fini educativi.