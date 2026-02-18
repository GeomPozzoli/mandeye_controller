# Trigger camera via PPS — guida cablaggio e configurazione

## Il problema: cosa significa davvero "trigger" per libcamera

Il sensore della Pi-Camera gira sempre in **free-running**: il pixel array scatta frame
continuamente al proprio ritmo interno. Un approccio del tipo *"rilevo il GPIO, chiamo
queueRequest"* non controlla il momento dell'esposizione — l'esposizione è già avvenuta.

Esistono due soluzioni genuine, implementate come modalità selezionabili via JSON config.

---

## Modalità 1 — NEAREST_FRAME (qualsiasi Pi-Camera)

La camera gira in free-running a frequenza fissa. Un thread dedicato (`ppsWatchThread`)
ascolta un pin GPIO per i fronti di salita del PPS. Quando arriva un edge, registra
il timestamp UTC del fronte. `requestComplete()` confronta poi il timestamp hardware
di ogni frame con l'ultimo PPS: se la differenza è inferiore a `maxFrameAgeMs`,
quel frame viene salvato e **taggato con il timestamp del PPS** — non con quello del
sensore. Un frame per ogni PPS, tutti gli altri vengono scartati silenziosamente.

```
PPS edge (T=0)
     │
     │   frame 1 scartato (delta > maxFrameAge)
     │   frame 2 scartato
     │   frame 3 accettato → salvato con timestamp = T=0
     │   frame 4 scartato
     │
PPS edge (T=1s)
     │
     │   frame N accettato → salvato con timestamp = T=1s
```

**Precisione:** ±(periodo_frame / 2). A 10 fps → ±50 ms. A 30 fps → ±17 ms.

Per la colorizzazione di nuvole di punti LiDAR a velocità di camminata (~1 m/s),
±50 ms produce uno spostamento di ~5 cm — nella maggior parte dei casi accettabile.

**Cablaggio NEAREST_FRAME:**

```
RPi GPIO 17 (PPS verso LiDAR) ──┬──► LiDAR PPS IN
                                 │
                                 └──► RPi GPIO 22 (trigger camera, input)
                                       │
                              semplice biforcazione del filo
```

Entrambi i pin devono condividere la stessa GND. Non serve nessun componente aggiuntivo
perché il segnale rimane interno al Pi (3.3V → 3.3V).

**Configurazione (cam0_config.json):**
```json
{
  "trigger": {
    "mode": "NEAREST_FRAME",
    "gpioPin": 22,
    "gpioChip": "/dev/gpiochip0",
    "maxFrameAgeMs": 60
  },
  "picamera": {
    "FrameDurationLimits": 100000
  }
}
```

`FrameDurationLimits = 100000 µs` forza la camera a girare a 10 fps
(un frame ogni 100 ms), quindi `maxFrameAgeMs = 60` è un margine comodo.

---

## Modalità 2 — XVS_HARD (solo IMX477 HQ Camera / IMX296 Global Shutter)

Il sensore IMX477 ha un pin fisico chiamato **XVS** (Vertical Sync). Quando è in
modalità slave (abilitata via libcamera FrameSync draft control, impostato
automaticamente da `start()`), il sensore aspetta un fronte su XVS prima di
iniziare ogni frame. Il segnale PPS viene collegato direttamente al pin XVS.

**Precisione:** < 1 µs (dominio del clock del pixel del sensore).

### Prerequisiti software

Nel file `/boot/firmware/config.txt` (o `/boot/config.txt` su sistemi più vecchi):

```
# Disabilita il rilevamento automatico e carica il driver IMX477 in modalità sync
camera_auto_detect=0
dtoverlay=imx477,sync
```

Riavviare dopo la modifica.

### Cablaggio XVS_HARD

Il pin XVS si trova sul connettore FPC a 22 pin della HQ Camera (quello che va al Pi).
Sulla maggior parte delle carrier board è marcato "SYNC" o "XVS".

```
IMX477 HQ Camera (FPC 22 pin)          Raspberry Pi 4
────────────────────────────────────    ─────────────────────
Pin 11 — XVS / SYNC (1.8V logic) ◄──── Divisore di tensione ◄── GPIO 17 (3.3V, PPS)
Pin 1  — GND ───────────────────────── GND (pin 6 o 9)

Divisore resistivo (3.3V → 1.8V):
  GPIO 17 ──[ 3.3 kΩ ]──┬── XVS pin
                         │
                      [ 1.8 kΩ ]
                         │
                        GND
```

**ATTENZIONE:** Il livello logico del pin XVS sull'IMX477 è 1.8V. Il GPIO del Pi
è 3.3V. Superare 1.8V sul pin XVS può danneggiare il sensore. Il divisore resistivo
è **obbligatorio**. In alternativa si può usare un level shifter bidirezionale TXS0101
o equivalente.

**Configurazione (cam0_config.json):**
```json
{
  "trigger": {
    "mode": "XVS_HARD",
    "gpioPin": 17,
    "gpioChip": "/dev/gpiochip0"
  }
}
```

In XVS_HARD il `gpioPin` serve solo al `ppsWatchThread` per registrare il timestamp
del PPS nel file `.meta.json`. Il timing reale è controllato dal wire fisico XVS.

---

## Struttura dei file salvati

In entrambe le modalità, i file prodotti sono gli stessi dell'originale:

```
/media/usb/session_001/
  CAMERA_0/
    cam0_1708123456000000000.jpg        ← timestamp = PPS edge UTC (ns)
    cam0_1708123456000000000.meta.json  ← include diagnostica di trigger
    cam0_1708123457000000000.jpg
    cam0_1708123457000000000.meta.json
  lidar0000.laz
  imu0000.csv
```

Il file `.meta.json` ora include campi aggiuntivi:
```json
{
  "_triggerMode": 1,
  "_frameUtcNs": 1708123456042000000,
  "_reportTimestampNs": 1708123456000000000,
  "_ppsUtcNs": 1708123456000000000,
  "ExposureTime": "20000",
  "AnalogueGain": "2.0",
  ...
}
```

Il campo `_reportTimestampNs` corrisponde al timestamp del PPS edge (secondi interi UTC),
che è lo stesso timestamp usato dal LiDAR per i propri chunk. Questo rende banale
l'associazione foto↔nuvola di punti in post-processing.

`_frameUtcNs` è il timestamp hardware del sensore, utile per calcolare il delta
effettivo: `delta = _frameUtcNs - _ppsUtcNs`.

---

## Riepilogo cablaggio per RPi4 standard con PPS

Assumendo il file `mandeye-standard-rpi4-pps.h` (discusso in precedenza):

```
RPi4 header 40-pin
─────────────────────────────────────────────────────────────────
GPIO 17 (pin 11) ──┬──► LiDAR Mid-360 PPS IN
                   └──► GPIO 22 (pin 15)  [NEAREST_FRAME trigger]
                         oppure
                   └──► Divisore 3.3k/1.8k ──► Camera XVS [XVS_HARD]

GPIO  0 (pin 27) ──►  LiDAR Mid-360 UART RX (NMEA @ 9600)
GPIO 14 (pin  8) ──►  GNSS RX  (opzionale)
GPIO 15 (pin 10) ◄──  GNSS TX  (NMEA @ 38400)
GND     (pin  6) ──── GND comune (LiDAR, GNSS, Camera)
```

---

## Confronto tra le modalità

| Modalità       | Modelli supportati    | Precisione     | Hardware aggiuntivo     | Complessità |
|----------------|-----------------------|----------------|-------------------------|-------------|
| INTERNAL       | qualsiasi             | nessuna (random) | —                     | nulla       |
| NEAREST_FRAME  | qualsiasi             | ±(T_frame / 2) | 1 filo (fork del PPS)   | bassa       |
| XVS_HARD       | IMX477, IMX296        | < 1 µs         | Divisore 3.3k/1.8k      | media       |

Per uno scanner LiDAR portatile a uso di rilievo topografico o di edifici,
**NEAREST_FRAME a 10-20 fps** è la scelta più pratica: zero componenti aggiuntivi,
funziona con qualsiasi modello di Pi-Camera incluse le v2 e v3, e la precisione
di ±25-50 ms è sufficiente per associare correttamente le foto alla nuvola di punti.

XVS_HARD è riservato a applicazioni che richiedono sincronizzazione sub-millisecondo,
come la calibrazione di sistemi multi-camera o la fusione LiDAR-camera ad alta velocità.
