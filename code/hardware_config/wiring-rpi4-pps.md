# Sincronizzazione PPS su Raspberry Pi 4 Standard

Questa guida descrive come aggiungere la sincronizzazione hardware PPS (Pulse Per Second)
alla versione standard del Mandeye basata su Raspberry Pi 4, normalmente priva di questo
meccanismo. Il risultato è funzionalmente equivalente alla versione Pro CM4, con supporto
per **un LiDAR Livox Mid-360**.

---

## Principio di funzionamento

Il servizio `fake_pps` genera ogni secondo esatto:
1. Un **impulso elettrico** su un pin GPIO → ingresso PPS del Mid-360
2. Una **stringa NMEA `$GPRMC`** su una porta UART → ingresso TIMESYNC del Mid-360

Il LiDAR usa questi due segnali per agganciare il suo orologio interno all'UTC
del sistema operativo del Pi (il quale a sua volta si sincronizza via NTP).

---

## Modifiche software necessarie

### 1. Abilitare le UART aggiuntive — `/boot/config.txt`

```
# Abilita UART2 su GPIO 0 (TX) e GPIO 1 (RX) per il segnale NMEA verso il LiDAR
dtoverlay=uart2

# Disabilita il Bluetooth per liberare la UART PL011 (/dev/ttyAMA0) per il GNSS
# ATTENZIONE: questo disabilita il Bluetooth sul Pi
dtoverlay=disable-bt
```

Dopo la modifica, riavviare e disabilitare il servizio Bluetooth:

```bash
sudo systemctl disable hciuart
sudo reboot
```

Verifica che le porte siano disponibili:

```bash
ls /dev/ttyAMA*
# Atteso: /dev/ttyAMA0  /dev/ttyAMA1
```

### 2. Compilare con il nuovo header hardware

```bash
cd mandeye_controller
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DMANDEYE_HARDWARE_HEADER=mandeye-standard-rpi4-pps.h
make -j2
sudo make install
```

### 3. Abilitare il servizio fake_pps

```bash
sudo systemctl enable mandeye_fake_pps.service
sudo systemctl start mandeye_fake_pps.service
```

### 4. (Opzionale) Attendere la sincronizzazione prima di scansionare

Nel file `mandeye-standard-rpi4-pps.h`, impostare:

```cpp
constexpr bool WaitForLidarSync = true;
```

Questo fa sì che il sistema aspetti fino a 60 secondi che il LiDAR confermi
la sincronizzazione (`kKeyTimeSyncType > 0`) prima di passare allo stato IDLE.
Consigliato una volta che il cablaggio è verificato e funzionante.

---

## Schema di cablaggio

### Header 40-pin del Raspberry Pi 4

```
 3V3  (1) (2)  5V
GPIO2 (3) (4)  5V
GPIO3 (5) (6)  GND  ──────────────────────────────── GND comune
GPIO4 (7) (8)  GPIO14 [UART0 TX → GNSS RX]
  GND (9) (10) GPIO15 [UART0 RX ← GNSS TX]
GPIO17(11) (12) GPIO18   ← PPS OUT verso LiDAR
GPIO27(13) (14) GND
GPIO22(15) (16) GPIO23
 3V3 (17) (18) GPIO24
GPIO10(19) (20) GND
GPIO9 (21) (22) GPIO25
GPIO11(23) (24) GPIO8
  GND(25) (26) GPIO7
GPIO0 (27) (28) GPIO1  ← UART2 TX (pin 27) → LiDAR NMEA IN
GPIO5 (29) (30) GND
GPIO6 (31) (32) GPIO12 [BUZZER]
GPIO13(33) (34) GND
GPIO19(35) (36) GPIO16
GPIO26(37) (38) GPIO20
  GND(39) (40) GPIO21
```

> I numeri tra parentesi sono i numeri fisici dei pin dell'header (1-40).
> I numeri `GPIOxx` sono i numeri BCM usati nel codice.

---

### Connessioni da realizzare

#### LiDAR Livox Mid-360 → Raspberry Pi 4

Il Mid-360 ha un connettore di sincronizzazione (solitamente M12 o JST,
verificare il datasheet del proprio modello). I segnali rilevanti sono:

| Segnale Mid-360 | Pin RPi4        | GPIO BCM | Note                        |
|-----------------|-----------------|----------|-----------------------------|
| PPS IN          | Pin 11          | GPIO 17  | Impulso 0→1 ogni secondo    |
| UART RX (NMEA)  | Pin 27          | GPIO 0   | UART2 TX @ 9600 baud        |
| GND             | Pin 9 o 25      | —        | Massa comune obbligatoria   |

> **Livelli di tensione:** il Mid-360 lavora a 3.3V sui pin di sincronizzazione.
> Il Pi 4 produce 3.3V sui GPIO — compatibile diretto, nessun level shifter necessario.

#### GNSS receiver → Raspberry Pi 4

Esempio con ricevitore u-blox (M8N, M9N, ZED-F9P, ecc.):

| Segnale GNSS | Pin RPi4   | GPIO BCM | Note                         |
|--------------|------------|----------|------------------------------|
| TX (NMEA out)| Pin 10     | GPIO 15  | UART0 RX → Pi riceve NMEA    |
| RX (opz.)    | Pin  8     | GPIO 14  | UART0 TX → comandi al GNSS   |
| GND          | Pin  6     | —        | Massa comune                 |
| VCC          | Pin 1 (3V3)| —        | Solo se il modulo è 3.3V!    |

> Molti moduli u-blox accettano sia 3.3V che 5V — verificare il datasheet.
> Se il modulo è 5V, usare un level shifter sul pin TX del GNSS verso il Pi.

---

## Schema elettrico riassuntivo

```
                    ┌─────────────────────┐
                    │   Raspberry Pi 4    │
                    │                     │
          ┌─────────┤ GPIO 17 (pin 11)    │
          │  PPS    │                     │
          │         │ GPIO  0 (pin 27) TX ├──────────┐  NMEA
          │         │                     │          │
          │         │ GPIO 15 (pin 10) RX ◄──────┐  │
          │         │ GPIO 14 (pin  8) TX ├───┐  │  │
          │         │                     │   │  │  │
          │         │ GND     (pin  6)    ├─┐ │  │  │
          │         └─────────────────────┘ │ │  │  │
          │                                 │ │  │  │
          │   ┌─────────────────────────┐   │ │  │  │
          │   │   Livox Mid-360         │   │ │  │  │
          └──►│ PPS IN                  │   │ │  │  │
              │                         │   │ │  │  │
              │◄────────────────────────┼───┼─┼──┘  │  NMEA @ 9600
              │ UART RX (TIMESYNC)      │   │ │     │
              │                         │   │ │     │
              │ GND ────────────────────┼───┘ │     │
              └─────────────────────────┘     │     │
                                              │     │
          ┌─────────────────────────┐         │     │
          │   GNSS Receiver         │         │     │
          │ TX (NMEA) ──────────────┼─────────┘     │
          │ RX (opz.) ◄─────────────┼───────────────┘
          │ GND ────────────────────┼── GND comune
          │ VCC ────────────────────┼── 3.3V Pi (se compatibile)
          └─────────────────────────┘
```

---

## Verifica del funzionamento

### 1. Controllare che il servizio fake_pps sia attivo

```bash
sudo systemctl status mandeye_fake_pps.service
```

### 2. Controllare la porta UART verso il LiDAR

```bash
# Il Mid-360 deve ricevere qualcosa su /dev/ttyAMA1
# Si può verificare loopback (TX→RX fisicamente cortocircuitati):
cat /dev/ttyAMA1 &
echo "test" > /dev/ttyAMA1
```

### 3. Monitorare lo stato di sync tramite API REST

```bash
curl http://192.168.1.5:8003/status | python3 -m json.tool | grep -A5 timesync
```

Il campo `timesyncmode` deve mostrare un valore `> 0` dopo alcuni secondi
dall'avvio del fake_pps. Il valore tipico per PPS è `1`.

### 4. Controllare il GNSS

```bash
# Leggere NMEA grezze dalla porta GNSS
cat /dev/ttyAMA0
# Devono apparire stringhe tipo: $GPGGA,143022.00,...
```

---

## Differenze rispetto alla versione Pro CM4

| Caratteristica            | Standard RPi4 (originale) | Standard RPi4 + PPS (questo file) | Pro CM4         |
|---------------------------|---------------------------|-------------------------------------|-----------------|
| Sincronizzazione PPS      | ✗ (solo NTP del Pi)       | ✓ GPIO 17 + UART2                   | ✓ GPIO 3 + UART |
| N. LiDAR sincronizzabili  | 0                         | 1                                   | 2               |
| Precisione sync           | ~ms (NTP)                 | ~100-500 µs (fake PPS su Linux)     | ~100-500 µs     |
| GNSS integrato            | ✓ /dev/ttyS0              | ✓ /dev/ttyAMA0                      | ✓ /dev/ttyAMA3  |
| Bluetooth disponibile     | ✓                         | ✗ (disabilitato per UART)           | ✗               |
| Hardware aggiuntivo       | —                         | Solo fili                           | Carrier board   |

---

## Troubleshooting

**Il campo `timesyncmode` rimane 0**
- Verificare che il filo da GPIO 17 al PPS IN del Mid-360 sia collegato correttamente
- Verificare che GND sia comune tra Pi e Mid-360
- Controllare con un oscilloscopio o multimetro che GPIO 17 produca impulsi ogni secondo

**`/dev/ttyAMA1` non esiste dopo il riavvio**
- Verificare che `dtoverlay=uart2` sia nella sezione corretta di `/boot/config.txt`
- Eseguire `dmesg | grep ttyAMA` per vedere quali UART il kernel ha registrato

**Il GNSS non viene letto (`/dev/ttyAMA0` vuoto)**
- Verificare che `dtoverlay=disable-bt` e `sudo systemctl disable hciuart` siano stati eseguiti
- Alternativa senza disabilitare BT: usare `/dev/ttyS0` a 9600 baud (mini-UART, meno preciso)
  e cambiare `GetGNSSPort()` e `GetGNSSBaudrate()` nell'header

**Bluetooth necessario**
- Se il Bluetooth è indispensabile, è possibile usare un ricevitore GNSS USB
  (appare come `/dev/ttyUSB0` o `/dev/ttyACM0`) e aggiornare `GetGNSSPort()` di conseguenza
- In questo scenario `/dev/ttyAMA1` rimane disponibile per il LiDAR senza toccare BT
