# Camera Trigger & PPS Synchronization — Setup Guide

> Branch: `feature/pps-trigger-camera`  
> Hardware: Raspberry Pi 4 · Livox Mid-360 · Innomaker GS Camera

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Hardware Wiring](#2-hardware-wiring)
3. [Raspberry Pi Configuration](#3-raspberry-pi-configuration)
4. [fake_pps — PPS and Trigger Signal Generation](#4-fake_pps--pps-and-trigger-signal-generation)
5. [Camera Acquisition Modes](#5-camera-acquisition-modes)
6. [JSON Configuration Reference](#6-json-configuration-reference)
7. [Build and Deployment](#7-build-and-deployment)
8. [Verification](#8-verification)
9. [Troubleshooting](#9-troubleshooting)
10. [Modified Files](#10-modified-files)

---

## 1. System Overview

The system generates a PPS (Pulse Per Second) signal via GPIO to synchronize:

- **LiDAR Livox Mid-360** — receives PPS on the M12 connector + NMEA string over serial
- **Camera** — receives a synchronization signal via GPIO, either as a software hint (NEAREST_FRAME) or as a true hardware trigger (HARDWARE_TRIGGER for Innomaker GS)

Timestamp alignment between photos and LiDAR point clouds is essential for point cloud colorization in HDMapping.

### Software Architecture

```
fake_pps (one thread per channel)
  ├── Master Thread   → broadcasts every exact second boundary + sends NMEA
  ├── Thread GPIO 17  → PPS_HIGH 1Hz → Livox Mid-360
  └── Thread GPIO 23  → TRIGGER_LOW 10Hz → Innomaker GS XTR

mandeye_libcamera
  ├── ppsWatchThread  → listens for GPIO edge → records UTC timestamp
  └── requestComplete → selects or accepts frames based on trigger mode
```

Each GPIO channel runs in a dedicated thread — no timing interference between channels.

### Supported Camera Configurations

| Camera | Sensor | Trigger Mode | Signal Type | Notes |
|--------|--------|-------------|-------------|-------|
| Any Pi Camera (IMX219, IMX477...) | Rolling or Global Shutter | NEAREST_FRAME | PPS_HIGH 1Hz (Option A) | 1 photo/sec, no extra wiring |
| Any Pi Camera (IMX219, IMX477...) | Rolling or Global Shutter | NEAREST_FRAME | PPS_HIGH up to 50Hz (Option B) | Multi fps, GPIO 23 → 22 fork only |
| Innomaker GS Camera | Global Shutter | HARDWARE_TRIGGER | TRIGGER_LOW falling edge | True hardware trigger, recommended |

---

## 2. Hardware Wiring

### LiDAR Livox Mid-360

| RPi Physical Pin | GPIO BCM | Direction | Mid-360 M12 Pin |
|---|---|---|---|
| Pin 11 | GPIO 17 | → | Pin 8 (PPS) |
| Pin 27 | GPIO 0  | → | Pin 10 (NMEA TX) |
| GND    | —       | → | GND |

### Generic Pi Camera — NEAREST_FRAME mode

Any Pi Camera (IMX219, IMX477, etc.) can be synchronized using the NEAREST_FRAME mode.
The camera runs free and the software selects the frame closest to the trigger edge.
**No physical trigger wire to the camera is needed.**

Two wiring options are available depending on the desired acquisition rate:

**Option A — 1 fps** (fork GPIO 17 PPS to GPIO 22):

| RPi Physical Pin | GPIO BCM | Direction | Destination |
|---|---|---|---|
| Pin 15 | GPIO 22 | ← input | Fork of GPIO 17 PPS signal |

```
RPi GPIO 17 (pin 11) ──┬──► Mid-360 M12 pin 8 (PPS)
                        └──► RPi GPIO 22 (pin 15) ──► ppsWatchThread
```

**Option B — up to 50 fps** (dedicated PPS_HIGH channel on GPIO 23, direct connection to GPIO 22):

| RPi Physical Pin | GPIO BCM | Direction | Destination |
|---|---|---|---|
| Pin 16 | GPIO 23 | → output | Direct wire to GPIO 22 (not connected to any other device) |
| Pin 15 | GPIO 22 | ← input | Direct wire from GPIO 23 |

```
RPi GPIO 23 (pin 16) ──► RPi GPIO 22 (pin 15) ──► ppsWatchThread
(point-to-point connection — GPIO 23 is not connected to any camera pin)
```

### Innomaker GS Camera — HARDWARE_TRIGGER mode

The Innomaker GS supports true hardware triggering via the XTR (Trig+) pin.
The sensor only produces a frame when it receives a low pulse on XTR.

| RPi Physical Pin | GPIO BCM | Direction | Destination |
|---|---|---|---|
| Pin 16 | GPIO 23 | → | XTR (Trig+) Innomaker GS |
| Pin 15 | GPIO 22 | ← input | Fork of GPIO 23 (ppsWatchThread) |
| GND    | —       | → | GND (Trig-) Innomaker GS |

Wiring diagram:

```
RPi GPIO 23 (pin 16) ──┬──► XTR Trig+ Innomaker GS
                        └──► RPi GPIO 22 (pin 15) ──► ppsWatchThread
RPi GND               ──► GND Trig- Innomaker GS
```

> **Innomaker GS note:** The Innomaker GS camera exposes XTR at 3.3V logic level —
> **no resistor divider is needed**. The standard RPi GPIO 3.3V output connects directly.

### Summary of all GPIO pins used

| Physical Pin | GPIO BCM | Function |
|---|---|---|
| Pin 11 | GPIO 17 | PPS output → LiDAR |
| Pin 15 | GPIO 22 | Trigger/PPS input → ppsWatchThread |
| Pin 16 | GPIO 23 | Trigger output → Innomaker GS XTR |
| Pin 27 | GPIO 0  | NMEA TX → LiDAR serial |
| GND    | —       | Common ground |

---

## 3. Raspberry Pi Configuration

### /boot/firmware/config.txt

For Innomaker GS Camera:
```ini
camera_auto_detect=0
dtoverlay=imx296
```

For generic Pi Camera — check your specific sensor:
```ini
camera_auto_detect=0
dtoverlay=imx219      # Raspberry Pi Camera Module 2
# dtoverlay=imx477    # Raspberry Pi HQ Camera
# dtoverlay=ov5647    # Raspberry Pi Camera Module v1
```

> `camera_auto_detect=1` may work for some officially supported sensors, but most cameras
> require `camera_auto_detect=0` and the explicit `dtoverlay` for their sensor.
> Check the sensor model printed on your camera board and refer to the
> [Raspberry Pi camera documentation](https://www.raspberrypi.com/documentation/accessories/camera.html)
> for the correct overlay name.

Verify the camera is detected:
```bash
rpicam-still --list-cameras
```

### Innomaker GS Kernel Trigger Mode

The Innomaker GS kernel module (imx296) must be configured for external trigger mode.
Without this setting the sensor ignores the XTR pin entirely and runs free-running,
making the hardware trigger ineffective.

Create the module configuration file (persists across reboots):
```bash
sudo nano /etc/modprobe.d/imx296.conf
```

Content:
```
options imx296 trigger_mode=1
```

Verify after reboot:
```bash
cat /sys/module/imx296/parameters/trigger_mode
# expected output: 1
```

> **Note:** If the imx296 kernel module is not loaded (camera not connected), the path
> `/sys/module/imx296/parameters/` does not exist — no error occurs and the system
> works normally without a camera.

> **This step is only required for HARDWARE_TRIGGER mode with the Innomaker GS camera.**
> For NEAREST_FRAME with any camera, no kernel configuration is needed.

### System Clock Synchronization

The Pi 4 has no hardware RTC. Ensure NTP is active:

```bash
sudo systemctl enable systemd-timesyncd
sudo systemctl start systemd-timesyncd
timedatectl set-ntp true
timedatectl status
# should show: NTP service: active  /  System clock synchronized: yes
```

---

## 4. fake_pps — PPS and Trigger Signal Generation

### Multi-Thread Architecture

`fake_pps` uses one dedicated thread per GPIO channel, all synchronized by a
**master thread** that wakes up at each exact second boundary and broadcasts via `condition_variable`.

```
Master Thread
  - sleep_until(next exact second boundary)
  - update g_tickMs (T=0 timestamp)
  - notify_all() → wake all channel threads simultaneously
  - send NMEA string over serial port

Channel Thread (one per channel)
  - wait for notify_all()
  - fire first pulse immediately at T=0
  - fire remaining pulses with sleep_until(t0 + n*periodMs)  ← absolute time, no drift
  - wait for next second
```

Advantages over the previous single-thread architecture:
- No timing interference between channels
- The PPS_HIGH sleep (100 ms) does not block the TRIGGER_LOW thread
- Precise and deterministic per-channel timing
- Scales to N arbitrary channels

### Channel Modes

**PPS_HIGH**
- Pin idle=LOW, pulse=HIGH for `pulseMs` ms at T=0 every second
- Rising edge = PPS reference
- Used for: LiDAR Livox Mid-360, camera in NEAREST_FRAME mode
- `freqHz` must be 1

**TRIGGER_LOW**
- Pin idle=HIGH, pulse=LOW for `pulseMs` ms at each trigger event
- Falling edge = start of Innomaker GS exposure
- `freqHz` controls how many triggers per second (1..50)
- First trigger of each second is aligned to T=0 (PPS boundary)
- Used for: Innomaker GS XTR hardware trigger

### Supported Frequencies

`freqHz` must divide 1000 ms evenly: **1, 2, 4, 5, 8, 10, 20, 25, 40, 50**

### Innomaker GS Exposure Calculation

```
Real exposure = pulseMs + 14.26 µs  (Innomaker GS sensor characteristic)
```

| pulseMs | Real Exposure | Motion Blur at 1.5 m/s | Recommended Use |
|---------|--------------|------------------------|-----------------|
| 5       | ~5.014 ms    | ~7.5 mm                | Cycling |
| 10      | ~10.014 ms   | ~15 mm                 | Walking (default) |
| 20      | ~20.014 ms   | ~30 mm                 | Low light / indoors |

The Innomaker GS uses a global shutter sensor — no rolling shutter distortion regardless of exposure time.

---

## 5. Camera Acquisition Modes

`LibCameraWrapper` supports three modes configurable via `cam0_config.json`.

### INTERNAL (default)

The camera streams freely. Frames are forwarded to the callback no faster than
once every `rateMs` milliseconds. No relationship with PPS or trigger signals.

```json
"rateMs": 500,
"trigger": { "mode": "INTERNAL" }
```

Use case: simple preview, debug, no synchronization needed.

---

### NEAREST_FRAME

The camera streams freely at the rate set by `FrameDurationLimits`.
A dedicated thread (`ppsWatchThread`) blocks on a GPIO pin waiting for edges.
On each edge it records the UTC nanosecond timestamp and sets `m_ppsPending = true`.
`requestComplete()` evaluates every incoming frame:

```
delta = |frame_utc - trigger_utc|
if delta < maxFrameAgeMs → accept frame, tag with trigger timestamp, clear m_ppsPending
else → silently drop frame and wait for a closer one
```

Net effect: one photo per trigger pulse, timestamped at the exact trigger edge.

#### Option A — 1 fps: fork GPIO 17 PPS signal to GPIO 22

The simplest wiring. The PPS_HIGH signal on GPIO 17 is forked to GPIO 22 for the
`ppsWatchThread`. The camera does not receive any physical trigger wire.
This produces **1 photo per second**, synchronized with the LiDAR PPS pulse.

Wiring:
```
RPi GPIO 17 (pin 11) ──┬──► Mid-360 M12 pin 8 (PPS)
                        └──► RPi GPIO 22 (pin 15) ──► ppsWatchThread
```

`pps_config.json` — no additional channel needed beyond GPIO 17:
```json
{
    "channels": [
        {
            "gpio": 17,
            "mode": "PPS_HIGH",
            "pulseMs": 100,
            "freqHz": 1
        }
    ]
}
```

`cam0_config.json` — trigger section:
```json
"trigger": {
    "mode": "NEAREST_FRAME",
    "gpioPin": 22,
    "gpioChip": "/dev/gpiochip0",
    "maxFrameAgeMs": 60,
    "edge": "rising"
}
```

`AeEnable: true` is acceptable — the camera adjusts exposure between frames automatically.

#### Option B — up to 50 fps: dedicated PPS_HIGH channel on GPIO 23

By adding a second fake_pps channel on GPIO 23 (PPS_HIGH) with a configurable `freqHz`,
the camera can be triggered at any supported frequency (1, 2, 4, 5, 8, 10, 20, 25, 40, 50 Hz)
instead of just 1 fps. The camera still runs free and the software selects the nearest frame —
but because the trigger rate now matches the camera rate, every trigger reliably hits a frame.

> **No physical connection to the camera is needed beyond the CSI cable.**
> GPIO 23 is connected directly to GPIO 22 — `ppsWatchThread` listens on GPIO 22
> to record the rising edge timestamp. GPIO 23 is not connected to any other device.

Wiring:
```
RPi GPIO 23 (pin 16) ──► RPi GPIO 22 (pin 15) ──► ppsWatchThread
(point-to-point — GPIO 23 is not connected to any camera pin)
```

`pps_config.json` — add a PPS_HIGH channel on GPIO 23 with the desired frequency:
```json
{
    "channels": [
        {
            "gpio": 17,
            "mode": "PPS_HIGH",
            "pulseMs": 100,
            "freqHz": 1,
            "_comment": "LiDAR PPS sync"
        },
        {
            "gpio": 23,
            "mode": "PPS_HIGH",
            "pulseMs": 10,
            "freqHz": 10,
            "_comment": "Camera timing reference 10Hz - GPIO23 wired directly to GPIO22"
        }
    ]
}
```

`cam0_config.json` — use `rising` edge on GPIO 22 to match PPS_HIGH:
```json
"trigger": {
    "mode": "NEAREST_FRAME",
    "gpioPin": 22,
    "gpioChip": "/dev/gpiochip0",
    "maxFrameAgeMs": 60,
    "edge": "rising"
}
```

Configure the camera free-running rate to match or exceed `freqHz`. For 10 Hz triggers,
set `FrameDurationLimits` to 100 000 µs (= 10 fps) or faster in `picamera` controls.

`AeEnable: true` is acceptable since the camera runs freely between software-selected frames.

#### maxFrameAgeMs selection guide

| Camera FPS | Frame Period | Recommended maxFrameAgeMs |
|-----------|-------------|--------------------------|
| 10 fps    | 100 ms      | 50–60 ms |
| 15 fps    | 67 ms       | 35–40 ms |
| 30 fps    | 33 ms       | 20 ms |

Set `maxFrameAgeMs` to slightly more than half the frame period.
Too tight → frames dropped; too wide → wrong frame accepted.

#### Precision

```
Timing precision = ±(frame_period / 2)
  At 10 fps → ±50 ms
  At 30 fps → ±17 ms
```

This is the maximum temporal uncertainty between the trigger edge and the
actual start of the accepted frame's exposure.

---

### HARDWARE_TRIGGER (recommended for Innomaker GS)

The Innomaker GS sensor only produces a frame when it receives a hardware pulse on XTR (Trig+).
Every frame is guaranteed to correspond to exactly one trigger event — no frame selection needed.

`ppsWatchThread` records the UTC timestamp of the trigger falling edge.
`requestComplete()` accepts every arriving frame and tags it with the last recorded
trigger timestamp. Only the **first** frame after each edge is accepted
(`m_ppsPending` mechanism) — extra frames buffered internally by libcamera are discarded.

Wiring:
```
RPi GPIO 23 (pin 16) ──┬──► XTR Trig+ Innomaker GS
                        └──► RPi GPIO 22 (pin 15) ──► ppsWatchThread
RPi GND               ──► GND Trig- Innomaker GS
```

`cam0_config.json`:
```json
"trigger": {
    "mode": "HARDWARE_TRIGGER",
    "gpioPin": 22,
    "gpioChip": "/dev/gpiochip0",
    "maxFrameAgeMs": 50,
    "edge": "falling"
}
```

- `maxFrameAgeMs` is ignored in this mode
- Requires `trigger_mode=1` in the IMX296 kernel module (see section 3)
- Requires `dtoverlay=imx296` in `/boot/firmware/config.txt`

#### Delta interpretation

```
[Camera] HARDWARE_TRIGGER accepted, trigger_utc=... frame_utc=... delta=10217 us
```

| Component | Typical value |
|-----------|--------------|
| Sensor exposure (pulseMs=10) | ~10 000 µs |
| Sensor readout | ~200 µs |
| **Total expected delta** | **~10 200 µs** |

A stable delta with low jitter (< ±50 µs) confirms correct hardware trigger operation.

#### Warm-up behavior

The first **5 trigger pulses** after startup are ignored by the Innomaker GS sensor.
This is normal sensor initialization — frames start from the 6th trigger onward.

#### Debounce

`ppsWatchThread` implements a 50 ms debounce window after each accepted edge.
At 10 Hz (100 ms period) this blocks any glitch at +10 ms without interfering
with the next legitimate trigger at +100 ms.

#### NEAREST_FRAME vs HARDWARE_TRIGGER for Innomaker GS

| | NEAREST_FRAME | HARDWARE_TRIGGER |
|---|---|---|
| Frame selection | Yes, checks delta | No, accepts all |
| maxFrameAgeMs | Used | Ignored |
| Sensor mode | Free-running | Stops without XTR pulses |
| Timing precision | ±50 ms at 10 fps | ~µs (hardware) |
| Requires trigger_mode=1 | No | **Yes** |
| Recommended | Fallback only | **Primary mode** |

---

## 6. JSON Configuration Reference

### /media/usb/pps_config.json

Controls the GPIO channels generated by `fake_pps`.
Auto-generated with hardware defaults on first run if the file does not exist.

#### Full example — LiDAR PPS + Innomaker GS at 10 Hz

```json
{
    "channels": [
        {
            "gpio": 17,
            "mode": "PPS_HIGH",
            "pulseMs": 100,
            "freqHz": 1,
            "_comment": "LiDAR PPS sync - RPi pin 11 (GPIO17) -> Mid-360 M12 pin 8"
        },
        {
            "gpio": 23,
            "mode": "TRIGGER_LOW",
            "pulseMs": 10,
            "freqHz": 10,
            "_comment": "Innomaker GS 10Hz - RPi pin 16 (GPIO23) -> XTR (Trig+)",
            "_note_cam0_config": "In cam0_config.json set: trigger.mode=HARDWARE_TRIGGER, trigger.edge=falling, picamera.ExposureTime=10000 (pulseMs*1000). If you change pulseMs here, update ExposureTime accordingly."
        }
    ]
}
```

#### ExposureTime rule

> **`picamera.ExposureTime` (µs) = `pulseMs` × 1000**

This value must be kept in sync manually between `pps_config.json` and `cam0_config.json`.
libcamera uses `ExposureTime` to configure the ISP pipeline even though the actual
sensor exposure is controlled by the hardware trigger pulse duration.

| pulseMs | ExposureTime | Recommended Use |
|---------|-------------|-----------------|
| 5       | 5000        | Cycling |
| 10      | 10000       | Walking (default) |
| 20      | 20000       | Low light / indoors |

### /media/usb/cam0_config.json examples

#### Generic Pi Camera — NEAREST_FRAME

```json
{
    "rateMs": 1000,
    "trigger": {
        "mode": "NEAREST_FRAME",
        "gpioPin": 22,
        "gpioChip": "/dev/gpiochip0",
        "maxFrameAgeMs": 60,
        "edge": "rising"
    },
    "picamera": {
        "AeEnable": true,
        "AnalogueGain": 1.0
    }
}
```

#### Innomaker GS — HARDWARE_TRIGGER (recommended)

```json
{
    "rateMs": 1000,
    "trigger": {
        "mode": "HARDWARE_TRIGGER",
        "gpioPin": 22,
        "gpioChip": "/dev/gpiochip0",
        "maxFrameAgeMs": 50,
        "edge": "falling"
    },
    "picamera": {
        "AeEnable": false,
        "ExposureTime": 10000,
        "AnalogueGain": 2.0
    }
}
```

### edge parameter reference

| Signal type | edge value | Used with |
|-------------|-----------|-----------|
| PPS_HIGH (GPIO 17, rising) | `"rising"` | NEAREST_FRAME Option A — any camera at 1 fps |
| PPS_HIGH (GPIO 23, rising) | `"rising"` | NEAREST_FRAME Option B — any camera up to 50 fps |
| TRIGGER_LOW (GPIO 23, falling) | `"falling"` | HARDWARE_TRIGGER — Innomaker GS only |

Default is `"rising"` if the parameter is omitted.

---

## 7. Build and Deployment

```bash
# Clone repository
cd ~
git clone https://github.com/GeomPozzoli/mandeye_controller.git
cd mandeye_controller
git checkout feature/pps-trigger-camera

# Build
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DMANDEYE_HARDWARE_HEADER=mandeye-standard-rpi4-pps.h
make -j1
sudo make install

# Reload and restart services
sudo systemctl daemon-reload
sudo systemctl restart mandeye_fake_pps.service
sudo systemctl restart mandeye_libcamera_cam0.service
```

### fake_pps service file

`/lib/systemd/system/mandeye_fake_pps.service`:

```ini
[Unit]
Description=Mandeye fake PPS
After=multi-user.target

[Service]
User=paso
ExecStartPre=/bin/sleep 5
ExecStart=/opt/mandeye/fake_pps
Restart=always
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

> The Innomaker GS `trigger_mode=1` is set persistently via `/etc/modprobe.d/imx296.conf`
> (see section 3) — no need to set it in the service file.

---

## 8. Verification

### Verify fake_pps

```bash
journalctl -u mandeye_fake_pps.service -f
```

Expected output:
```
[fake_pps] Channel thread started: GPIO=17 mode=PPS_HIGH pulseMs=100 freqHz=1
[fake_pps] Channel thread started: GPIO=23 mode=TRIGGER_LOW pulseMs=10 freqHz=10
[fake_pps] Master thread started.
```

Monitor GPIO 23 signal directly:
```bash
sudo gpiomon --falling-edge gpiochip0 23
# should print one event every 100 ms at 10 Hz
```

### Verify Innomaker GS trigger mode

```bash
cat /sys/module/imx296/parameters/trigger_mode
# expected: 1
```

### Verify camera — HARDWARE_TRIGGER (Innomaker GS)

```bash
sudo systemctl stop mandeye_libcamera_cam0.service
sudo /opt/mandeye/extras/mandeye_libcamera 0 8004 /media/usb/cam0_config.json
```

Expected log:
```
[Camera] TriggerMode=HARDWARE_TRIGGER GPIO=22 edge=falling maxFrameAge=50ms
[PPS] Listening for FALLING edge on GPIO 22
[PPS] Watch thread started, pin=22 chip=/dev/gpiochip0
[PPS] edge utc=... ns
[Camera] HARDWARE_TRIGGER accepted, trigger_utc=... frame_utc=... delta=10217 us
[PPS] edge utc=... ns
[Camera] HARDWARE_TRIGGER accepted, trigger_utc=... frame_utc=... delta=10217 us
```

Checks:
- ✅ Stable delta ~10.2 ms = exposure (10 ms) + readout (~200 µs)
- ✅ One `HARDWARE_TRIGGER accepted` line per `PPS edge` line
- ✅ First 5 triggers produce no frames (sensor warm-up, normal)

### Verify camera — NEAREST_FRAME

Expected log:
```
[Camera] TriggerMode=NEAREST_FRAME GPIO=22 edge=rising maxFrameAge=60ms
[PPS] Listening for RISING edge on GPIO 22
[PPS] Watch thread started, pin=22 chip=/dev/gpiochip0
[PPS] edge utc=... ns
[Camera] NEAREST_FRAME accepted, delta=... us
```

Checks:
- ✅ Delta < `maxFrameAgeMs * 1000` µs
- ✅ Positive delta = frame arrived after PPS edge (normal)
- ✅ Slightly negative delta = frame started just before edge (also acceptable)

### Definitive hardware trigger test (Innomaker GS only)

Stop fake_pps while the camera is running:
```bash
sudo systemctl stop mandeye_fake_pps.service
```

If HARDWARE_TRIGGER is working correctly, the camera stops producing frames within 1 second:
```
Dequeue timer of 1000000.00us has expired!
Camera frontend has timed out!
```

This is **expected behavior** — without XTR pulses the Innomaker GS sensor halts.
The service restarts automatically via `Restart=always`.

### Web preview

The HTTP preview server on port 8004 is always active, even in IDLE state.
Photos are saved to disk only during an active scan (SCANNING state).

```
http://<raspberry-ip>:8004
```

---

## 9. Troubleshooting

### trigger_mode stays 0 after reboot

Verify the modprobe config file:
```bash
cat /etc/modprobe.d/imx296.conf
# should show: options imx296 trigger_mode=1
```

### Camera running free despite trigger_mode=1

Set it manually before starting the camera:
```bash
sudo bash -c 'echo 1 > /sys/module/imx296/parameters/trigger_mode'
```

Verify that `/etc/modprobe.d/imx296.conf` exists and contains `options imx296 trigger_mode=1` (see section 3).

### Double frames per trigger

The `m_ppsPending` fix in HARDWARE_TRIGGER mode accepts only the first frame per edge.
Ensure `LibCameraWrapper.cpp` is up to date and recompiled.

### No frames arriving (trigger_mode=1 active)

Check wiring:
- GPIO 23 (pin 16) → XTR Trig+ Innomaker GS
- GPIO 22 (pin 15) → fork of GPIO 23
- GND → Trig- Innomaker GS

The first 5 triggers after startup are always ignored by the sensor — normal warm-up.

### NEAREST_FRAME: no frames accepted

Increase `maxFrameAgeMs`. The camera free-running rate must produce a frame
within `maxFrameAgeMs` of each trigger:

```
Required: fps > 1000 / (2 × maxFrameAgeMs)
Example:  maxFrameAgeMs=60 requires fps > 8.3  →  configure camera for 10+ fps
```

### Camera busy (Address already in use)

```bash
sudo systemctl stop mandeye_libcamera_cam0.service
sudo killall mandeye_libcamera 2>/dev/null
sleep 2
```

### Warning "Previous frame not saved yet — skipping this photo"

The JPEG save takes longer than the trigger period. Options:
- Reduce `freqHz` to 5 Hz in `pps_config.json`
- Use a USB 3.0/3.2 drive in the **blue** ports of the Pi 4
- Reduce JPEG quality in the save thread: `{cv::IMWRITE_JPEG_QUALITY, 85}`

---

## 10. Modified Files

| File | Change |
|------|--------|
| `code/fake_pps.cpp` | Multi-thread architecture (one thread per channel), freqHz support, TRIGGER_LOW mode |
| `extras/libcamera/LibCameraWrapper.h` | Added HARDWARE_TRIGGER mode, removed XVS_HARD (IMX477) |
| `extras/libcamera/LibCameraWrapper.cpp` | HARDWARE_TRIGGER implementation, configurable falling/rising edge, 50 ms debounce |
| `extras/libcamera/main.cpp` | Fixed race condition in async save thread (snapshot photo and metadata before thread launch) |
