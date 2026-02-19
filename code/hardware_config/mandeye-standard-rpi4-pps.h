#pragma once

/**
 * mandeye-standard-rpi4-pps.h
 *
 * Hardware configuration for Raspberry Pi 4 (standard, off-the-shelf)
 * with fake-PPS synchronization support for one Livox Mid-360.
 *
 * This variant adds hardware PPS sync (GPIO pulse + NMEA over UART)
 * to the standard RPi4 build, bringing it closer to the Pro CM4 behaviour.
 *
 * ─── Required /boot/config.txt additions ────────────────────────────────────
 *   # Enable UART2 on GPIO 0 (TX) and GPIO 1 (RX) — used for NMEA to LiDAR
 *   dtoverlay=uart2
 *
 *   # If Bluetooth is enabled (default on RPi4), disable it to free up the
 *   # full PL011 UART for the GNSS receiver on /dev/ttyAMA0:
 *   dtoverlay=disable-bt
 *   # (then: sudo systemctl disable hciuart)
 * ────────────────────────────────────────────────────────────────────────────
 *
 * ─── Wiring summary ─────────────────────────────────────────────────────────
 *
 *  RPi4 40-pin header                Livox Mid-360 sync port (6-pin)
 *  ─────────────────                 ───────────────────────────────
 *  GPIO 17  (pin 11)  ──────────────►  PPS IN          (pulse 0→1 each second)
 *  GPIO  0  (pin 27, TX UART2) ─────►  UART RX / TIMESYNC IN  (NMEA @9600)
 *  GND      (pin 9 or 25)  ──────────  GND
 *
 *  RPi4 40-pin header                GNSS receiver (e.g. u-blox M8/M9)
 *  ─────────────────                 ──────────────────────────────────
 *  GPIO 14  (pin  8, TX AMA0) ──────►  RX  (optional, for config commands)
 *  GPIO 15  (pin 10, RX AMA0) ◄──────  TX  (NMEA sentences)
 *  GND      (pin  6)          ──────── GND
 *  3.3V     (pin  1)          ──────── VCC  (check receiver spec!)
 *
 * ─── Pin usage table ────────────────────────────────────────────────────────
 *  GPIO  2  — free (I2C SDA, avoid if I2C in use)
 *  GPIO  3  — free (I2C SCL, avoid if I2C in use)
 *  GPIO  5  — BUTTON_STOP_SCAN        (pull-up, active low)
 *  GPIO  6  — BUTTON_CONTINOUS_SCAN   (pull-up, active low)
 *  GPIO 12  — BUZZER                  (active high)
 *  GPIO 13  — LED_GPIO_CONTINOUS_SCAN (active high)
 *  GPIO 14  — UART0 TX  → GNSS RX    (via /dev/ttyAMA0, needs disable-bt)
 *  GPIO 15  — UART0 RX  ← GNSS TX    (via /dev/ttyAMA0, needs disable-bt)
 *  GPIO 17  — LIDAR_SYNC_1 PPS OUT   ← NEW
 *  GPIO  0  — UART2 TX  → LiDAR NMEA (via /dev/ttyAMA1, needs dtoverlay=uart2) ← NEW
 *  GPIO 19  — LED_GPIO_COPY_DATA      (active high)
 *  GPIO 26  — LED_GPIO_STOP_SCAN      (active high)
 * ────────────────────────────────────────────────────────────────────────────
 */

#include "hardware_common.h"
#ifdef MANDEYE_HARDWARE_CONFIGURED
#   error "MANDEYE Hardware were configured. You included multiple hardware headers!"
#endif

#define MANDEYE_HARDWARE_CONFIGURED

namespace hardware
{
#define PISTACHE_SERVER

constexpr int Offset = 0;
constexpr bool Autostart = false;

// Set to true to make the system wait up to 60 s for PPS lock before
// transitioning to IDLE. Recommended once wiring is verified.
constexpr bool WaitForLidarSync = false;

constexpr const char* mandeyeHarwareType()
{
    return "MandeyeStandardRPi4PPS";
}

constexpr const char* GetGPIOChip()
{
    return "/dev/gpiochip0";
}

inline void ReportState([[maybe_unused]] const mandeye::States state)
{
    // no-op — extend here if you want LED patterns per state
}

inline void OnSavedLaz([[maybe_unused]] const std::string& filename)
{
    // no-op
}

constexpr int GetLED(LED led)
{
    if (led == LED::LED_GPIO_STOP_SCAN)
        return 26;

    if (led == LED::LED_GPIO_COPY_DATA)
        return 19;

    if (led == LED::LED_GPIO_CONTINOUS_SCANNING)
        return 13;

    // PPS output pin for LiDAR 1 sync — driven by fake_pps service
    if (led == LED::LIDAR_SYNC_1)
        return 17;

    // Only one LiDAR supported on standard RPi4 header
    if (led == LED::LIDAR_SYNC_2)
        return -1;

    if (led == LED::BUZZER)
        return 12;

    return -1;
}

constexpr int GetButton(BUTTON btn)
{
    if (btn == BUTTON::BUTTON_STOP_SCAN)
        return 5;

    if (btn == BUTTON::BUTTON_CONTINOUS_SCANNING)
        return 6;

    return -1;
}

constexpr GPIO::GPIO_PULL GetPULL([[maybe_unused]] BUTTON btn)
{
    return GPIO::GPIO_PULL::UP;
}

// One sync output: GPIO 17 drives the PPS pulse to the LiDAR
[[maybe_unused]] inline const std::array<LED, 1> GetLidarSyncLEDs()
{
    return {LED::LIDAR_SYNC_1};
}

// One UART for NMEA: /dev/ttyAMA1 enabled via dtoverlay=uart2
// TX on GPIO 0 (RPi4 header pin 27) -> connected to LiDAR TIMESYNC/UART RX
[[maybe_unused]] inline const std::array<const std::string, 1> GetLidarSyncPorts()
{
    return {"/dev/ttyAMA2"};
}

// GNSS receiver on /dev/ttyAMA0 (requires dtoverlay=disable-bt in config.txt)
// If Bluetooth cannot be disabled, use /dev/ttyS0 at a lower baud rate instead.
[[maybe_unused]] inline const std::string GetGNSSPort()
{
    return "/dev/ttyAMA0";
}

[[maybe_unused]] inline const LibSerial::BaudRate GetGNSSBaudrate()
{
    return LibSerial::BaudRate::BAUD_38400;
}

} // namespace hardware
