#include <chrono>
#include <gpios.h>
#include <iostream>
#include <fstream>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <stdio.h>
#include <vector>
#include <SerialPort.h>
#include <SerialStream.h>
#include <hardware_config/mandeye.h>
#include <gpiod.h>
#include "json.hpp"

namespace NMEA {
const unsigned int BufferLen = 128;
struct timestamp { uint8_t hours, mins, secs, day, month, year; };

std::string produceNMEA(const NMEA::timestamp& ts) {
    char buffer[BufferLen], payload[BufferLen];
    snprintf(payload, BufferLen,
        "GPRMC,%02d%02d%02d.00,A,5109.0262308,N,11401.8407342,W,0.004,133.4,%02d%02d%02d,0.0,E,D",
        ts.hours, ts.mins, ts.secs, ts.day, ts.month, ts.year);
    uint8_t chk = 0;
    for (size_t i = 0; i < strnlen(payload, BufferLen); i++) chk ^= payload[i];
    snprintf(buffer, BufferLen, "$%s*%02X\n", payload, chk);
    return std::string(buffer);
}

NMEA::timestamp GetTimestampFromSec(time_t t) {
    std::tm* ti = gmtime(&t);
    return {(uint8_t)ti->tm_hour, (uint8_t)ti->tm_min, (uint8_t)ti->tm_sec,
            (uint8_t)ti->tm_mday, (uint8_t)(ti->tm_mon+1), (uint8_t)(ti->tm_year-100)};
}
} // namespace NMEA

// ---- Channel mode ----------------------------------------------------------
//
//  PPS_HIGH
//    idle=LOW, pulse=HIGH for pulseMs at T=0 (rising edge = PPS reference).
//    freqHz must be 1 for this mode.
//    Used for: LiDAR sync, NEAREST_FRAME camera trigger (IMX219).
//
//  TRIGGER_LOW
//    idle=HIGH, pulse=LOW for pulseMs at each trigger event.
//    freqHz controls how many triggers per second (1, 2, 5, 10, 20 ...).
//    The first trigger of each second is aligned to the PPS edge (T=0).
//    Subsequent triggers follow at intervals of 1000/freqHz ms.
//    Exposure = pulseMs + 14.26us (IMX296 sensor characteristic).
//    Used for: IMX296 XTR (Trig+) hardware trigger.
//
//  Each channel runs in its own dedicated thread, synchronized by a
//  condition_variable broadcast at each second boundary. This avoids
//  timing interference between channels (e.g. PPS_HIGH sleep blocking
//  TRIGGER_LOW pulses).
//
//  freqHz must divide 1000ms evenly: 1,2,4,5,8,10,20,25,40,50.
//  For PPS_HIGH always use freqHz=1.

enum class ChannelMode { PPS_HIGH, TRIGGER_LOW };

struct ChannelConfig {
    int         gpioPin {-1};
    ChannelMode mode    {ChannelMode::PPS_HIGH};
    uint32_t    pulseMs {100};
    uint32_t    freqHz  {1};
};

constexpr const char* CONFIG_PATH = "/media/usb/pps_config.json";

// ---- Sync primitives -------------------------------------------------------
// The master thread broadcasts on g_tickCv at each second boundary.
// Each channel thread wakes up, fires its pulses, then waits again.

std::atomic<bool>       g_stop{false};
std::mutex              g_tickMtx;
std::condition_variable g_tickCv;
std::atomic<uint64_t>   g_tickMs{0}; // epoch-ms of the most recent second boundary

// ---- loadConfig ------------------------------------------------------------

std::vector<ChannelConfig> loadConfig() {
    std::vector<ChannelConfig> channels;
    std::ifstream f(CONFIG_PATH);
    if (!f.good()) {
        std::cout << "[fake_pps] No " << CONFIG_PATH << " found, using hardware defaults.\n";
        return channels;
    }
    try {
        nlohmann::json j; f >> j;
        for (const auto& ch : j["channels"]) {
            if (ch.contains("_disabled") && ch["_disabled"].get<bool>()) continue;
            ChannelConfig cfg;
            cfg.gpioPin = ch.value("gpio", -1);
            cfg.pulseMs = ch.value("pulseMs", 100u);
            cfg.freqHz  = ch.value("freqHz", 1u);
            cfg.mode    = (ch.value("mode","PPS_HIGH") == "TRIGGER_LOW")
                          ? ChannelMode::TRIGGER_LOW : ChannelMode::PPS_HIGH;
            if (cfg.freqHz == 0 || 1000 % cfg.freqHz != 0 || cfg.freqHz > 50) {
                std::cerr << "[fake_pps] GPIO " << cfg.gpioPin
                          << ": freqHz=" << cfg.freqHz
                          << " invalid (must divide 1000 evenly, max 50). Defaulting to 1.\n";
                cfg.freqHz = 1;
            }
            if (cfg.gpioPin >= 0) {
                channels.push_back(cfg);
                std::cout << "[fake_pps] Channel GPIO=" << cfg.gpioPin
                          << " mode=" << ch.value("mode","PPS_HIGH")
                          << " pulseMs=" << cfg.pulseMs
                          << " freqHz=" << cfg.freqHz << "\n";
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "[fake_pps] Config error: " << e.what() << ". Using defaults.\n";
        channels.clear();
    }
    return channels;
}

// ---- saveDefaultConfig -----------------------------------------------------

void saveDefaultConfig(const std::vector<ChannelConfig>& channels) {
    nlohmann::json j;
    j["_help"] =
        "PPS_HIGH: pin goes HIGH for pulseMs each second (LiDAR + NEAREST_FRAME camera), freqHz must be 1. "
        "TRIGGER_LOW: pin goes LOW for pulseMs at each trigger event, freqHz controls triggers/second. "
        "First trigger of each second is aligned to PPS edge (T=0). "
        "Exposure = pulseMs + 14.26us (IMX296). "
        "freqHz must divide 1000 evenly: 1,2,4,5,8,10,20,25,40,50. "
        "Each channel runs in its own thread - no timing interference between channels.";

    for (const auto& ch : channels) {
        nlohmann::json jch;
        jch["gpio"]     = ch.gpioPin;
        jch["mode"]     = (ch.mode == ChannelMode::PPS_HIGH) ? "PPS_HIGH" : "TRIGGER_LOW";
        jch["pulseMs"]  = ch.pulseMs;
        jch["freqHz"]   = ch.freqHz;
        jch["_comment"] = "LiDAR PPS sync - RPi pin 11 (GPIO17) -> Mid-360 M12 pin 8";
        j["channels"].push_back(jch);
    }

    // Example: IMX296 at 1Hz
    {
        nlohmann::json jex;
        jex["_disabled"] = true;
        jex["gpio"]      = 23;
        jex["mode"]      = "TRIGGER_LOW";
        jex["pulseMs"]   = 10;
        jex["freqHz"]    = 1;
        jex["_comment"]  =
            "IMX296 1Hz - RPi pin 16 (GPIO23) -> XTR (Trig+). "
            "Exposure=10ms+14.26us. Move to channels[] and remove _disabled to activate.";
        jex["_note_cam0_config"] =
            "In cam0_config.json set: trigger.mode=HARDWARE_TRIGGER, trigger.edge=falling, "
            "picamera.ExposureTime=10000 (pulseMs*1000). "
            "If you change pulseMs here, update ExposureTime accordingly.";
        j["channels_examples"].push_back(jex);
    }
    // Example: IMX296 at 10Hz
    {
        nlohmann::json jex;
        jex["_disabled"] = true;
        jex["gpio"]      = 23;
        jex["mode"]      = "TRIGGER_LOW";
        jex["pulseMs"]   = 10;
        jex["freqHz"]    = 10;
        jex["_comment"]  =
            "IMX296 10Hz - RPi pin 16 (GPIO23) -> XTR (Trig+). "
            "10 triggers/s, first aligned to PPS. Exposure=10ms+14.26us. "
            "Move to channels[] and remove _disabled to activate.";
        jex["_note_cam0_config"] =
            "In cam0_config.json set: trigger.mode=HARDWARE_TRIGGER, trigger.edge=falling, "
            "picamera.ExposureTime=10000 (pulseMs*1000). "
            "If you change pulseMs here, update ExposureTime accordingly.";
        j["channels_examples"].push_back(jex);
    }

    std::ofstream f(CONFIG_PATH);
    if (f.good()) {
        f << j.dump(4);
        std::cout << "[fake_pps] Default config saved to " << CONFIG_PATH << "\n";
    }
}

// ---- emitPulse -------------------------------------------------------------

static void emitPulse(gpiod_line* line, ChannelMode mode, uint32_t pulseMs) {
    int activeVal = (mode == ChannelMode::PPS_HIGH) ? 1 : 0;
    int idleVal   = (mode == ChannelMode::PPS_HIGH) ? 0 : 1;
    gpiod_line_set_value(line, activeVal);
    std::this_thread::sleep_for(std::chrono::milliseconds(pulseMs));
    gpiod_line_set_value(line, idleVal);
}

// ---- channelThread ---------------------------------------------------------
// One thread per channel. Waits for g_tickCv broadcast, then fires pulses
// at the configured frequency for the duration of that second.
// Completely independent from other channels - no shared sleep().

void channelThread(ChannelConfig cfg, gpiod_line* line)
{
    const uint32_t periodMs = 1000u / cfg.freqHz;
    const std::string modeName = (cfg.mode == ChannelMode::PPS_HIGH) ? "PPS_HIGH" : "TRIGGER_LOW";
    std::cout << "[fake_pps] Channel thread started: GPIO=" << cfg.gpioPin
              << " mode=" << modeName
              << " pulseMs=" << cfg.pulseMs
              << " freqHz=" << cfg.freqHz << "\n";

    uint64_t lastTick = 0;

    while (!g_stop.load()) {
        // Wait for the next second boundary broadcast from master thread
        uint64_t thisTick;
        {
            std::unique_lock<std::mutex> lk(g_tickMtx);
            g_tickCv.wait(lk, [&]{
                return g_stop.load() || g_tickMs.load() != lastTick;
            });
            if (g_stop.load()) break;
            thisTick = g_tickMs.load();
        }
        lastTick = thisTick;

        // T=0: fire first pulse immediately
        emitPulse(line, cfg.mode, cfg.pulseMs);

        // Fire remaining pulses at intervals of periodMs from T=0
        // Use absolute time points to avoid drift accumulation
        const auto t0 = std::chrono::system_clock::time_point(
            std::chrono::milliseconds(thisTick));

        for (uint32_t n = 1; n < cfg.freqHz; n++) {
            // Sleep until T=0 + n*periodMs
            auto fireAt = t0 + std::chrono::milliseconds(uint64_t(n) * periodMs);
            std::this_thread::sleep_until(fireAt);
            if (g_stop.load()) break;
            emitPulse(line, cfg.mode, cfg.pulseMs);
        }
    }

    // Cleanup: restore pin to idle state
    int idleVal = (cfg.mode == ChannelMode::PPS_HIGH) ? 0 : 1;
    gpiod_line_set_value(line, idleVal);
    std::cout << "[fake_pps] Channel thread exiting: GPIO=" << cfg.gpioPin << "\n";
}

// ---- masterThread ----------------------------------------------------------
// Handles NMEA serial output and broadcasts the second boundary tick
// to all channel threads via condition_variable.

void masterThread(std::vector<std::unique_ptr<LibSerial::SerialPort>>& serialPorts)
{
    std::cout << "[fake_pps] Master thread started.\n";

    constexpr uint64_t Rate = 1000;
    uint64_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    ms = (ms / Rate + 1) * Rate;
    auto wakeUp = std::chrono::system_clock::time_point(std::chrono::milliseconds(ms));

    while (!g_stop.load()) {
        std::this_thread::sleep_until(wakeUp);
        ms    += Rate;
        wakeUp = std::chrono::system_clock::time_point(std::chrono::milliseconds(ms));

        // Broadcast tick to all channel threads
        {
            std::lock_guard<std::mutex> lk(g_tickMtx);
            g_tickMs.store(ms - Rate); // store T=0 of this second (ms - Rate = current second start)
        }
        g_tickCv.notify_all();

        // Send NMEA string (within 0-430ms window per Livox spec)
        // Sent after broadcast so channel threads start their pulses immediately
        auto ts = NMEA::GetTimestampFromSec((ms - Rate) / 1000);
        const std::string nmea = NMEA::produceNMEA(ts);
        for (auto& sp : serialPorts) sp->Write(nmea);
    }

    std::cout << "[fake_pps] Master thread exiting.\n";
}

// ---- main ------------------------------------------------------------------

int main(int argc, char* argv[]) {
    std::cout << "fake_pps starting\n";

    // Serial ports for NMEA toward LiDAR
    std::vector<std::unique_ptr<LibSerial::SerialPort>> serialPorts;
    for (const auto& portName : hardware::GetLidarSyncPorts()) {
        std::cout << "[fake_pps] Opening port " << portName << "\n";
        auto sp = std::make_unique<LibSerial::SerialPort>();
        sp->Open(portName, std::ios_base::out);
        sp->SetBaudRate(LibSerial::BaudRate::BAUD_9600);
        serialPorts.emplace_back(std::move(sp));
    }

    // Load channel config
    auto channelConfigs = loadConfig();
    const bool usedJson = !channelConfigs.empty();
    if (!usedJson) {
        for (const auto& led : hardware::GetLidarSyncLEDs())
            channelConfigs.push_back({hardware::GetLED(led), ChannelMode::PPS_HIGH, 100u, 1u});
    }

    // Open GPIO chip and request lines
    gpiod_chip* chip = gpiod_chip_open(mandeye::GetGPIOChip());
    if (!chip) { std::cerr << "[fake_pps] Cannot open GPIO chip.\n"; return 1; }

    struct Channel { ChannelConfig cfg; gpiod_line* line; };
    std::vector<Channel> channels;
    for (const auto& cfg : channelConfigs) {
        auto* line = gpiod_chip_get_line(chip, cfg.gpioPin);
        if (!line) {
            std::cerr << "[fake_pps] Cannot get GPIO line " << cfg.gpioPin << "\n";
            gpiod_chip_close(chip); return 1;
        }
        int initVal = (cfg.mode == ChannelMode::PPS_HIGH) ? 0 : 1;
        if (gpiod_line_request_output(line, "mandeye_fake_pps", initVal) < 0) {
            std::cerr << "[fake_pps] Cannot request GPIO " << cfg.gpioPin << "\n";
            gpiod_chip_close(chip); return 1;
        }
        channels.push_back({cfg, line});
    }

    if (!usedJson) saveDefaultConfig(channelConfigs);

    // Start one thread per channel
    std::vector<std::thread> threads;
    for (auto& ch : channels)
        threads.emplace_back(channelThread, ch.cfg, ch.line);

    // Start master thread (NMEA + tick broadcast)
    std::thread master(masterThread, std::ref(serialPorts));

    // Wait forever (service runs until systemd stops it)
    while (!g_stop.load())
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Shutdown
    g_stop.store(true);
    g_tickCv.notify_all();
    master.join();
    for (auto& t : threads) t.join();

    // Cleanup GPIO
    for (auto& ch : channels) {
        gpiod_line_set_value(ch.line, 0);
        gpiod_line_release(ch.line);
    }
    gpiod_chip_close(chip);

    std::cout << "fake_pps stopped.\n";
    return 0;
}
