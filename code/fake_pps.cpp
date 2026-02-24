#include <chrono>
#include <gpios.h>
#include <iostream>
#include <fstream>
#include <thread>
#include <atomic>
#include <stdio.h>
#include <vector>
#include <algorithm>
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
//    Used for: LiDAR sync, NEAREST_FRAME camera trigger.
//
//  TRIGGER_LOW
//    idle=HIGH, pulse=LOW for pulseMs at each trigger event.
//    freqHz controls how many triggers per second (1, 2, 5, 10, 20 ...).
//    The first trigger of each second is aligned to the PPS edge (T=0).
//    Subsequent triggers follow at intervals of 1000/freqHz ms.
//    Exposure = pulseMs + 14.26us (IMX296 sensor characteristic).
//    Used for: IMX296 XTR (Trig+) hardware trigger.
//
//  freqHz must divide 1000ms evenly, e.g.: 1, 2, 4, 5, 8, 10, 20, 25, 40, 50.
//  For PPS_HIGH always use freqHz=1.

enum class ChannelMode { PPS_HIGH, TRIGGER_LOW };

struct ChannelConfig {
    int         gpioPin {-1};
    ChannelMode mode    {ChannelMode::PPS_HIGH};
    uint32_t    pulseMs {100};
    uint32_t    freqHz  {1};    // triggers per second, synchronized to PPS
};

constexpr const char* CONFIG_PATH = "/media/usb/pps_config.json";

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
            // Validate freqHz: must divide 1000 evenly, max 50Hz
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
        "freqHz must divide 1000 evenly: 1,2,4,5,8,10,20,25,40,50.";

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
        j["channels_examples"].push_back(jex);
    }

    std::ofstream f(CONFIG_PATH);
    if (f.good()) {
        f << j.dump(4);
        std::cout << "[fake_pps] Default config saved to " << CONFIG_PATH << "\n";
    }
}

// ---- emitPulse helper ------------------------------------------------------
// Fires the active (pulse) level on the line, waits pulseMs, restores idle.

static void emitPulse(gpiod_line* line, ChannelMode mode, uint32_t pulseMs) {
    int activeVal = (mode == ChannelMode::PPS_HIGH) ? 1 : 0;
    int idleVal   = (mode == ChannelMode::PPS_HIGH) ? 0 : 1;
    gpiod_line_set_value(line, activeVal);
    std::this_thread::sleep_for(std::chrono::milliseconds(pulseMs));
    gpiod_line_set_value(line, idleVal);
}

// ---- PPS thread ------------------------------------------------------------

std::atomic<bool> stop{false};

void oneSecondThread() {
    // Serial ports for NMEA toward LiDAR
    std::vector<std::unique_ptr<LibSerial::SerialPort>> serialPorts;
    for (const auto& portName : hardware::GetLidarSyncPorts()) {
        std::cout << "[fake_pps] Opening port " << portName << "\n";
        auto sp = std::make_unique<LibSerial::SerialPort>();
        sp->Open(portName, std::ios_base::out);
        sp->SetBaudRate(LibSerial::BaudRate::BAUD_9600);
        serialPorts.emplace_back(std::move(sp));
    }

    // Channel config: JSON file takes priority, otherwise hardware defaults
    auto channelConfigs = loadConfig();
    const bool usedJson = !channelConfigs.empty();
    if (!usedJson) {
        for (const auto& led : hardware::GetLidarSyncLEDs())
            channelConfigs.push_back({hardware::GetLED(led), ChannelMode::PPS_HIGH, 100u, 1u});
    }

    // Open GPIO chip and request lines
    gpiod_chip* chip = gpiod_chip_open(mandeye::GetGPIOChip());
    if (!chip) { std::cerr << "[fake_pps] Cannot open GPIO chip.\n"; std::abort(); }

    struct Channel { ChannelConfig cfg; gpiod_line* line; };
    std::vector<Channel> channels;
    for (const auto& cfg : channelConfigs) {
        auto* line = gpiod_chip_get_line(chip, cfg.gpioPin);
        if (!line) {
            std::cerr << "[fake_pps] Cannot get GPIO line " << cfg.gpioPin << "\n";
            gpiod_chip_close(chip); std::abort();
        }
        int initVal = (cfg.mode == ChannelMode::PPS_HIGH) ? 0 : 1;
        if (gpiod_line_request_output(line, "mandeye_fake_pps", initVal) < 0) {
            std::cerr << "[fake_pps] Cannot request GPIO " << cfg.gpioPin << "\n";
            gpiod_chip_close(chip); std::abort();
        }
        channels.push_back({cfg, line});
        std::cout << "[fake_pps] GPIO " << cfg.gpioPin
                  << " mode=" << (cfg.mode == ChannelMode::PPS_HIGH ? "PPS_HIGH" : "TRIGGER_LOW")
                  << " pulseMs=" << cfg.pulseMs
                  << " freqHz=" << cfg.freqHz << "\n";
    }

    if (!usedJson) saveDefaultConfig(channelConfigs);

    // Align to next exact second boundary
    constexpr uint64_t Rate = 1000;
    uint64_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    ms = (ms / Rate + 1) * Rate;
    auto wakeUp = std::chrono::system_clock::time_point(std::chrono::milliseconds(ms));

    while (!stop) {
        // ---- Wait for next second boundary (T=0) ---------------------------
        std::this_thread::sleep_until(wakeUp);
        ms    += Rate;
        wakeUp = std::chrono::system_clock::time_point(std::chrono::milliseconds(ms));
        auto ts = NMEA::GetTimestampFromSec(ms / 1000);

        // ---- Separate channels by freqHz -----------------------------------
        // ppsChannels : PPS_HIGH or TRIGGER_LOW with freqHz=1  -> fire once at T=0
        // multiChannels: TRIGGER_LOW with freqHz>1             -> fire N times per second
        std::vector<size_t> ppsIdx, multiIdx;
        for (size_t i = 0; i < channels.size(); i++) {
            if (channels[i].cfg.freqHz <= 1)
                ppsIdx.push_back(i);
            else
                multiIdx.push_back(i);
        }

        // ---- T=0: fire all 1Hz channels simultaneously --------------------
        // PPS_HIGH  -> HIGH then LOW after pulseMs
        // TRIGGER_LOW 1Hz -> LOW then HIGH after pulseMs
        //
        // We fire pulses sequentially ordered by duration (shortest first)
        // so all rising/falling edges happen as close to T=0 as possible.
        std::vector<std::pair<uint32_t,size_t>> pulseOrder;
        for (size_t idx : ppsIdx)
            pulseOrder.push_back({channels[idx].cfg.pulseMs, idx});
        std::sort(pulseOrder.begin(), pulseOrder.end());

        // Fire all active edges at T=0
        for (auto& [pms, idx] : pulseOrder) {
            int val = (channels[idx].cfg.mode == ChannelMode::PPS_HIGH) ? 1 : 0;
            gpiod_line_set_value(channels[idx].line, val);
        }
        // End pulses in order of duration
        uint32_t elapsed = 0;
        for (auto& [pms, idx] : pulseOrder) {
            if (pms > elapsed) {
                std::this_thread::sleep_for(std::chrono::milliseconds(pms - elapsed));
                elapsed = pms;
            }
            int idleVal = (channels[idx].cfg.mode == ChannelMode::PPS_HIGH) ? 0 : 1;
            gpiod_line_set_value(channels[idx].line, idleVal);
        }

        // ---- Send NMEA after 1Hz pulses (within 0-430ms Livox window) ------
        const std::string nmea = NMEA::produceNMEA(ts);
        for (auto& sp : serialPorts) sp->Write(nmea);

        // ---- Multi-Hz channels: fire N times per second --------------------
        // The first pulse was already fired above at T=0 (elapsed ms).
        // Now fire the remaining N-1 pulses at intervals of periodMs.
        //
        // Example freqHz=10, periodMs=100:
        //   T=  0ms: pulse 1  (fired above with 1Hz channels)
        //   T=100ms: pulse 2
        //   T=200ms: pulse 3
        //   ....
        //   T=900ms: pulse 10
        //
        if (!multiIdx.empty()) {
            // Fire first pulse of multi-Hz channels at T=0 (same as 1Hz)
            for (size_t idx : multiIdx)
                emitPulse(channels[idx].line, channels[idx].cfg.mode, channels[idx].cfg.pulseMs);

            // Build schedule for remaining pulses
            // nextFireMs[i] = ms from T=0 when channel i fires next
            std::vector<uint32_t> nextFireMs(channels.size(), 0);
            for (size_t idx : multiIdx)
                nextFireMs[idx] = 1000u / channels[idx].cfg.freqHz; // first interval

            // Loop until we reach the end of this second
            // wakeUp is already set to T+1s so ms from T=0 is tracked via elapsed2
            uint32_t elapsed2 = elapsed; // ms already spent on 1Hz pulses + NMEA
            while (true) {
                // Find the nearest next fire time among all multi channels
                uint32_t nextMs = 1000;
                for (size_t idx : multiIdx)
                    nextMs = std::min(nextMs, nextFireMs[idx]);
                if (nextMs >= 1000) break; // all pulses done for this second

                // Sleep to that time
                if (nextMs > elapsed2) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(nextMs - elapsed2));
                    elapsed2 = nextMs;
                }

                // Fire all channels due at this time
                for (size_t idx : multiIdx) {
                    if (nextFireMs[idx] == nextMs) {
                        emitPulse(channels[idx].line,
                                  channels[idx].cfg.mode,
                                  channels[idx].cfg.pulseMs);
                        // Schedule next pulse for this channel
                        nextFireMs[idx] += 1000u / channels[idx].cfg.freqHz;
                    }
                }
            }
        }

        // ---- Wait for next second boundary ---------------------------------
        std::this_thread::sleep_until(wakeUp);
    }

    // Cleanup
    for (auto& ch : channels) {
        gpiod_line_set_value(ch.line, 0);
        gpiod_line_release(ch.line);
    }
    gpiod_chip_close(chip);
}

int main(int argc, char* argv[]) {
    std::cout << "fake_pps starting\n";
    std::thread t1(oneSecondThread);
    while (!stop)
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    t1.join();
    return 0;
}
