#include "LibCameraWrapper.h"
#include <bits/this_thread_sleep.h>
#include <sys/mman.h>
#include <gpiod.h>

using namespace libcamera;
using namespace std::chrono_literals;

namespace mandeye {

// -- string helpers -----------------------------------------------------------

constexpr std::string_view controlToString(uint32_t id)
{
    using namespace libcamera::controls;
    switch (id) {
    case AE_ENABLE:                return "AE_ENABLE";
    case AE_STATE:                 return "AE_STATE";
    case AE_METERING_MODE:         return "AE_METERING_MODE";
    case AE_CONSTRAINT_MODE:       return "AE_CONSTRAINT_MODE";
    case AE_EXPOSURE_MODE:         return "AE_EXPOSURE_MODE";
    case EXPOSURE_VALUE:           return "EXPOSURE_VALUE";
    case EXPOSURE_TIME:            return "EXPOSURE_TIME";
    case EXPOSURE_TIME_MODE:       return "EXPOSURE_TIME_MODE";
    case ANALOGUE_GAIN:            return "ANALOGUE_GAIN";
    case ANALOGUE_GAIN_MODE:       return "ANALOGUE_GAIN_MODE";
    case AE_FLICKER_MODE:          return "AE_FLICKER_MODE";
    case AE_FLICKER_PERIOD:        return "AE_FLICKER_PERIOD";
    case AE_FLICKER_DETECTED:      return "AE_FLICKER_DETECTED";
    case BRIGHTNESS:               return "BRIGHTNESS";
    case CONTRAST:                 return "CONTRAST";
    case LUX:                      return "LUX";
    case AWB_ENABLE:               return "AWB_ENABLE";
    case AWB_MODE:                 return "AWB_MODE";
    case AWB_LOCKED:               return "AWB_LOCKED";
    case COLOUR_GAINS:             return "COLOUR_GAINS";
    case COLOUR_TEMPERATURE:       return "COLOUR_TEMPERATURE";
    case SATURATION:               return "SATURATION";
    case SENSOR_BLACK_LEVELS:      return "SENSOR_BLACK_LEVELS";
    case SHARPNESS:                return "SHARPNESS";
    case FOCUS_FO_M:               return "FOCUS_FO_M";
    case COLOUR_CORRECTION_MATRIX: return "COLOUR_CORRECTION_MATRIX";
    case SCALER_CROP:              return "SCALER_CROP";
    case DIGITAL_GAIN:             return "DIGITAL_GAIN";
    case FRAME_DURATION:           return "FRAME_DURATION";
    case FRAME_DURATION_LIMITS:    return "FRAME_DURATION_LIMITS";
    case SENSOR_TEMPERATURE:       return "SENSOR_TEMPERATURE";
    case SENSOR_TIMESTAMP:         return "SENSOR_TIMESTAMP";
    case AF_MODE:                  return "AF_MODE";
    case AF_RANGE:                 return "AF_RANGE";
    case AF_SPEED:                 return "AF_SPEED";
    case AF_METERING:              return "AF_METERING";
    case AF_WINDOWS:               return "AF_WINDOWS";
    case AF_TRIGGER:               return "AF_TRIGGER";
    case AF_PAUSE:                 return "AF_PAUSE";
    case LENS_POSITION:            return "LENS_POSITION";
    case AF_STATE:                 return "AF_STATE";
    case AF_PAUSE_STATE:           return "AF_PAUSE_STATE";
    case HDR_MODE:                 return "HDR_MODE";
    case HDR_CHANNEL:              return "HDR_CHANNEL";
    case GAMMA:                    return "GAMMA";
    case DEBUG_METADATA_ENABLE:    return "DEBUG_METADATA_ENABLE";
    case FRAME_WALL_CLOCK:         return "FRAME_WALL_CLOCK";
    default:                       return "";
    }
}

uint64_t getCurrentTimestamp() {
    using namespace std::chrono;
    return static_cast<uint64_t>(
        duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count());
}

// -- control helpers ----------------------------------------------------------

template <typename T>
static bool checkIfInRange(const libcamera::ControlInfo &ctrl, const T &value) {
    T mn = ctrl.min().get<T>(), mx = ctrl.max().get<T>();
    if (value < mn || value > mx) {
        std::cerr << "Value " << value << " out of range [" << mn << "," << mx << "]\n";
        return false;
    }
    return true;
}

template<typename T>
bool LibCameraWrapper::setControlNumeric(const std::string &name, T valueInput) {
    for (auto const &ctrl : m_controlsInfo) {
        if (ctrl.first->name() != name) continue;
        const int id   = ctrl.first->id();
        const int type = ctrl.first->type();
        if (ctrl.first->isArray())             { std::cerr << name << " is array\n";    return false; }
        if (type == libcamera::ControlTypeNone){ std::cerr << name << " unsupported\n"; return false; }
        if      (type == libcamera::ControlTypeBool)
            m_controlList.set(id, static_cast<bool>(valueInput));
        else if (type == libcamera::ControlTypeByte) {
            auto v = static_cast<uint8_t>(valueInput);
            if (checkIfInRange<uint8_t>(ctrl.second, v)) m_controlList.set(id, v);
        }
        else if (type == libcamera::ControlTypeUnsigned16) {
            auto v = static_cast<uint16_t>(valueInput);
            if (checkIfInRange<uint16_t>(ctrl.second, v)) m_controlList.set(id, v);
        }
        else if (type == libcamera::ControlTypeInteger32) {
            auto v = static_cast<int32_t>(valueInput);
            if (checkIfInRange<int32_t>(ctrl.second, v)) m_controlList.set(id, v);
        }
        else if (type == libcamera::ControlTypeInteger64) {
            auto v = static_cast<int64_t>(valueInput);
            if (checkIfInRange<int64_t>(ctrl.second, v)) m_controlList.set(id, v);
        }
        else if (type == libcamera::ControlTypeFloat) {
            auto v = static_cast<float>(valueInput);
            if (checkIfInRange<float>(ctrl.second, v)) m_controlList.set(id, v);
        }
        else if (type == libcamera::ControlTypeString)
            m_controlList.set(id, ControlValue(std::to_string(valueInput)));
        return true;
    }
    std::cerr << "Control " << name << " not found\n";
    return false;
}

template bool LibCameraWrapper::setControlNumeric<bool>      (const std::string&, bool);
template bool LibCameraWrapper::setControlNumeric<int64_t>   (const std::string&, int64_t);
template bool LibCameraWrapper::setControlNumeric<float>     (const std::string&, float);

// -- buffer helpers -----------------------------------------------------------

std::vector<libcamera::Span<uint8_t>> LibCameraWrapper::Mmap(libcamera::FrameBuffer *buf) {
    auto it = m_mapped_buffers.find(buf);
    return (it == m_mapped_buffers.end()) ? std::vector<libcamera::Span<uint8_t>>{} : it->second;
}

std::vector<std::unique_ptr<Request>> LibCameraWrapper::CreateRequests() {
    std::vector<std::unique_ptr<Request>> reqs;
    const auto &bufs = m_allocator->buffers(m_stream);
    for (unsigned i = 0; i < bufs.size(); ++i) {
        auto req = m_camera->createRequest();
        if (!req) { std::cerr << "Can't create request\n"; return {}; }
        if (req->addBuffer(m_stream, bufs[i].get()) < 0) {
            std::cerr << "Can't set buffer for request\n"; return {};
        }
        reqs.push_back(std::move(req));
    }
    return reqs;
}

void LibCameraWrapper::AdjustSystemClock() {
    timespec ts_real, ts_mono;
    clock_gettime(CLOCK_REALTIME,  &ts_real);
    clock_gettime(CLOCK_MONOTONIC, &ts_mono);
    uint64_t real_ns = uint64_t(ts_real.tv_sec)*1'000'000'000ULL + ts_real.tv_nsec;
    uint64_t mono_ns = uint64_t(ts_mono.tv_sec)*1'000'000'000ULL + ts_mono.tv_nsec;
    assert(real_ns > mono_ns);
    m_monoOffset = real_ns - mono_ns;
}

// -- ppsWatchThread -----------------------------------------------------------
//
// Blocks on GPIO edge events (rising or falling, configured via m_triggerEdgeFalling).
//
// The gpiod event structure carries a kernel timestamp (CLOCK_MONOTONIC),
// which we convert to UTC using the same m_monoOffset used for sensor
// timestamps. This keeps trigger timestamps and frame timestamps in the same
// reference frame without any additional offset.
//
// For NEAREST_FRAME:    sets m_ppsPending = true so requestComplete() knows
//                       to look for the next matching frame.
// For HARDWARE_TRIGGER: records the trigger UTC timestamp; requestComplete()
//                       accepts every frame and tags it with this timestamp.
// For XVS_HARD:         the sensor drives itself; we just record the timestamp
//                       for metadata so post-processing can verify alignment.
//
void LibCameraWrapper::ppsWatchThread()
{
    std::cout << "[PPS] Watch thread started, pin=" << m_triggerGpioPin
              << " chip=" << m_gpioChipPath << "\n";

    constexpr timespec timeout{0, 50'000'000}; // 50 ms — lets us check m_running

    // Debounce: after accepting an edge, ignore further edges for this many ns.
    // For TRIGGER_LOW at 10Hz (period=100ms), 50ms blocks the spurious rising
    // edge that arrives 10ms after the falling edge (end of pulse).
    // For PPS_HIGH at 1Hz (period=1000ms), 50ms is negligible.
    const uint64_t debounceNs = 50'000'000ULL; // 50 ms
    uint64_t lastAcceptedMonoNs = 0;

    while (m_running.load())
    {
        int ret = gpiod_line_event_wait(m_ppsLine, &timeout);
        if (ret < 0) {
            if (errno == EINTR) continue;
            std::cerr << "[PPS] gpiod_line_event_wait: " << strerror(errno) << "\n";
            break;
        }
        if (ret == 0) continue; // timeout, loop back

        gpiod_line_event ev;
        if (gpiod_line_event_read(m_ppsLine, &ev) < 0) continue;

        // Accept the configured edge type only
        const int wantedEdge = m_triggerEdgeFalling
                               ? GPIOD_LINE_EVENT_FALLING_EDGE
                               : GPIOD_LINE_EVENT_RISING_EDGE;
        if (ev.event_type != wantedEdge) continue;

        // Debounce: ignore edges that arrive too soon after the last accepted one
        const uint64_t evMonoNs = uint64_t(ev.ts.tv_sec)*1'000'000'000ULL + ev.ts.tv_nsec;
        if (lastAcceptedMonoNs > 0 && evMonoNs - lastAcceptedMonoNs < debounceNs) {
            std::cout << "[PPS] edge ignored (debounce), dt="
                      << (evMonoNs - lastAcceptedMonoNs) / 1'000'000 << " ms\n";
            continue;
        }
        lastAcceptedMonoNs = evMonoNs;

        // Convert CLOCK_MONOTONIC to UTC
        AdjustSystemClock();
        const uint64_t evUtcNs = evMonoNs + m_monoOffset;

        m_lastPpsNs.store(evUtcNs, std::memory_order_release);
        m_ppsPending.store(true,   std::memory_order_release);

        std::cout << "[PPS] edge utc=" << evUtcNs << " ns\n";
    }
    std::cout << "[PPS] Watch thread exiting.\n";
}

void LibCameraWrapper::releasePpsGpio()
{
    if (m_ppsLine) { gpiod_line_release(m_ppsLine); m_ppsLine = nullptr; }
    if (m_ppsChip) { gpiod_chip_close(m_ppsChip);   m_ppsChip = nullptr; }
}

// -- start() ------------------------------------------------------------------

bool LibCameraWrapper::start(int camNo, nlohmann::json config, StreamRole role)
{
    AdjustSystemClock();

    // Parse trigger section from JSON config
    m_triggerMode    = TriggerMode::INTERNAL;
    m_triggerGpioPin = -1;
    m_gpioChipPath   = "/dev/gpiochip0";
    m_maxFrameAgeNs  = 500'000'000ULL;

    if (config.contains("trigger")) {
        const auto &tc = config["trigger"];
        const std::string mode = tc.value("mode", "INTERNAL");
        if      (mode == "NEAREST_FRAME")    m_triggerMode = TriggerMode::NEAREST_FRAME;
        else if (mode == "HARDWARE_TRIGGER") m_triggerMode = TriggerMode::HARDWARE_TRIGGER;
        else if (mode == "XVS_HARD")         m_triggerMode = TriggerMode::XVS_HARD;
        if (tc.contains("gpioPin"))       m_triggerGpioPin = tc["gpioPin"].get<int>();
        if (tc.contains("gpioChip"))      m_gpioChipPath   = tc["gpioChip"].get<std::string>();
        if (tc.contains("maxFrameAgeMs"))
            m_maxFrameAgeNs = uint64_t(tc["maxFrameAgeMs"].get<uint32_t>()) * 1'000'000ULL;
        // edge: "falling" for TRIGGER_LOW signals (IMX296 XTR), "rising" for PPS_HIGH [default]
        const std::string edge = tc.value("edge", "rising");
        m_triggerEdgeFalling = (edge == "falling");
    }

    const char* modeNames[] = {"INTERNAL", "NEAREST_FRAME", "HARDWARE_TRIGGER", "XVS_HARD"};
    std::cout << "[Camera] TriggerMode=" << modeNames[int(m_triggerMode)];
    if (m_triggerMode != TriggerMode::INTERNAL)
        std::cout << " GPIO=" << m_triggerGpioPin
                  << " edge=" << (m_triggerEdgeFalling ? "falling" : "rising")
                  << " maxFrameAge=" << m_maxFrameAgeNs/1'000'000 << "ms";
    std::cout << "\n";

    // Camera device
    m_cm = std::make_unique<CameraManager>();
    m_cm->start();
    auto cameras = m_cm->cameras();
    if (cameras.empty() || camNo >= int(cameras.size())) {
        std::cerr << "Camera " << camNo << " not available.\n";
        m_cm->stop(); return false;
    }
    m_camera = m_cm->get(cameras[camNo]->id());
    m_camera->acquire();
    std::cout << "Using camera: " << m_camera->id() << "\n";

    // Stream configuration
    m_config = m_camera->generateConfiguration({role});
    StreamConfiguration &sc = m_config->at(0);
    sc.pixelFormat = libcamera::formats::RGB888;
    m_config->validate();
    std::cout << "Stream config: " << sc.toString() << "\n";
    m_camera->configure(m_config.get());

    // Buffer allocation + mmap
    m_allocator = std::make_unique<libcamera::FrameBufferAllocator>(m_camera);
    for (StreamConfiguration &cfg : *m_config) {
        m_allocator->allocate(cfg.stream());
        for (const auto &buf : m_allocator->buffers(cfg.stream())) {
            size_t total = 0;
            for (unsigned i = 0; i < buf->planes().size(); i++) {
                const auto &plane = buf->planes()[i];
                total += plane.length;
                bool lastOrNewFd = (i == buf->planes().size()-1 ||
                    plane.fd.get() != buf->planes()[i+1].fd.get());
                if (lastOrNewFd) {
                    void *mem = mmap(nullptr, total, PROT_READ|PROT_WRITE,
                                     MAP_SHARED, plane.fd.get(), 0);
                    m_mapped_buffers[buf.get()].push_back(
                        libcamera::Span<uint8_t>(static_cast<uint8_t*>(mem), total));
                    total = 0;
                }
            }
        }
    }

    m_stream = sc.stream();
    requests = CreateRequests();
    m_camera->requestCompleted.connect(this, &LibCameraWrapper::requestComplete);
    m_controlsInfo = m_camera->controls();

    // Apply user controls from JSON
    if (config.contains("picamera")) {
        for (const auto &[key, val] : config["picamera"].items()) {
            if (key.front() == '_' || val.is_null()) continue;
            try {
                if      (val.is_boolean())       setControlNumeric(key, val.get<bool>());
                else if (val.is_number_float())   setControlNumeric(key, val.get<float>());
                else if (val.is_number_integer()) setControlNumeric(key, val.get<int32_t>());
            } catch (const std::exception &e) {
                std::cerr << "Failed to set " << key << ": " << e.what() << "\n";
            }
        }
    }
    if (config.contains("rateMs"))
        m_rateMs = config["rateMs"].get<uint32_t>();

    // XVS_HARD: put sensor into external-sync / slave mode.
    //
    // This uses the draft FrameSync control (id 0x009e0021). Setting it to 1
    // tells the IPA and the kernel sensor driver to wait for an external XVS
    // pulse on the sensor's sync input pin before starting each frame.
    //
    // Physical connection (IMX477 HQ Camera):
    //   RPi GPIO 17 (PPS, 3.3V) --> resistor divider --> XVS pad on FPC
    //   The XVS pad is pin 11 on the 22-pin flat cable, labelled "SYNC" on
    //   some carrier boards. Check the schematic of your specific board.
    //   Divider: 3.3 kOhm (top) + 1.8 kOhm (bottom) gives ~1.8 V at XVS.
    if (m_triggerMode == TriggerMode::XVS_HARD) {
        constexpr uint32_t FRAME_SYNC_ID = 0x009e0021; // draft::FrameSync
        m_controlList.set(FRAME_SYNC_ID, static_cast<int32_t>(1));
        std::cout << "[Camera] XVS_HARD: FrameSync=1 (slave mode). "
                     "Ensure XVS wire is connected.\n";
    }

    // Init GPIO PPS watch for NEAREST_FRAME, HARDWARE_TRIGGER and XVS_HARD
    if (m_triggerMode != TriggerMode::INTERNAL) {
        bool gpioOk = false;
        if (m_triggerGpioPin < 0) {
            std::cerr << "[PPS] No gpioPin configured. Falling back to INTERNAL.\n";
        } else {
            m_ppsChip = gpiod_chip_open(m_gpioChipPath.c_str());
            if (!m_ppsChip) {
                std::cerr << "[PPS] Cannot open " << m_gpioChipPath
                          << ": " << strerror(errno) << ". Falling back to INTERNAL.\n";
            } else {
                m_ppsLine = gpiod_chip_get_line(m_ppsChip, m_triggerGpioPin);
                if (!m_ppsLine) {
                    std::cerr << "[PPS] Cannot get GPIO line " << m_triggerGpioPin
                              << ". Falling back to INTERNAL.\n";
                    releasePpsGpio();
                } else {
                    // Choose edge direction from config:
                    //   "rising"  (default) for PPS_HIGH signals (IMX219, fork of LiDAR PPS)
                    //   "falling"           for TRIGGER_LOW signals (IMX296 XTR Trig+)
                    int edgeRet;
                    if (m_triggerEdgeFalling) {
                        edgeRet = gpiod_line_request_falling_edge_events(
                                      m_ppsLine, "mandeye_cam_pps");
                        std::cout << "[PPS] Listening for FALLING edge on GPIO "
                                  << m_triggerGpioPin << "\n";
                    } else {
                        edgeRet = gpiod_line_request_rising_edge_events(
                                      m_ppsLine, "mandeye_cam_pps");
                        std::cout << "[PPS] Listening for RISING edge on GPIO "
                                  << m_triggerGpioPin << "\n";
                    }
                    if (edgeRet < 0) {
                    std::cerr << "[PPS] Cannot request edge events: "
                              << strerror(errno) << ". Falling back to INTERNAL.\n";
                    releasePpsGpio();
                    } else {
                        gpioOk = true;
                    }
                }
            }
        }
        if (!gpioOk) m_triggerMode = TriggerMode::INTERNAL;
    }

    m_running.store(true);
    m_camera->start(&m_controlList);

    if (m_triggerMode != TriggerMode::INTERNAL) {
        m_ppsPending.store(false);
        m_ppsThread = std::thread(&LibCameraWrapper::ppsWatchThread, this);
    }

    return true;
}

// -- stop() -------------------------------------------------------------------

void LibCameraWrapper::stop()
{
    m_running.store(false);
    // Join PPS thread first so it doesn't race with camera teardown
    if (m_ppsThread.joinable()) m_ppsThread.join();
    releasePpsGpio();

    std::this_thread::sleep_for(100ms);
    m_stream = nullptr;

    if (m_camera) {
        m_camera->stop();
        if (m_allocator) m_allocator->free(m_stream);
        requests.clear();
        m_camera->release();
        m_camera.reset();
        m_allocator.reset();
    }
    m_cm->stop();
    m_cm.reset();
}

// -- requestComplete() --------------------------------------------------------
//
// Called by the libcamera internal thread each time a frame is ready.
//
// INTERNAL mode
//   Identical to the original: forward to callback no faster than rateMs.
//
// NEAREST_FRAME mode
//   The camera streams continuously (all frames arrive here, at the free-
//   running rate). Decision logic:
//     1. Convert sensor CLOCK_MONOTONIC timestamp to UTC using m_monoOffset.
//     2. If m_ppsPending is set:
//          delta = |frame_utc - pps_utc|
//          if delta < m_maxFrameAgeNs: accept, tag with PPS timestamp, clear flag.
//          else: drop (re-queue silently) and wait for a closer frame.
//     3. If m_ppsPending is not set: drop silently.
//   At 10 fps (100 ms period) a frame arrives on average 50 ms from the PPS
//   edge, so maxFrameAgeMs=60 is a comfortable margin.
//
// XVS_HARD mode
//   Every frame was started by a hardware XVS edge, so every frame is accepted.
//   We tag it with the last recorded PPS UTC timestamp.
//
void LibCameraWrapper::requestComplete(Request *request)
{
    AdjustSystemClock();

    if (request->status() == Request::RequestCancelled) {
        // Re-queue even on cancel so the buffer pool stays healthy
        request->reuse(libcamera::Request::ReuseBuffers);
        if (!m_oneFrame) m_camera->queueRequest(request);
        return;
    }

    // Get the sensor hardware timestamp of the first (and only) buffer plane.
    // This is CLOCK_MONOTONIC in nanoseconds; convert to UTC.
    uint64_t frameUtcNs = 0;
    for (auto &bp : request->buffers()) {
        frameUtcNs = bp.second->metadata().timestamp + m_monoOffset;
        break;
    }

    // -- decide whether to keep this frame ------------------------------------
    bool     keepFrame        = false;
    uint64_t reportTimestampNs = frameUtcNs;

    switch (m_triggerMode) {

    case TriggerMode::INTERNAL: {
        const auto now = std::chrono::steady_clock::now();
        const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - m_FrameStart).count();
        if (age > long(m_rateMs)) {
            m_FrameStart      = now;
            keepFrame         = true;
            reportTimestampNs = frameUtcNs;
        }
        break;
    }

    case TriggerMode::NEAREST_FRAME: {
        if (m_ppsPending.load(std::memory_order_acquire)) {
            const uint64_t ppsNs = m_lastPpsNs.load(std::memory_order_acquire);
            // Signed delta: positive means frame arrived after the PPS edge (normal).
            // Negative means the frame started just before the edge (also acceptable).
            const int64_t delta = int64_t(frameUtcNs) - int64_t(ppsNs);
            if (std::abs(delta) < int64_t(m_maxFrameAgeNs)) {
                keepFrame         = true;
                reportTimestampNs = ppsNs;   // align to the second boundary
                m_ppsPending.store(false, std::memory_order_release);
                std::cout << "[Camera] NEAREST_FRAME accepted, delta="
                          << delta / 1000 << " us\n";
            }
        }
        break;
    }

    case TriggerMode::HARDWARE_TRIGGER:
        // The IMX296 sensor only produces a frame when it receives a hardware
        // trigger pulse on XTR (Trig+). We accept only the FIRST frame after
        // each trigger edge (m_ppsPending is set by ppsWatchThread on each edge
        // and cleared here after the first accepted frame).
        // This discards any extra frames that libcamera delivers from buffered
        // requests before the next trigger edge arrives.
        if (m_ppsPending.load(std::memory_order_acquire)) {
            reportTimestampNs = m_lastPpsNs.load(std::memory_order_acquire);
            keepFrame         = true;
            m_ppsPending.store(false, std::memory_order_release);
            std::cout << "[Camera] HARDWARE_TRIGGER accepted, trigger_utc="
                      << reportTimestampNs << " frame_utc=" << frameUtcNs
                      << " delta=" << (int64_t(frameUtcNs) - int64_t(reportTimestampNs)) / 1000
                      << " us\n";
        }
        break;

    case TriggerMode::XVS_HARD:
        // Sensor only produces a frame when it receives XVS, so every frame is valid.
        keepFrame         = true;
        reportTimestampNs = m_lastPpsNs.load(std::memory_order_acquire);
        break;
    }

    // -- decode and forward to callback ---------------------------------------
    if (keepFrame) {
        nlohmann::json meta;
        for (auto f : request->metadata()) {
            const auto id = f.first;
            std::string name;
            if (name = controlToString(id); name.empty()) name = std::to_string(id);
            meta[name] = f.second.toString();
        }
        // Diagnostics added to .meta.json
        meta["_triggerMode"]       = int(m_triggerMode);
        meta["_frameUtcNs"]        = frameUtcNs;
        meta["_reportTimestampNs"] = reportTimestampNs;
        if (m_triggerMode != TriggerMode::INTERNAL)
            meta["_ppsUtcNs"]      = m_lastPpsNs.load();

        m_requestTimestamp = reportTimestampNs;

        const auto &sc = m_config->at(0);
        if (sc.pixelFormat == libcamera::formats::MJPEG) {
            for (auto &bp : request->buffers()) {
                auto mem = Mmap(bp.second);
                std::vector<uchar> memv(mem[0].begin(), mem[0].end());
                cv::Mat img = cv::imdecode(memv, cv::IMREAD_COLOR);
                if (m_callback) m_callback(img, reportTimestampNs, meta);
            }
        } else if (sc.pixelFormat == libcamera::formats::RGB888) {
            const unsigned vw = sc.size.width, vh = sc.size.height, vs = sc.stride;
            for (auto &bp : request->buffers()) {
                auto mem = Mmap(bp.second);
                cv::Mat img(vh, vw, CV_8UC3);
                uint8_t *ptr = mem[0].data();
                for (unsigned i = 0; i < vh; i++, ptr += vs)
                    memcpy(img.ptr(i), ptr, vw * 3);
                if (m_callback) m_callback(img, reportTimestampNs, meta);
            }
        } else {
            std::cout << "Unsupported pixel format\n";
        }
    }

    // Always re-queue: the camera must keep streaming to receive the next PPS frame
    request->reuse(libcamera::Request::ReuseBuffers);
    if (!m_oneFrame) m_camera->queueRequest(request);
}

// -- capture() ----------------------------------------------------------------

void LibCameraWrapper::capture(bool oneFrame)
{
    m_FrameStart = std::chrono::steady_clock::now();
    m_oneFrame   = oneFrame;
    m_ppsPending.store(false);
    for (auto &req : requests) {
        m_requestTimestamp = getCurrentTimestamp();
        m_camera->queueRequest(req.get());
    }
}

// -- getCameraConfig() --------------------------------------------------------

static nlohmann::json reportValue(const libcamera::ControlValue &v,
                                  const libcamera::ControlType   type) {
    if (v.isNone()) return {};
    switch (type) {
        case libcamera::ControlTypeBool:       return v.get<bool>();
        case libcamera::ControlTypeByte:       return v.get<uint8_t>();
        case libcamera::ControlTypeUnsigned16: return v.get<uint16_t>();
        case libcamera::ControlTypeUnsigned32: return v.get<uint32_t>();
        case libcamera::ControlTypeInteger32:  return v.get<int32_t>();
        case libcamera::ControlTypeInteger64:  return v.get<int64_t>();
        case libcamera::ControlTypeFloat:      return v.get<float>();
        case libcamera::ControlTypeString:     return v.get<std::string>();
        default:                               return {};
    }
}

nlohmann::json LibCameraWrapper::getCameraConfig() {
    nlohmann::json cfg;
    if (!m_camera) return cfg;
    cfg["id"]     = m_camera->id();
    cfg["rateMs"] = m_rateMs;

    // Trigger section is self-documenting in the generated config file
    cfg["trigger"]["_help_mode"]   =
        "INTERNAL = free-running rate-limited (original). "
        "NEAREST_FRAME = PPS GPIO selects closest frame (any camera model). "
        "XVS_HARD = hardware XVS sync (IMX477 HQ / IMX296 GS only, "
        "needs dtoverlay=imx477,sync in /boot/firmware/config.txt).";
    cfg["trigger"]["mode"]          = "INTERNAL";
    cfg["trigger"]["gpioPin"]       = -1;
    cfg["trigger"]["gpioChip"]      = "/dev/gpiochip0";
    cfg["trigger"]["_help_maxAge"]  =
        "NEAREST_FRAME only: max milliseconds between PPS edge and frame "
        "timestamp. Set to slightly more than half the frame period. "
        "At 10 fps (100 ms period) use 60. At 30 fps use 20.";
    cfg["trigger"]["maxFrameAgeMs"] = 500;

    cfg["picamera"]["_Note"] = "Adjust camera parameters here";

    for (auto const &ctrl : m_controlsInfo) {
        if (ctrl.first->isArray()) continue;
        auto name = ctrl.first->name();
        if (name.empty()) continue;
        const unsigned int id   = ctrl.first->id();
        const auto         type = ctrl.first->type();

        cfg["controls_info"][name]["id"]       = id;
        cfg["controls_info"][name]["min"]      = ctrl.second.min().toString();
        cfg["controls_info"][name]["max"]      = ctrl.second.max().toString();
        cfg["controls_info"][name]["def"]      = ctrl.second.def().toString();
        cfg["controls_info"][name]["isInput"]  = ctrl.first->isInput();
        cfg["controls_info"][name]["isOutput"] = ctrl.first->isOutput();
        if (LibCameraControlTypeToString.count(type))
            cfg["controls_info"][name]["type_str"] = LibCameraControlTypeToString.at(type);

        cfg["picamera"][name] = m_controlList.contains(id)
            ? reportValue(m_controlList.get(id), type)
            : reportValue(ctrl.second.def(), type);
    }
    return cfg;
}

} // namespace mandeye
