#pragma once
#include <libcamera/libcamera.h>
#include <opencv2/opencv.hpp>
#include <atomic>
#include <thread>
#include <gpiod.h>
#include "json.hpp"

namespace mandeye
{
    uint64_t getCurrentTimestamp();

    const std::unordered_map<libcamera::ControlType, std::string> LibCameraControlTypeToString = {
        {libcamera::ControlType::ControlTypeNone,      "ControlTypeNone"},
        {libcamera::ControlType::ControlTypeBool,      "ControlTypeBool"},
        {libcamera::ControlType::ControlTypeByte,      "ControlTypeByte"},
        {libcamera::ControlType::ControlTypeInteger32, "ControlTypeInteger32"},
        {libcamera::ControlType::ControlTypeInteger64, "ControlTypeInteger64"},
        {libcamera::ControlType::ControlTypeFloat,     "ControlTypeFloat"},
        {libcamera::ControlType::ControlTypeString,    "ControlTypeString"},
        {libcamera::ControlType::ControlTypeRectangle, "ControlTypeRectangle"},
        {libcamera::ControlType::ControlTypeSize,      "ControlTypeSize"}
    };

    using CaptureCallback = void(cv::Mat& img, uint64_t timestamp, nlohmann::json& metaDataDump);

    /**
     * Trigger mode for camera capture timing.
     *
     * INTERNAL  (default, original behaviour)
     *   Camera streams freely, frames are forwarded to the callback no faster
     *   than once every rateMs milliseconds. No relationship with PPS.
     *
     * NEAREST_FRAME  (any Pi-Camera model)
     *   Camera still runs free at the rate set by FrameDurationLimits.
     *   A dedicated thread (ppsWatchThread) blocks on a GPIO pin waiting for
     *   rising edges from the PPS signal. On each edge it stores the UTC
     *   nanosecond timestamp and sets m_ppsPending = true.
     *   requestComplete() checks every incoming frame:
     *     - Convert the sensor hardware timestamp (CLOCK_MONOTONIC) to UTC.
     *     - If m_ppsPending is set AND |frame_utc - pps_utc| < maxFrameAgeNs:
     *         accept the frame, tag it with the PPS timestamp, clear m_ppsPending.
     *     - Otherwise: silently drop the frame (re-queue it for the next PPS).
     *   Net effect: one photo per PPS pulse, timestamped at the exact second
     *   boundary, aligned with the LiDAR data.
     *   Precision: +/-(frame_period / 2). At 10 fps -> +/-50 ms. At 30 fps -> +/-17 ms.
     *   Wiring: one GPIO pin wired to the same PPS line as the LiDAR (simple fork).
     *
     * XVS_HARD  (IMX477 HQ Camera or IMX296 Global Shutter only)
     *   The sensor is put into slave mode via the libcamera FrameSync draft
     *   control (set automatically in start()). In slave mode the sensor waits
     *   for a rising edge on its physical XVS pin before starting each frame.
     *   The PPS GPIO is wired to the XVS pad on the camera FPC connector.
     *   Precision: <1 us (hardware pixel-clock domain).
     *   Requires: dtoverlay=imx477,sync  in /boot/firmware/config.txt.
     *   Voltage: RPi GPIO = 3.3 V, IMX477 XVS = 1.8 V.
     *            Use a resistor divider (3.3k / 1.8k) or a level shifter.
     *   ppsWatchThread still runs to record the PPS UTC timestamp for metadata.
     */
    enum class TriggerMode {
        INTERNAL,
        NEAREST_FRAME,
        XVS_HARD,
    };

    class LibCameraWrapper
    {
    private:
        void requestComplete(libcamera::Request *request);

        // Thread that blocks on GPIO rising edges and records PPS timestamps
        void ppsWatchThread();

        std::function<CaptureCallback> m_callback;
        std::shared_ptr<libcamera::Camera>               m_camera;
        std::unique_ptr<libcamera::CameraManager>        m_cm;
        std::unique_ptr<libcamera::FrameBufferAllocator> m_allocator;
        std::map<libcamera::FrameBuffer*,
                 std::vector<libcamera::Span<uint8_t>>>  m_mapped_buffers;
        std::unique_ptr<libcamera::CameraConfiguration>  m_config;
        libcamera::Stream                               *m_stream = nullptr;

        std::vector<libcamera::Span<uint8_t>>            Mmap(libcamera::FrameBuffer*);
        std::vector<std::unique_ptr<libcamera::Request>> requests;

        libcamera::ControlInfoMap m_controlsInfo;
        libcamera::ControlList   m_controlList;

        uint64_t m_requestTimestamp = 0;
        std::chrono::time_point<std::chrono::steady_clock> m_FrameStart;
        uint64_t m_monoOffset = 0;
        bool     m_oneFrame   = false;
        std::atomic<bool> m_running{false};

        std::vector<std::unique_ptr<libcamera::Request>> CreateRequests();
        void AdjustSystemClock();

        uint32_t    m_rateMs = 500;    // used only in INTERNAL mode
        std::thread m_captureThread;

        // -- trigger state --------------------------------------------------
        TriggerMode m_triggerMode    {TriggerMode::INTERNAL};
        int         m_triggerGpioPin {-1};
        std::string m_gpioChipPath   {"/dev/gpiochip0"};

        // UTC nanoseconds of the most recent PPS edge.
        // Written by ppsWatchThread, read by requestComplete -> atomic.
        std::atomic<uint64_t> m_lastPpsNs{0};

        // True after each PPS edge, cleared once a frame is accepted.
        std::atomic<bool>     m_ppsPending{false};

        // Max |frame_utc - pps_utc| (ns) for a frame to be accepted.
        // Default = 500 ms (safe for up to ~2 fps free-running rate).
        uint64_t m_maxFrameAgeNs{500'000'000ULL};

        gpiod_chip *m_ppsChip {nullptr};
        gpiod_line *m_ppsLine {nullptr};
        std::thread m_ppsThread;
        void releasePpsGpio();

    public:
        LibCameraWrapper()  = default;
        ~LibCameraWrapper() = default;

        bool start(int camNo,
                   nlohmann::json config = {},
                   libcamera::StreamRole role = libcamera::StreamRole::StillCapture);
        void capture(bool oneFrame = false);
        void stop();
        void registerCallback(std::function<CaptureCallback>&& cb)
             { m_callback = std::move(cb); }
        nlohmann::json getCameraConfig();
        libcamera::ControlList& getControlList() { return m_controlList; }
        template <typename T> bool setControlNumeric(const std::string& name, T value);
    };

} // namespace mandeye
