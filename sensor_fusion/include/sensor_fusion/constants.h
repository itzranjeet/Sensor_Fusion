#ifndef SENSOR_FUSION_CONSTANTS_H
#define SENSOR_FUSION_CONSTANTS_H

#include <stdint.h>
#include <unordered_map>
#include <functional>

namespace sensor_fusion
{

    static constexpr unsigned sleepDelay{1};
    static constexpr char kSensorImageMessageTopic[]{"/sensor_fusion/image"};
    static constexpr char kSensorRadarMessageTopic[]{"/sensor_fusion/radar"};
    static constexpr char kSensorLidarMessageTopic[]{"/sensor_fusion/lidar"};
    static constexpr char kSensorCameraMessageTopic[]{"/sensor_fusion/camera"};
    static constexpr char kSensorShortRangeMessageTopic[]{"/sensor_fusion/srr"};
    static constexpr char kSensorLongRangeMessageTopic[]{"/sensor_fusion/lrr"};
    static constexpr char kOriginalSensorMessageTopic[]{"/sensor_fusion/original"};
    static constexpr char kOutputMessageTopic[]{"/sensor_fusion/output"};
    static constexpr char kVisualizationMessageTopic[]{"/sensor_fusion/visualization"};

    static constexpr  char kEmittedTickTopic[]{"/sensor_fusion/csv_clocktick"};
    static constexpr  char kConsumedTickTopic[]{"/sensor_fusion/csv_clocktick_consumed"};
    static constexpr  char kResetClockTopic[]{"/sensor_fusion/csv_clock_reset"};
    static constexpr  char kRegisterTickTopic[]{"/sensor_fusion/csv_clocktick_register"};
    static constexpr double kNowaitTimeout{0.1};

    static constexpr unsigned kObjectIdToken{0};
    static constexpr unsigned kXCoordinateToken{1};
    static constexpr unsigned kYCoordinateToken{2};
    static constexpr unsigned kTimestampToken{3};
    static constexpr unsigned kValidTokenCount{4};

    static constexpr uint32_t kSensorMessageQueueSize{10};
    static constexpr uint32_t kOutputMessageQueueSize{10};
    static constexpr uint32_t kOriginalMessageQueueSize{10};

    static constexpr int32_t kTrackLost{3};
    static constexpr int32_t kTrack{0};
    static constexpr int32_t kInitLost{0};
    static constexpr int32_t kInitTracked{2};
    static constexpr int32_t kPostInit{0};
    static constexpr int32_t kPost{0};
    static constexpr int32_t kPostLost{4};

    static constexpr int32_t kTolerenceX{100};
    static constexpr int32_t kTolerenceY{100};
    static constexpr int32_t kTolerenceD{100};

    static constexpr int clock_queue_size{1000};
    static constexpr int publisher_queue_size{1};
    enum class LogMessageType
    {
        error,
        debug,
        warn,
        info,
        error_stream,
        debug_stream,
        warn_stream,
        info_stream
    };

    using log_function_lookup_based_on_type = std::unordered_map<LogMessageType, std::function<void()>>;

} //namespace sensor_fusion

namespace visualizer
{

    static char kHeaderMarkerFrameId[]{"Observer"};
    static constexpr char kOriginNamespace[]{"Origin"};
    static constexpr char kObjectLabel[]{"Object "};
    static constexpr char kObjectPathNamespace[]{"Object path "};

    static constexpr int32_t kNodeBufferSize{10};
    static constexpr int32_t kMarkersQueueSize{50};
    static constexpr int32_t kOriginCylinderId{0};
    static constexpr int8_t kMarkerColorsAmount{6};

    static constexpr double kPrimitiveLifeTime{5.0};  // seconds
    static constexpr double kPrimitiveZScale{0.1};
    static constexpr double kPrimitiveZPosition{0.0};
    static constexpr double kOriginScale{10.0};
    static constexpr double kOriginAlpha{0.35};
    static constexpr double kFilteredPointScale{0.4};
    static constexpr double kFilteredPointAlpha{1.0};
    static constexpr double kSensorPointScale{0.1};
    static constexpr double kSensorPointAlpha{1.0};
    static constexpr double kLineWidth{0.05};
    static constexpr double kLineAlpha{0.7};

} // namespace visualizer

#endif // SENSOR_FUSION_CONSTANTS_H
