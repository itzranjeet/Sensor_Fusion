#ifndef SENSOR_FUSION_ROS_LOGGER_STREAM_H
#define SENSOR_FUSION_ROS_LOGGER_STREAM_H

#include "constants.h"
#include "logger_stream_base.h"

namespace sensor_fusion
{

    class ROSLoggerStream : public LoggerStreamBase<ROSLoggerStream>
    {
        public:
            static auto& GetTypeLookupMap(const LoggerStreamBase&);
    };

} //namespace sensor_fusion

#include "ros_logger_stream.hpp"

#endif //SENSOR_FUSION_ROS_LOGGER_STREAM_H
