#ifndef SENSOR_FUSION_ROS_LOGGER_H
#define SENSOR_FUSION_ROS_LOGGER_H

#include "constants.h"
#include "logger_base.h"

namespace sensor_fusion
{

    class ROSLogger : public LoggerBase<ROSLogger>
    {
        public:
            ROSLogger()
            { }

            template<typename... Args>
            static auto& GetTypeLookupMap(Args&&... args);
    };

} //namespace sensor_fusion

#include "ros_logger.hpp"

#endif //SENSOR_FUSION_ROS_LOGGER_H

