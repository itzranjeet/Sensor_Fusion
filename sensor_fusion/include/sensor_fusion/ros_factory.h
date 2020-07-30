#ifndef SENSOR_FUSION_ROS_FACTORY_H
#define SENSOR_FUSION_ROS_FACTORY_H

#include "component_factory.h"
#include "ros_logger.h"
#include "ros_logger_stream.h"
#include <memory>

namespace sensor_fusion
{

    class ROSFactory : public ComponentFactory
    {
        public:
            virtual std::shared_ptr<AbstractComponentBase> CreateLoggerBase() override
            {
                static std::shared_ptr<ROSLogger> logger = std::make_shared<ROSLogger>();
                return logger;
            }

            virtual std::shared_ptr<AbstractComponentBase> CreateLoggerStream() override
            {
                static std::shared_ptr<ROSLoggerStream> logger_stream = std::make_shared<ROSLoggerStream>();
                return logger_stream;
            }
    };

} //namespace sensor_fusion

#endif //SENSOR_FUSION_ROS_FACTORY_H
