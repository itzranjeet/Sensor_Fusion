#ifndef SENSOR_FUSION_LOGGER_BASE_H
#define SENSOR_FUSION_LOGGER_BASE_H

#include "constants.h"
#include "component_factory.h"

namespace sensor_fusion
{

    template<typename Derived, typename Base = AbstractComponentBase>
    class LoggerBase : public Base
    {
        public:
            template<typename... Args>
            static void LogMessage(LogMessageType log_msg_type, Args&&... args);
            friend Derived;
        private:
            LoggerBase() { };
    };

} //namespace sensor_fusion

#include "logger_base.hpp"

#endif //SENSOR_FUSION_LOGGER_BASE_H
