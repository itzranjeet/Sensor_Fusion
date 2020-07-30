#ifndef SENSOR_FUSION_LOGGER_STREAM_BASE_H
#define SENSOR_FUSION_LOGGER_STREAM_BASE_H

#include "constants.h"
#include "component_factory.h"

namespace sensor_fusion
{

    template<typename Derived, typename Base = AbstractComponentBase>
    class LoggerStreamBase : public Base
    {
        public:
            template<typename arg>
            LoggerStreamBase& operator << (const arg& a);

            void LogStream(LogMessageType type, LoggerStreamBase& logger_obj);
            friend Derived;

        private:
            std::stringstream list_of_args_;
    };

} //namespace sensor_fusion

#include "logger_stream_base.hpp"

#endif //SENSOR_FUSION_LOGGER_STREAM_BASE_H
