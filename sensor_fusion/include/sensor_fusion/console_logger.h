#ifndef SENSOR_FUSION_CONSOLE_LOGGER_H
#define SENSOR_FUSION_CONSOLE_LOGGER_H

#include "constants.h"
#include "logger_base.h"

namespace sensor_fusion
{

    class ConsoleLogger : public LoggerBase<ConsoleLogger>
    {
        public:
            ConsoleLogger()
            { }

            template<typename... Args>
            static auto& GetTypeLookupMap(Args&&... args);
    };

} //namespace sensor_fusion

#include "console_logger.hpp"

#endif //SENSOR_FUSION_CONSOLE_LOGGER_H

