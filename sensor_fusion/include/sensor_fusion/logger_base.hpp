#include "logger_base.h"
#include "ros_logger.h"
#include "console_logger.h"

namespace sensor_fusion
{

    template<typename Derived, typename Base>
    template<typename... Args>
    void LoggerBase<Derived, Base> :: LogMessage(LogMessageType log_msg_type, Args&&... args)
    {
        Derived :: GetTypeLookupMap(args...)[log_msg_type]();
    }

}
