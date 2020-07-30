#include "logger_stream_base.h"

#define LOGSTREAM(instance, arg, args) \
        LogStream(arg, (*instance) << args)

namespace sensor_fusion
{
    template<typename Derived, typename Base>
    template<typename arg>
    LoggerStreamBase<Derived, Base>& LoggerStreamBase<Derived, Base> :: operator << (const arg& arg_passed)
    {
        list_of_args_ << arg_passed;
        return *this;
    }

    template<typename Derived, typename Base>
    void LoggerStreamBase<Derived, Base> :: LogStream(LogMessageType type, LoggerStreamBase& logger_obj)
    {
        Derived :: GetTypeLookupMap(logger_obj)[type]();
        list_of_args_.str(std::string());
    }
}
