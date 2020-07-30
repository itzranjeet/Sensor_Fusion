#include "ros_logger_stream.h"

namespace sensor_fusion
{

    auto& ROSLoggerStream :: GetTypeLookupMap(const LoggerStreamBase& base_instance)
    {
        static log_function_lookup_based_on_type type_lookup_;
        type_lookup_[LogMessageType :: error] = [&base_instance](){
                                                            ROS_ERROR_STREAM(base_instance.list_of_args_.str());
                                                 };
        
        type_lookup_[LogMessageType :: debug] = [&base_instance]() {
                                                            ROS_DEBUG_STREAM(base_instance.list_of_args_.str());
                                                 };

        type_lookup_[LogMessageType :: warn] = [&base_instance]() {
                                                            ROS_WARN_STREAM(base_instance.list_of_args_.str());
                                                 };

        type_lookup_[LogMessageType :: info] = [&base_instance](){
                                                            ROS_INFO_STREAM(base_instance.list_of_args_.str());
                                                 };
        return type_lookup_;
    }

} //namespace sensor_fusion
