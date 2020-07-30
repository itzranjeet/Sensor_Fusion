#include <iostream>
#include <sstream>
#include "ros_logger.h"
#include "logger_base.h"

namespace sensor_fusion
{

    template<typename... Args>
    auto& ROSLogger::GetTypeLookupMap(Args&&... args)
    {
        static log_function_lookup_based_on_type type_lookup_;
        type_lookup_[LogMessageType :: error] = [&args...](){
                                                            ROS_ERROR(args...);
                                                 };
        
        type_lookup_[LogMessageType :: debug] = [&args...]() {
                                                            ROS_DEBUG(args...);
                                                 };

        type_lookup_[LogMessageType :: warn] = [&args...]() {
                                                            ROS_WARN(args...);
                                                 };

        type_lookup_[LogMessageType :: info] = [&args...](){
                                                            ROS_INFO(args...);
                                                 };
        return type_lookup_;
    }

} //namespace sensor_fusion
