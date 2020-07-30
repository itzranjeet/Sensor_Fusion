#include <iostream>
#include "logger_base.h"
#include "console_logger.h"

namespace sensor_fusion
{

    template<typename... Args>
    auto& ConsoleLogger::GetTypeLookupMap(Args&&... args)
    {
        static log_function_lookup_based_on_type type_lookup_;
        type_lookup_[LogMessageType :: error] = [&args...](){
                                                            std::cout << "STD_ERROR: ";
                                                            printf(args...);
                                                            std::cout << std::endl;
                                                 };
        
        type_lookup_[LogMessageType :: debug] = [&args...]() {
                                                            std::cout << "STD_DEBUG: ";
                                                            printf(args...);
                                                            std::cout << std::endl;
                                                 };

        type_lookup_[LogMessageType :: warn] = [&args...]() {
                                                            std::cout << "STD_WARNING: ";
                                                            printf(args...);
                                                            std::cout << std::endl;
                                                 };

        type_lookup_[LogMessageType :: info] = [&args...](){
                                                            std::cout << "STD_INFO: ";
                                                            printf(args...);
                                                            std::cout << std::endl;
                                                 };
        return type_lookup_;
    }

} //namespace sensor_fusion
