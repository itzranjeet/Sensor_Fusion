/*
 * The actual component instantiation knowledge must be confined to this 
 * factory class. Currently hardcoded to instantiate ROSFactory.
 * Each client must route its request to this class to obtain the correct instantiation
 * of a component.
 */


#ifndef SENSOR_FUSION_FACTORY_INIT_H
#define SENSOR_FUSION_FACTORY_INIT_H

#include <memory>
#include <mutex>
#include "component_factory.h"
#include "ros_factory.h"
#include "constants.h"

namespace sensor_fusion
{

    class ComponentFactory;
    class ROSFactory;
    class ROSLogger;
    class ROSLoggerStream;

    class FactoryInit
    {
        public:
            static void Init()
            {
                std::call_once(init_done,[&]() {
                        factory_ = std::make_shared<ROSFactory>();
                    });
            }

            static auto GetLoggerInstance()
            {
                FactoryInit::Init();
                return std::dynamic_pointer_cast<ROSLogger>(factory_->CreateLoggerBase());
            }

            static auto GetLogStreamerInstance()
            {
                FactoryInit::Init();
                return std::dynamic_pointer_cast<ROSLoggerStream>(factory_->CreateLoggerStream());
            }

        private:
            static std::shared_ptr<ComponentFactory> factory_;
            static std::once_flag init_done; 
            FactoryInit() = delete;
            ~FactoryInit() = delete;
            FactoryInit(const FactoryInit&) = delete;
            FactoryInit& operator=(const FactoryInit&) = delete;
            FactoryInit(FactoryInit&&) = delete;
            FactoryInit& operator=(FactoryInit&&) = delete;
    };

    std::shared_ptr<ComponentFactory> FactoryInit :: factory_;
    std::once_flag FactoryInit :: init_done;

} //namespace sensor_fusion

#endif //SENSOR_FUSION_FACTORY_INIT_H
