#ifndef SENSOR_FUSION_COMPONENT_FACTORY_H
#define SENSOR_FUSION_COMPONENT_FACTORY_H

#include <memory>

namespace sensor_fusion
{

    class AbstractComponentBase
    {
        public:
            virtual ~AbstractComponentBase() { }
    };

    class ComponentFactory
    {
        public:
            virtual std::shared_ptr<AbstractComponentBase> CreateLoggerBase() = 0;
            virtual std::shared_ptr<AbstractComponentBase> CreateLoggerStream() = 0;
    };

} //namespace sensor_fusion

#endif //SENSOR_FUSION_COMPONENT_FACTORY_H
