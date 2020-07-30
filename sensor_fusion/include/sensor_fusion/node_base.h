/// @copyright Copyright (C) 2019, KPIT
/// @brief SHARED CSV NODE FUNCTIONALITY

#ifndef SENSOR_FUSION_NODE_BASE_H
#define SENSOR_FUSION_NODE_BASE_H
#include "constants.h"
#include <sensor_fusion/SensorMessage.h>
#include "sensor_fusion/SensorMessageArray.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "platform_interface.h"
#include <string>

namespace sensor_fusion
{

    class NodeBase
    {
    public:
        NodeBase() = default;
        NodeBase (const std::string& nodename);
        virtual ~NodeBase() = default;
        virtual void Run(int &argc, char **argv) = 0;
        virtual void RegisterPublishers() const = 0;
        virtual void RegisterSubscribers() = 0;

    protected:
        platform_communicator::PlatformInterface platform_interface_;
        const std::string node_name_;

    };

}//namespace sensor_fusion`

#endif //SENSOR_FUSION_NODE_BASE_H
