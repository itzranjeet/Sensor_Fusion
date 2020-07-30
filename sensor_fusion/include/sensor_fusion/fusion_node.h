#ifndef SENSOR_FUSION_FUSION_NODE_H
#define SENSOR_FUSION_FUSION_NODE_H
#include <sensor_fusion/SensorMessage.h>
#include "node_base.h"

namespace sensor_fusion
{

    class FusionNode : public NodeBase
    {
    public:
        FusionNode() = default;
        FusionNode(const std::string& nodename);
        virtual ~FusionNode() = default;
        template<const uint32_t sensor_id>
        void HandleMessageCallback(const sensor_fusion::SensorMessageArrayConstPtr& messages);
        virtual void RegisterPublishers() const override;
        virtual void RegisterSubscribers() override;
        virtual void Run(int &argc, char **argv) override;

    protected:
        sensor_fusion::FusionImplementation sensorfusion_;

    };

}//namespace sensor_fusion

#endif // SENSOR_FUSION_FUSION_NODE_H
