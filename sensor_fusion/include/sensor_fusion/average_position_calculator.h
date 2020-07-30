#ifndef SENSOR_FUSION_DATA_FUSION_H
#define SENSOR_FUSION_DATA_FUSION_H

#include <queue>

#include <sensor_fusion/SensorMessage.h>

namespace sensor_fusion
{

    class AveragePositionCalculator
    {
    public:
        AveragePositionCalculator() = default;
        virtual ~AveragePositionCalculator() = default;
        void UpdateAverage(const SensorMessage& msg);
        const SensorMessage& GetAverageMessage() const;
        void Reset();
    private:
        SensorMessage average_message_;
        std::vector<SensorMessage> measured_messages_;
    };

}//namespace sensor_fusion

#endif // SENSOR_FUSION_DATA_FUSION_H
