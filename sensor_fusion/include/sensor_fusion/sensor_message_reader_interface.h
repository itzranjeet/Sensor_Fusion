#ifndef SENSOR_FUSION_I_CSV_CONVERTER_H
#define SENSOR_FUSION_I_CSV_CONVERTER_H

#include <string>
#include <vector>

namespace sensor_fusion
{

    template <typename returnType>
    class SensorMessageReaderInterface
    {
    public:
        virtual std::vector<returnType>& ReadMessage(const char* BagPath,std::string TopicName) = 0;
    };

} //namespace sensor_fusion

#endif // SENSOR_FUSION_I_CSV_CONVERTER_H
