#ifndef SENSOR_MESSAGE_FROM_RADAR_ROSBAG
#define SENSOR_MESSAGE_FROM_RADAR_ROSBAG
#include <sensor_msgs/Image.h>
#include <sensor_fusion/SensorMessage.h>
#include "sensor_message_reader_interface.h"
#include "constants.h"
#include <fstream>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h> 
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
namespace sensor_fusion
{

    class SensorMessageFromRadarRosbag : public SensorMessageReaderInterface<SensorMessage>
    {
    public:
        std::vector<SensorMessage>& ReadMessage(const char* BagPath,std::string TopicName) override;
    private:
        std::vector<SensorMessage> sensor_message_vec;
                rosbag::Bag bag;
    };

}//namespace sensor_fusion

#endif //SENSOR_MESSAGE_FROM_RADAR_ROSBAG
