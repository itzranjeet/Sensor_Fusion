#ifndef SENSOR_MESSAGE_FROM_IMAGE_ROSBAG
#define SENSOR_MESSAGE_FROM_IMAGE_ROSBAG
#include <sensor_msgs/Image.h>
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

    class SensorMessageFromImageRosbag : public SensorMessageReaderInterface<sensor_msgs::Image>
    {
    public:
        std::vector<sensor_msgs::Image>& ReadMessage(const char* BagPath,std::string TopicName) override;
    private:
        std::vector<sensor_msgs::Image> sensor_message_vec;
                rosbag::Bag bag;
    };

}//namespace sensor_fusion

#endif //SENSOR_MESSAGE_FROM_IMAGE_ROSBAG
