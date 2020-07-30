#ifndef SENSOR_MESSAGE_FROM_LIDAR_ROSBAG
#define SENSOR_MESSAGE_FROM_LIDAR_ROSBAG
#include <sensor_msgs/PointCloud.h>
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
    class SensorMessageFromLidarRosbag : public SensorMessageReaderInterface<sensor_msgs::PointCloud>
    {
    public:
        std::vector<sensor_msgs::PointCloud>& ReadMessage(const char* BagPath,std::string TopicName) override;
    private:
        std::vector<sensor_msgs::PointCloud> sensor_message_vec;
        rosbag::Bag bag;
    };
}//namespace sensor_fusion

#endif //SENSOR_MESSAGE_FROM_LIDAR_ROSBAG
