#include "sensor_message_from_radar_rosbag.h"
#include <boost/tokenizer.hpp>
#include "Utility.h"

namespace sensor_fusion
{
    std::vector<SensorMessage>& SensorMessageFromRadarRosbag::ReadMessage(const char* BagPath,std::string TopicName)
    {
        bag.open(BagPath, rosbag::bagmode::Read);
        std::cout<<"Entered in Radar Rosbag:"<<std::endl;
        ros::Rate loop_rate(5);
        std::vector<std::string> topics;      
        topics.push_back(std::string(TopicName));
        rosbag::View view(bag, rosbag::TopicQuery(topics));
                std::cout<<"radar....."<<"";
        foreach(rosbag::MessageInstance const m, view)
       {
                std::cout<<"radar1"<<"";
        sensor_fusion::SensorMessage::ConstPtr radar_msg = m.instantiate<sensor_fusion::SensorMessage>();
        std::cout<<"radar2"<<"";
        if (radar_msg != NULL){
			auto message = *radar_msg;
			std::cout<<radar_msg<<std::endl;
			sensor_message_vec.push_back(message);
			    }
       }
   bag.close();

//for (int i = 0; i < sensor_message_vec.size(); i++) {
//            std::cout << sensor_message_vec[i] << " ";
  //      }
   return sensor_message_vec;
  }
       
}//namespace sensor_fusion
