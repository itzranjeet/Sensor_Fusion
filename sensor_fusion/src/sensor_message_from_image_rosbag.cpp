#include "sensor_message_from_image_rosbag.h"
#include <boost/tokenizer.hpp>
#include "Utility.h"

namespace sensor_fusion
{
    std::vector<sensor_msgs::Image>& SensorMessageFromImageRosbag::ReadMessage(const char* BagPath,std::string TopicName)
    {
        bag.open(BagPath, rosbag::bagmode::Read);
         std::cout<<"Entered in Image Rosbag:"<<std::endl;
        ros::Rate loop_rate(5);
        std::vector<std::string> topics;      
        topics.push_back(std::string(TopicName));
        rosbag::View view(bag, rosbag::TopicQuery(topics));
        foreach(rosbag::MessageInstance const m, view)
       {
         sensor_msgs::ImagePtr msg = m.instantiate<sensor_msgs::Image>();
        if (msg != NULL){
				auto message = *msg;
                                //std::cout << msg <<std::endl;
				sensor_message_vec.push_back(message);
			      }
       }
   bag.close();
   
/*for (int i = 0; i < sensor_message_vec.size(); i++) {
           std::cout << sensor_message_vec[i] << " ";
        }*/
   return sensor_message_vec;
  }
       
}//namespace sensor_fusion
