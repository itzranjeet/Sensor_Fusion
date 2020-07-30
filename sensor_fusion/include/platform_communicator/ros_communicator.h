#ifndef PLATFORM_COMMUNICATOR_ROS_COMMUNICATOR_H
#define PLATFORM_COMMUNICATOR_ROS_COMMUNICATOR_H

#include "communicator.h"
#include <unordered_map>
#include <ros/ros.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <rosbag/view.h> 
#include <boost/foreach.hpp> 
#define foreach BOOST_FOREACH
namespace platform_communicator
{
	class ROSCommunicator : public Communicator<ROSCommunicator> 
	{
	public:
               std::vector<sensor_msgs::Image> Image_vect;
	       //std::vector<sensor_msgs::PointCloud> sensor_lidar_vec;
		void InitNode(int &argc, char **argv, const std::string &name, uint32_t options)
		{
		   ros::init(argc,argv,name,options);
		   node_handle_ = std::make_unique<ros::NodeHandle>();
		   it_=std::make_unique<image_transport::ImageTransport>(*node_handle_);
		   it1_=std::make_unique<image_transport::ImageTransport>(*node_handle_);
		}

		void StartNode()
		{
			ros::spin();
		}

		ros::Time Time(double t){
			  ros::Time Time(t);
			return Time;
		}
         /*template<typename S>
	    std::vector<S> RosbagRead(const char* bag_file_name,std::vector<S> sensor_vect)
	   { std::cout<<"entered in ros........."<<bag_file_name<<std::endl;
	    // std::vector<S> Image_vect;

		rosbag::Bag bag;
	 	bag.open(bag_file_name, rosbag::bagmode::Read);
		//ros::Rate loop_rate(5);
		std::vector<std::string> topics;
		topics.push_back(std::string("ImageWrite"));
		topics.push_back(std::string("Image_topic"));
		topics.push_back(std::string("Lidar_topic"));
		topics.push_back(std::string("LidarWrite"));
		rosbag::View view(bag, rosbag::TopicQuery(topics));
		foreach(rosbag::MessageInstance const m, view)
	       {	        
		 typename S::ConstPtr image_msg = m.instantiate<S>();
		if (image_msg != NULL){
		                  S Image_msg;
		                  Image_msg=*image_msg;		          		      
				 // Image_vect.push_back(Image_msg);
				  sensor_vect.push_back(Image_msg);
				  std::cout<<"Image read........."<<sensor_vect.size()<<std::endl;
		                }
               /* sensor_msgs::PointCloudConstPtr msg = m.instantiate<sensor_msgs::PointCloud>();
	        if (msg != NULL){
                          sensor_msgs::PointCloud point_msg;
                          point_msg=*msg;
			  sensor_lidar_vec.push_back(point_msg);
                        }
	       }
	std::cout<<"closing........."<<std::endl;
	   bag.close();
	return sensor_vect;
	 /*if(Image_vect.size()!=0)
	 {
            return Image_vect;
	 }
	 else if(sensor_lidar_vec.size()!=0)
	 {
	    return sensor_lidar_vec;
	 }
	}*/
		template<typename M>
		void RegisterPublisherToTopic(const std::string& topic, uint32_t size);

		template <typename M>
		void PublishMessageToTopic(const M& message, const std::string & topic);


		template<typename M, typename N >
		void RegisterSubscriberToTopic(const std::string& topic, const uint32_t size, void(N::*callback)(M), N *obj);

		template<typename M>
		void RegisterImagePublisherToTopic(const std::string& topic, uint32_t size);

		template <typename M>
		void PublishImageMessageToTopic(const M& message, const std::string & topic);

template <typename M>
		void PublishImageMessageToTopic1(const M& message, const std::string & topic);

		template<typename M>
		void RegisterPointPublisherToTopic(const std::string& topic, uint32_t size);


	private:
		std::unique_ptr<ros::NodeHandle>  node_handle_;    		
		std::unique_ptr<image_transport::ImageTransport>  it_;
		std::unique_ptr<image_transport::ImageTransport>  it1_;
		std::unordered_map<std::string, ros::Publisher> publishers_;
		std::unordered_map<std::string, ros::Subscriber> subscribers_;
		image_transport::Publisher Image_publishers_;
		image_transport::Publisher Image_publishers1_;
	};

}//namespace platform_communicator

#include "ros_communicator_impl.h"

#endif //PLATFORM_COMMUNICATOR_ROS_COMMUNICATOR_H
