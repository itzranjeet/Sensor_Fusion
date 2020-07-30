#ifndef SENSOR_FUSION_ROSBAG_SENSORS_H
#define SENSOR_FUSION_ROSBAG_SENSORS_H
#include "node_base.h"
#include <sensor_msgs/Image.h>
#include "constants.h"
#include "sensor_message_reader_interface.h"
#include "message_reader_factory.h"
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <thread>
namespace sensor_fusion
{
    class RosBagSensors : public NodeBase
    {
    public:
        RosBagSensors(const std::string& nodename, const std::string& topicname);
        virtual ~RosBagSensors();
        virtual void RegisterPublishers() const override;
        virtual void RegisterSubscribers() override;
        virtual void Run(int &argc, char **argv) override;

    private:
        bool BagValidationCheck(const char* status_csv_path);
        void SensorSelection();
        void PublishSensorsData();


    protected:
        const std::string topic_name_;
	std::vector<sensor_msgs::Image> cam_ONE_vect;
        std::vector<sensor_msgs::Image> cam_TWO_vect;
        std::vector<sensor_msgs::PointCloud> lidar_ONE_vect;
	std::vector<sensor_msgs::PointCloud> lidar_TWO_vect;     
        std::vector<SensorMessage> radar_ONE_vect;
	std::vector<sensor_msgs::Image>::const_iterator next_message_;
        double last_timestamp_{std::numeric_limits<double>::infinity()};

	std::ifstream myfile;
	std::string topic_type;
	std::string topic_name;
	std::string status;
        std::string image_type= "sensor_msgs/Image";
        std::string radar_type= "sensor_fusion/SensorMessage";
        std::string lidar_type= "sensor_msgs/PointCloud";
	std::string bags_path;	
	bool statusflag =true;	
	std::vector<std::string> topic_type_vect; 
	std::vector<std::string> topic_name_vect;
	std::vector<std::string> bags_path_vect;
        std::vector<std::string> img_vect1;
        std::vector<std::string> img_vect2;
    };

}//namespace sensor_fusion`

#endif // SENSOR_FUSION_ROSBAG_SENSORS_H
