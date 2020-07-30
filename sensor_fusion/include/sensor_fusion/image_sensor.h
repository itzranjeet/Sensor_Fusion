#ifndef SENSOR_FUSION_CAMERA_SENSOR_H
#define SENSOR_FUSION_CAMERA_SENSOR_H
#include "node_base.h"
#include <sensor_msgs/Image.h>
#include "constants.h"
#include "sensor_message_reader_interface.h"
#include "message_reader_factory.h"
//#include "sensor_message_from_folder.h"
#include <iostream>

namespace sensor_fusion
{
    class ImageSensor : public NodeBase
    {
    public:
        ImageSensor(const std::string& nodename, const std::string& topicname);
        virtual ~ImageSensor();
        virtual void RegisterPublishers() const override;
        virtual void RegisterSubscribers() override;
        virtual void Run(int &argc, char **argv) override;

    private:
        void RegisterAtClock(bool is_new);
        void ResetClock(bool hard_reset = false);
        void HandleClock(const std_msgs::Float64Ptr& clock_message);
        void SendNextSensorMessage(double timestamp);

    protected:
        const std::string topic_name_;
	std::vector<sensor_msgs::Image> sensor_Image_messages_;
        std::vector<sensor_msgs::PointCloud> sensor_Lidar_messages_;
        std::vector<RadarObjects> sensor_Radar_messages_;
	std::vector<sensor_msgs::Image>::const_iterator next_message_;
        double last_timestamp_{std::numeric_limits<double>::infinity()};
    };

}//namespace sensor_fusion`

#endif // SENSOR_FUSION_CAMERA_SENSOR_H
