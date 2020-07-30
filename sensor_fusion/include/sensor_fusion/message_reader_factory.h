#ifndef SENSOR_FUSION_MESSAGE_READER_H
#define SENSOR_FUSION_MESSAGE_READER_H
#include <sensor_message_reader_interface.h>
//#include <sensor_fusion/SensorMessage.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_fusion/SensorMessage.h>
#include <memory>

namespace sensor_fusion
{

	class MessageReaderFactory
	{
	public:
		static std::shared_ptr<SensorMessageReaderInterface<sensor_msgs::Image>> CreateImageReader(const char* readerpath);
                static std::shared_ptr<SensorMessageReaderInterface<sensor_msgs::PointCloud>> CreateLidarReader(const char* readerpath);
                static std::shared_ptr<SensorMessageReaderInterface<SensorMessage>> CreateRadarReader(const char* readerpath);
	};

}//namespace sensor_fusion

#endif // SENSOR_FUSION_MESSAGE_READER_H
