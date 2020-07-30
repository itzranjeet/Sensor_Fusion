#include "message_reader_factory.h"
#include "sensor_message_from_image_rosbag.h"
#include "sensor_message_from_lidar_rosbag.h"
#include "sensor_message_from_radar_rosbag.h"
#include "get_file_extension.h"

namespace sensor_fusion
{
    std::shared_ptr<SensorMessageReaderInterface<sensor_msgs::Image>> MessageReaderFactory::CreateImageReader(const char* readerpath)
    {
        if( access( readerpath, F_OK ) != -1 )
        {

          return std::shared_ptr<SensorMessageReaderInterface<sensor_msgs::Image>>( new sensor_fusion::SensorMessageFromImageRosbag());
                             
        }
        return nullptr;
    }

   std::shared_ptr<SensorMessageReaderInterface<sensor_msgs::PointCloud>> MessageReaderFactory::CreateLidarReader(const char* readerpath)
    {
    if( access( readerpath, F_OK ) != -1 )
     {
       return std::shared_ptr<SensorMessageReaderInterface<sensor_msgs::PointCloud>>( new sensor_fusion::SensorMessageFromLidarRosbag());
     }
        return nullptr;
    }

   std::shared_ptr<SensorMessageReaderInterface<SensorMessage>> MessageReaderFactory::CreateRadarReader(const char* readerpath)
    {
    if( access( readerpath, F_OK ) != -1 )
     {
       return std::shared_ptr<SensorMessageReaderInterface<SensorMessage>>( new sensor_fusion::SensorMessageFromRadarRosbag());
     }
        return nullptr;
    }
}//namespace sensor_fusion
