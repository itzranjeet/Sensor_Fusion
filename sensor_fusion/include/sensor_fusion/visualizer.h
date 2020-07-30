/// @file visualizer.h
/// @copyright Copyright (C) 2019, KPIT

#ifndef SENSOR_FUSION_VISUALIZER_H
#define SENSOR_FUSION_VISUALIZER_H

#include "constants.h"
#include <queue>
#include <unordered_map>
#include <sensor_fusion/SensorMessage.h>
#include <sensor_fusion/SensorMessageArray.h>
#include <visualization_msgs/MarkerArray.h>
#include "platform_interface.h"

namespace sensor_fusion
{

    struct SensorPoint
    {
        double x;
        double y;
        double x_dif;
        double y_dif;
    };

    struct GraphColor
    {
        double r = 0.0;
        double g = 0.0;
        double b = 0.0;
    };

    enum class PointType
    {
        kFilteredPoint,
        kCameraPoint,
        kShortRangeRadarPoint,
        kLongRangeRadarPoint
    };

    class Visualizer
    {
    public:
        Visualizer() = default;
        ~Visualizer() = default;
        void Initialize(int &argc, char **argv);

        typedef std::unordered_map<int, GraphColor> ColorMap;
        typedef  std::tuple<double,double,double,uint8_t> DataTuple;
        typedef std::unordered_map<PointType, DataTuple> PointMap;
        static PointMap point_type_data_map_;
        static ColorMap color_id_map_;

    private:
        /// @brief Creates a marker which contains common options for points, lines and origin circle
        visualization_msgs::Marker CreatePrimitive(uint64_t object_id, GraphColor color, bool unique_id = false);

        /// @brief Publishes an origin circle to the rviz around the observer object
        void PublishOriginCircle();

        /// @brief Publishes coordinate frame transform information to rviz
        /// @param angle Angle around z-axis (calculated using the coordinates from the last line of the trajectory)
        void SendTFObject(uint64_t object_id, double x, double y, double angle);

        /// @brief Publishes array of the points to rviz
        /// @param message_array Array of ROS-messages received from sensors
        /// @param point_type Type of the point (camera, longrangeradar, shortrangeradar)
        void PublishPointArray(const sensor_fusion::SensorMessageArrayConstPtr &message_array, PointType point_type);

        /// @brief Callback for filtered data (from fusionnode)
        /// @param message_array Array of ROS-messages received from the fusionnode
        void FilteredCallback(const sensor_fusion::SensorMessageArrayConstPtr &message_array);

        GraphColor GetColor(uint64_t object_id);

        /// @brief Messages are processed by the func f function and published.
        template<typename Func, typename... Args>
        void publishVisualElement(uint64_t object_id, Func f, Args...args);

        template<PointType type>
        void SensorCallBack(const sensor_fusion::SensorMessageArrayConstPtr &message_array);

        platform_communicator::PlatformInterface platform_interface_;
        int32_t global_unique_id_{0};
        void RegisterPublishers();
        void RegisterSubscribers();
        std::map <uint32_t, SensorPoint> last_point_map_;
    };

}//namespace sensor_fusion

#endif  // SENSOR_FUSION_VISUALIZER_H
