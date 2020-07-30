#ifndef SENSOR_FUSION_FUSION_IMPLEMENTATION_H
#define SENSOR_FUSION_FUSION_IMPLEMENTATION_H

#include <deque>
#include <map>
#include <unordered_map>

#include <sensor_fusion/SensorMessage.h>
#include <sensor_fusion/SensorMessageArray.h>
#include "tracked_object.h"
#include "average_position_calculator.h"

namespace sensor_fusion
{

    class FusionImplementation
    {
    public:
        virtual ~FusionImplementation() = default;
        FusionImplementation();

        uint32_t Update(const uint32_t& sensor_id, const SensorMessageArrayConstPtr& messages);
        void Fuse(const SensorMessageArrayConstPtr& frame, sensor_fusion::SensorMessageArray& fused_messages);

    private:
        void GetLatestDetectionsFromQueue(double timestamp, double& last_collect_stamp,
                                          const std::deque<SensorMessageArray>& queue, std::vector<SensorMessage>& messages);
        void ProcessTrackedObject(SensorMessage, sensor_fusion::SensorMessageArray& fused_messages );

        std::deque<SensorMessageArray> srr_frames_;
        std::deque<SensorMessageArray> lrr_frames_;
        std::deque<SensorMessageArray> cam_frames_;
        double last_collected_srr_stamp_{-1.0};
        double last_collected_lrr_stamp_ {1.0};
        double last_collected_cam_stamp_ {-1.0};
        unsigned int internal_frame_counter{0};
        std::unordered_map<uint32_t, std::unique_ptr<AveragePositionCalculator> > sensor_data_fusion_;
        typedef std::list<std::shared_ptr<TrackedObject>> ObjectTracker;
        ObjectTracker object_tracker_;
        std::map<uint32_t, std::deque<sensor_fusion::SensorMessageArray>*> sensor_message_queues_;
    };

}//namespace sensor_fusion

#endif // SENSOR_FUSION_FUSION_IMPLEMENTATION_H
