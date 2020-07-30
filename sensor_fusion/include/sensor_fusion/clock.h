#ifndef SENSOR_FUSION_CLOCK_H
#define SENSOR_FUSION_CLOCK_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <fstream>

#include "constants.h"
#include "Utility.h"

namespace sensor_fusion
{

    class Clock
    {
    public:
        Clock ();
        ~Clock() = default;

        void Initialize(int &argc, char **argv);
        void TickConsumerCallback(const std_msgs::Float64::ConstPtr& consumed_timestamp);
        void ResetCallback(const std_msgs::Bool::ConstPtr& hard_reset);
        void RegisterCallback(const std_msgs::Bool::ConstPtr& register_new);
        void TimerCallback(const ros::TimerEvent&);
        void StartTimerTick();
        auto GetLoggerInstance();
        auto GetLogStreamerInstance();

    private:
        void PublishTimestamp();
        void PrintUsage(const char* exe_name);
        void RegisterclkSubscribers();
        void RegisterclkPublishers();

        int clockticksconsumed_;
        double timerinterval_;
        double timestamp_;
        ros::Publisher clockpublisher_;
        double clockstep_;
        bool timermode_;
        int sensornodecount_;
        double initialtimestamp_;
        ros::Timer timer_;
        int queuesize_;
        std::unique_ptr<ros::NodeHandle> clknodehandle_;
        std::unordered_map<std::string, ros::Subscriber> subscribers_;
    };

} //namespace sensor_fusion

#endif //SENSOR_FUSION_CLOCK_H
