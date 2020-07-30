#ifndef SENSOR_FUSION_TRACKER_H
#define SENSOR_FUSION_TRACKER_H

#include <queue>
#include <sensor_fusion/SensorMessage.h>
#include <opencv2/video/tracking.hpp>
#include <opencv2/core/mat.hpp>

namespace sensor_fusion
{

	enum class State {INIT,TRACK,POST,LOST};
	
	class TrackedObject
	{
	public:
		explicit TrackedObject(const SensorMessage& msg);
		virtual ~TrackedObject() = default;
		void SetCurrentState(State state);
		bool ProcessMessage(SensorMessage& msg, bool tracked, unsigned  int current_frame_count);
		bool IsOffTrack();
		auto GetObjectID() {return id;}
		State GetCurrentState();
		void Predict();
		void Correct(const SensorMessage& msg);
		const SensorMessage& GetFilteredMessage();
		unsigned int GetLastFrameUpdate() const { return last_frame_update;}
		
	private:
		auto CorelateID(SensorMessage& msg) ;
		auto CorelateData(SensorMessage& msg) ;
		State current_state;
		uint32_t id;
		double x,y;
		int32_t tolerence_x,tolerence_y,tolerence_d;
		int tracker_not_found_couter;
		int init_found_counter;
		int posttrack_not_found_counter;
		cv::KalmanFilter kalman_filter;
		cv::Mat_<double> corrected_value, prediction;
		SensorMessage filtered_message_;
		std::vector<SensorMessage> measured_messages_;
		unsigned int last_frame_update;
	};

} // namespace sensor_fusion

#endif // SENSOR_FUSION_TRACKER_H
