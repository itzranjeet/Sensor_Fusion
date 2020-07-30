#include <gtest/gtest.h>
#include "sensor_fusion/SensorMessage.h"
#include "average_position_calculator.h"
#include "tracked_object.h"
#include <opencv2/video/tracking.hpp>
#include <opencv2/core/mat.hpp>
#include <cmath>
namespace sensor_fusion
{
	TEST(TestTracker, testGetLatestRawPoint)
	{
			SensorMessage msg;
			msg.obj_id = 0;
			msg.x = 3.;
			msg.y = 2.;
			msg.timestamp = 1.;
			auto tracker = std::make_unique<AveragePositionCalculator>();
			tracker->UpdateAverage(msg);
			SensorMessage filtered_message = tracker->GetAverageMessage();
			ASSERT_EQ(filtered_message.x, msg.x);
			ASSERT_EQ(filtered_message.y, msg.y);
	 }
	 TEST(TestTracker, testCurrentState)
         {
		   SensorMessage msg;
		   msg.obj_id = 0;
		   msg.x = 3.;
		   msg.y = 2.;
		   msg.timestamp = 1.;
		  TrackedObject trackobj(msg);
		  trackobj.SetCurrentState(State::TRACK);
		  ASSERT_EQ(State::TRACK,trackobj.GetCurrentState());


         } 

        TEST(TestTracker, testIsOffTrack)
        {
			 SensorMessage msg;
			 msg.obj_id = 0;
			 msg.x = 3.;
			 msg.y = 2.;
			 msg.timestamp = 1.;
			 TrackedObject trackobj(msg);
			 trackobj.SetCurrentState(State::TRACK);          
			 ASSERT_NE(true,trackobj.IsOffTrack()); 
			 trackobj.SetCurrentState(State::LOST);
			 ASSERT_EQ(true,trackobj.IsOffTrack());
         } 
      TEST(TestTracker, testProcessMsg)
      {
			SensorMessage msg, msgnew;
			msg.obj_id = 1;
			msg.x = 3.;
			msg.y = 2.;
			msg.timestamp = 1.;
			TrackedObject trackobj(msg);
			
			msgnew.obj_id = 2;
			msg.x = 4.;
			msg.y = 5.;
			msg.timestamp = 2.;
			
			trackobj.SetCurrentState(State::TRACK);
			ASSERT_EQ(true,trackobj.ProcessMessage(msg,false,5));
			ASSERT_EQ(false,trackobj.ProcessMessage(msg,true,5));
			ASSERT_EQ(false,trackobj.ProcessMessage(msg,true,5));
			ASSERT_EQ(false,trackobj.ProcessMessage(msg,true,5));
			ASSERT_EQ(false,trackobj.ProcessMessage(msg,true,5));
				
			 trackobj.SetCurrentState(State::POST);
			 ASSERT_EQ(true,trackobj.ProcessMessage(msg,false,5));
			 ASSERT_EQ(false,trackobj.ProcessMessage(msg,true,5));
			 ASSERT_EQ(false,trackobj.ProcessMessage(msg,true,5));
			 ASSERT_EQ(false,trackobj.ProcessMessage(msg,true,5));
			 ASSERT_EQ(false,trackobj.ProcessMessage(msg,true,5));
			 ASSERT_EQ(false,trackobj.ProcessMessage(msg,true,5));
				
			 trackobj.SetCurrentState(State::INIT);
				//	ASSERT_EQ(false,trackobj.ProcessMessage(msgnew,false,5));
			 ASSERT_EQ(false,trackobj.ProcessMessage(msg,true,5));	
			 ASSERT_EQ(true,trackobj.ProcessMessage(msg,false,5));
			 ASSERT_EQ(true,trackobj.ProcessMessage(msg,false,5));
			 ASSERT_EQ(true,trackobj.ProcessMessage(msg,false,5));
			//   ASSERT_EQ(false,trackobj.ProcessMessage(msgnew,false,5));
			//    ASSERT_EQ(false,trackobj.ProcessMessage(msgnew,true,5));


      }

}
int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
