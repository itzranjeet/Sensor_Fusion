#include <gtest/gtest.h>
#include "fusion_implementation.h"
#include "fusion_node.h"
#include <sensor_fusion/SensorMessage.h>
#include <sensor_fusion/SensorMessageArray.h>
#include <gmock/gmock.h>


namespace sensor_fusion
{
	TEST(TestFusion, testAdd)
	{
		SensorMessage msg;
		msg.obj_id = 0;
		msg.x = 3.;
		msg.y = 2.;
		msg.timestamp = 1.;
		auto fusion = std::make_unique<FusionImplementation>();
		SensorMessageArrayPtr messages(new SensorMessageArray());
		messages->sensor_messages.push_back(msg);
		uint32_t no_of_msg_queued = fusion->Update(SensorMessage::CAM_SENSOR_ID, messages);
		ASSERT_EQ(no_of_msg_queued, 1);

	}
	
	TEST(TestFusion, testAddFuseTest1)
	{
		SensorMessage msg;
		msg.obj_id = 0;
		msg.x = 3.;
		msg.y = 2.;
		msg.timestamp = 0.;
		auto fusion = std::make_unique<FusionImplementation>();
		SensorMessageArrayPtr messages(new SensorMessageArray());
		messages->sensor_messages.push_back(msg);
		uint32_t no_of_msg_queued = fusion->Update(SensorMessage::CAM_SENSOR_ID, messages);
		ASSERT_EQ(no_of_msg_queued, 1);
		
		sensor_fusion::SensorMessageArray fused_messages;
		fusion->Fuse(messages, fused_messages);
		
		
	}
	TEST(TestFusion, testAddFuseTest2)
	{
		SensorMessage msg;
		msg.obj_id = 1;
		msg.x = 3.;
		msg.y = 2.;
		msg.timestamp = 3.;
		auto fusion = std::make_unique<FusionImplementation>();
		SensorMessageArrayPtr messages(new SensorMessageArray());
		messages->sensor_messages.push_back(msg);
		uint32_t no_of_msg_queued = fusion->Update(SensorMessage::CAM_SENSOR_ID, messages);
		ASSERT_EQ(no_of_msg_queued, 1);
		sensor_fusion::SensorMessageArray fused_messages;
		fusion->Fuse(messages, fused_messages);
		
		
	}
	TEST(TestFusion, testAddFuseTest3)
	{
		SensorMessage msg;
		msg.obj_id = 2;
		msg.x = 4.;
		msg.y = 2.;
		msg.timestamp = 50.;
		auto fusion = std::make_unique<FusionImplementation>();
		SensorMessageArrayPtr messages(new SensorMessageArray());
		messages->sensor_messages.push_back(msg);
		uint32_t no_of_msg_queued = fusion->Update(SensorMessage::CAM_SENSOR_ID, messages);
		ASSERT_EQ(no_of_msg_queued, 1);
		sensor_fusion::SensorMessageArray fused_messages;
		fusion->Fuse(messages, fused_messages);
		
		
	}
	
		
	
	
}//namespace sensor_fusion

int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);
	
	
	return RUN_ALL_TESTS();
}
