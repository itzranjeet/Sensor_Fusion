#include "sensor_message_from_csv_convertor.h"
#include <sensor_fusion/SensorMessage.h>
#include <gtest/gtest.h>
#include "message_reader_factory.h"
namespace sensor_fusion
{
	TEST(TestCsvFile, testCameraCsv)
	{
		std::string csv_file_name = "camera.csv";
		auto reader = sensor_fusion::MessageReaderFactory::CreateReader(csv_file_name.c_str());
		auto sensor_messages = reader->ReadMessage(csv_file_name.c_str());
		ASSERT_EQ(sensor_messages.size(), 5724);
		auto message = sensor_messages.at(9);
		ASSERT_EQ(message.obj_id, 9);
		ASSERT_EQ(message.x, 65.84579701748567);
		ASSERT_EQ(message.y, 3.1009491178544804);
		ASSERT_EQ(message.timestamp, 0.0);
	}

	TEST(TestCsvFile, testLongRangeRadarCsv)
	{
		std::string csv_file_name = "long_range_radar.csv";
		auto csv_converter = std::make_unique<SensorMessageFromCsvConvertor>();
		auto sensor_messages = csv_converter->ReadMessage(csv_file_name.c_str());
		ASSERT_EQ(sensor_messages.size(), 3807);
		auto message = sensor_messages.at(120);
		ASSERT_EQ(message.obj_id, 33);
		ASSERT_EQ(message.x, 43.34761591130326);
		ASSERT_EQ(message.y, -2.348481593792047);
		ASSERT_EQ(message.timestamp, 6.0);
	}

	TEST(TestCsvFile, testShortRangeRadarCsv)
	{
		std::string csv_file_name = "short_range_radar.csv";
		auto csv_converter = std::make_unique<SensorMessageFromCsvConvertor>();
		auto sensor_messages = csv_converter->ReadMessage(csv_file_name.c_str());
		ASSERT_EQ(sensor_messages.size(), 2929);
		auto message = sensor_messages.at(1998);
		ASSERT_EQ(message.obj_id, 61);
		ASSERT_EQ(message.x, 18.41768404138929);
		ASSERT_EQ(message.y, 0.9945127379561618);
		ASSERT_EQ(message.timestamp, 130.0);
	}

	TEST(TestCsvFile, testNonExistingFile)
	{
		std::string csv_file_name = "this_file_does_not_exist.csv";
		auto csv_converter = std::make_unique<SensorMessageFromCsvConvertor>();
		auto sensor_messages = csv_converter->ReadMessage(csv_file_name.c_str());
		ASSERT_EQ(sensor_messages.size(), 0);
	}
}//namespace sensor_fusion

int main(int argc, char* argv[])
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
