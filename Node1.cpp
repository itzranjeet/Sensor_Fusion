#include "Node1.h"
#include "sensor_message_from_lidar_rosbag.h"
#include "sensor_message_from_image_rosbag.h"
#include "sensor_message_from_radar_rosbag.h"
namespace sensor_fusion
{
    RosBagSensors::RosBagSensors(const std::string& nodename, const std::string& topicname):NodeBase{nodename},topic_name_{topicname}
    {}
    RosBagSensors::~RosBagSensors()
     {
        //RegisterAtClock(false);
     }
    void RosBagSensors:: Run(int &argc, char **argv)
     {
        platform_interface_.Init(argc, argv, node_name_);
        RegisterPublishers();
        int a= BagValidationCheck(argv[1]);
          if (a==1)
           {
              SensorSelection();
           }
       PublishSensorsData(); 
						   
       // RegisterSubscribers();
        platform_interface_.Start();
      }
    void RosBagSensors::RegisterPublishers() const
     {
       platform_interface_.RegisterImagePublisher<sensor_msgs::Image>("cam_1", publisher_queue_size);
       //platform_interface_.RegisterImagePublisher<sensor_msgs::Image>("cam_2", publisher_queue_size);
       platform_interface_.RegisterPublisher<sensor_msgs::PointCloud>("lidar_1", publisher_queue_size);
	//platform_interface_.RegisterPublisher<sensor_msgs::PointCloud>("lidar_2", publisher_queue_size);
	//platform_interface_.RegisterPublisher<SensorMessage>("radar_1", publisher_queue_size);     
     }
    void RosBagSensors::RegisterSubscribers()
     {
        //platform_interface_.RegisterSubscriber(sensor_fusion::kEmittedTickTopic, clock_queue_size, &ImageSensor::HandleClock, this);
     }
    bool RosBagSensors::BagValidationCheck(const char* status_csv_path)
     {
        myfile.open(status_csv_path,std::ios::in);      
        while(myfile.good())
         {
         getline (myfile,topic_type,',');
         topic_type_vect.push_back(topic_type);
	 getline (myfile,topic_name,',');
         topic_name_vect.push_back(topic_name);
         topic_name.clear();
         getline (myfile,status,',');
         if(status != "Succeed") 
	  {
           statusflag = false;                 
          }
         getline (myfile,bags_path,'\n');
         bags_path_vect.push_back(bags_path);           
          }
         myfile.close();
	 if(statusflag ==false)
	  {       
           std::cout<<"ROSBAG VALIDATION IS UNSUCCESSFULL"<<std::endl;
 	   return false;
	  }
         else
	  {
	   std::cout<<"ROSBAG VALIDATION IS SUCCESSFULL"<<std::endl;
	   return true;
	  }
 	} 

 void RosBagSensors::SensorSelection()
      {
       for (int i = 0; i <(topic_name_vect.size()-1); i++) 
         {           
           if(topic_type_vect[i]==image_type)
              {
                const char* file_path = bags_path_vect[i].c_str();
                auto Img_reader = sensor_fusion::MessageReaderFactory::CreateImageReader(file_path);
                if(cam_ONE_vect.empty())
		 {                   
                      cam_ONE_vect =  Img_reader->ReadMessage(file_path,topic_name_vect[i]);    

                 } 
                /* if(cam_TWO_vect.empty())
		 {                   
                      cam_TWO_vect =  Img_reader->ReadMessage(file_path,topic_name_vect[i]);                  
                 } */                              
               } 

         /*if(topic_type_vect[i]==image_type)
              {
                const char* file_path = bags_path_vect[i].c_str();
                auto Img_reader = sensor_fusion::MessageReaderFactory::CreateImageReader(file_path);
                if(cam_TWO_vect.empty())
		 {                   
                      cam_TWO_vect =  Img_reader->ReadMessage(file_path,topic_name_vect[i]);    
          
                 } */
           if(topic_type_vect[i]==lidar_type)
               {
                const char* file_path = bags_path_vect[i].c_str();
                auto lidar_reader = sensor_fusion::MessageReaderFactory::CreateLidarReader(file_path);
                if(lidar_ONE_vect.empty())
		 {                 
                      lidar_ONE_vect =  lidar_reader->ReadMessage(file_path,topic_name_vect[i]);                  
                 } 

		/*if(lidar_TWO_vect.empty())
		 {                   
                      lidar_TWO_vect =  lidar_reader->ReadMessage(file_path,topic_name_vect[i]);                  
                 }*/
                           
               }

             
           } 
	}


   /* void RosBagSensors::SensorSelection()
     {
        for (int i = 0; i <(topic_name_vect.size()-1); i++) 
         {           
           if(topic_type_vect[i]==image_type)
              {
                const char* file_path = bags_path_vect[i].c_str();
                auto Img_reader = sensor_fusion::MessageReaderFactory::CreateImageReader(file_path);
                if(cam_ONE_vect.empty())
		 {                   
                      cam_ONE_vect =  Img_reader->ReadMessage(file_path,topic_name_vect[i]);    
                                 
                 } 
                 if(cam_TWO_vect.empty())
		 {                   
                      cam_TWO_vect =  Img_reader->ReadMessage(file_path,topic_name_vect[i]);                  
                 }                               
               }         
          
           }
        std::cout<<"theread1......run...."<<std::endl;
	std::thread thread_1(&RosBagSensors::PublishSensorsData,this); 
 	std::cout<<"theread2......run...."<<std::endl;
     	 std::thread thread_2(&RosBagSensors::PublishSensorsData,this);

	thread_1.join();
     	//thread_2.join();
	}*/
   void RosBagSensors::PublishSensorsData()
     {
      for (int i = 0; i < cam_ONE_vect.size(); i++) 
	{   
                 
           platform_interface_.publish(lidar_ONE_vect[i], "lidar_1");
	std::cout<<"Published lidar_1"<<std::endl;
	 //platform_interface_.publish(lidar_TWO_vect[i], "lidar_2");	    		
           platform_interface_.Imagepublish(cam_ONE_vect[i], "cam_1");
	std::cout<<"Published cam_1"<<std::endl;
	//platform_interface_.Imagepublish(cam_TWO_vect[i], "cam_2");
	
	  // platform_interface_.publish(radar_ONE_vect[i], "radar_1");                                          		
        }
       
     }

} 

int main(int argc, char* argv[])
{
std::shared_ptr<sensor_fusion::NodeBase> sensor;
sensor = std::make_shared<sensor_fusion::RosBagSensors>("Node1",sensor_fusion::kSensorImageMessageTopic);	
sensor->Run(argc,argv);
return 0;
}
