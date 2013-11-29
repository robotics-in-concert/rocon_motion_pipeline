#include <urdf/model.h>
#include "ros/ros.h"

#include "cereal_port/CerealPort.h"

#define REPLY_SIZE 1000
#define TIMEOUT 20

int main(int argc, char** argv){
  cereal::CerealPort device;
  char reply[REPLY_SIZE];
  char* tempData;
  int read_len = 0;
  int data_counter = 0;
  double pendant_r1, pendant_r3, pendant_r5 = 0.0;
  double pendant_l2, pendant_l4, pendant_l6 = 0.0;
  int angle = 0; 

  ros::init(argc, argv, "my_parser");
  //ros::NodeHandle n;
  // Change the next line according to your port name and baud rate
  try{ device.open("/dev/ttyUSB0", 9600); }
  catch(cereal::Exception& e)
  {
      ROS_FATAL("Failed to open the serial port!!!");
      ROS_BREAK();
  }
  ROS_INFO("The serial port is opened.");

  //ros::Rate loop_rate(30);

  if (argc != 2){
    ROS_ERROR("Need a urdf file as argument");
    return -1;
  }
  std::string urdf_file = argv[1];

  urdf::Model model;
  if (!model.initFile(urdf_file)){
    ROS_ERROR("Failed to parse urdf file");
    return -1;
  }
  ROS_INFO("Successfully parsed urdf file");

  while(ros::ok())
    {
        // Send 'R' over the serial port
        //device.write("3");

        // Get the reply, the last value is the timeout in ms
        try{ read_len = device.readLine(reply, REPLY_SIZE, TIMEOUT); }
        catch(cereal::TimeoutException& e)
        {
            ROS_ERROR(e.what());
	    device.open("/dev/ttyUSB0", 9600); 
        }
	if(read_len > 0)
	{
	    //ROS_INFO("read len : %d", read_len);
	    tempData = strtok(reply, "   ");

	    if(strncmp(tempData, "0001", 4)==0)
	    {
		data_counter|=1;
		tempData = strtok(NULL, "\r");
		angle = strtol(tempData,NULL,16);
		pendant_r1 = (double)angle*6.28/1024.0 - 3.14;
	    }
	    else if(strncmp(tempData, "0003", 4)==0)
	    {
		data_counter|=2;
		tempData = strtok(NULL, "\r");
		angle = strtol(tempData,NULL,16);
		pendant_r3 = (double)angle*6.28/1024.0 - 3.14;
	    }
	    else if(strncmp(tempData, "0005", 4)==0)
	    {
		data_counter|=4;
		tempData = strtok(NULL, "\r");
		angle = strtol(tempData,NULL,16);
		pendant_r5 = (double)angle*6.28/1024.0 - 3.14;
	    }
	    else if(strncmp(tempData, "0002", 4)==0)
	    {
		data_counter|=8;
		tempData = strtok(NULL, "\r");
		angle = strtol(tempData,NULL,16);
		pendant_l2 = (double)angle*6.28/1024.0 - 3.14;
	    }
	    else if(strncmp(tempData, "0004", 4)==0)
	    {
		data_counter|=16;
		tempData = strtok(NULL, "\r");
		angle = strtol(tempData,NULL,16);
		pendant_l4 = (double)angle*6.28/1024.0 - 3.14;
	    }
	    else if(strncmp(tempData, "0006", 4)==0)
	    {
		data_counter|=32;
		tempData = strtok(NULL, "\r");
		angle = strtol(tempData,NULL,16);
		pendant_l6 = (double)angle*6.28/1024.0 - 3.14;
	    }
    

	    if(data_counter == 63)
	    {            
                   
              ROS_INFO("Got this reply for right hand:  sensor1 : %f ,sensor3 : %f ,sensor5 : %f", pendant_r1,pendant_r3,pendant_r5);
	      ROS_INFO("Got this reply for left hand:  sensor2 : %f ,sensor4 : %f ,sensor6 : %f", pendant_l2,pendant_l4,pendant_l6);

	      data_counter = 0;
	    }  
	    
            read_len = 0;	    	
	}

        // This will adjust as needed per iteration
        //loop_rate.sleep();
    }   
  return 0;
}

