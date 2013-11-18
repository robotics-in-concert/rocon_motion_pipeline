#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

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

  ros::init(argc, argv, "state_publisher");
  ros::NodeHandle n;
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  tf::TransformBroadcaster broadcaster;
  ros::Rate loop_rate(30);
  // Change the next line according to your port name and baud rate
  try{ device.open("/dev/ttyUSB0", 9600); }
  catch(cereal::Exception& e)
  {
      ROS_FATAL("Failed to open the serial port!!!");
      ROS_BREAK();
  }
  ROS_INFO("The serial port is opened.");
  
  // message declarations
  sensor_msgs::JointState joint_state;
  
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
                   
              //ROS_INFO("Got this reply for right hand:  sensor1 : %f ,sensor3 : %f ,sensor5 : %f", pendant_r1,pendant_r3,pendant_r5);
	      //ROS_INFO("Got this reply for left hand:  sensor2 : %f ,sensor4 : %f ,sensor6 : %f", pendant_l2,pendant_l4,pendant_l6);
	       //update joint_state
	       joint_state.header.stamp = ros::Time::now();
	       joint_state.name.resize(6);
	       joint_state.position.resize(3);
	       joint_state.name[0] ="base_to_right_shoulder";
	       joint_state.position[0] = pendant_r1;
	       joint_state.name[1] ="base_to_left_shoulder";
	       joint_state.position[1] = pendant_l1;
	       joint_state.name[2] ="right_shoulder_to_upper";
	       joint_state.position[2] = pendant_r2;
	       joint_state.name[3] ="left_shoulder_to_upper";
	       joint_state.position[3] = pendant_l2;
	       joint_state.name[4] ="right_elbow";
	       joint_state.position[4] = pendant_r3;
	       joint_state.name[5] ="left_elbow";
	       joint_state.position[5] = pendant_l3;

	       data_counter = 0;
	       //send the joint state and transform
               joint_pub.publish(joint_state);
	    }  
	    
            read_len = 0;	    	
	}

        // This will adjust as needed per iteration
        //loop_rate.sleep();
    }   
  return 0;
}

