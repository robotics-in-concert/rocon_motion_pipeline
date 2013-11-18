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
  char* tempData="XXXX   0123  ";
  int read_len = 0;
  int data_counter = 0;
  double pendant_r1, pendant_r3, pendant_r5 = 0.0;
  double pendant_r7, pendant_r9, pendant_r11 = 0.0;
  double pendant_r13, pendant_r15, pendant_r17 = 0.0;
  double pendant_l2, pendant_l4, pendant_l6 = 0.0;
  double pendant_l8, pendant_l10, pendant_l12 = 0.0;
  double pendant_l14, pendant_l16, pendant_l18 = 0.0;
  int angle = 0; 

  ros::init(argc, argv, "pandent_publisher");
  ros::NodeHandle n;
  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  tf::TransformBroadcaster broadcaster;
  ros::Rate loop_rate(200);
  // Change the next line according to your port name and baud rate
  try{ device.open("/dev/ttyUSB0", 19200); }
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
	    device.open("/dev/ttyUSB0", 19200); 
        }

	tempData = strtok(reply, "   ");
	while(read_len > 0)
	{
	    if(strncmp(tempData, "0001", 4)==0)
	    {
		data_counter|=1;
		tempData = strtok(NULL, "\r\n");
		angle = strtol(tempData,NULL,16);
		pendant_r1 = (double)angle*6.28/1024.0 - 3.14;		    
            	read_len -= 13;
		tempData = strtok(NULL, "   ");
	    }
	    else if(strncmp(tempData, "0003", 4)==0)
	    {
		data_counter|=2;
		tempData = strtok(NULL, "\r\n");
		angle = strtol(tempData,NULL,16);
		pendant_r3 = (double)angle*6.28/1024.0 - 3.14;		    
            	read_len -= 13;
		tempData = strtok(NULL, "   ");
	    }
	    else if(strncmp(tempData, "0005", 4)==0)
	    {
		data_counter|=4;
		tempData = strtok(NULL, "\r\n");
		angle = strtol(tempData,NULL,16);
		pendant_r5 = (double)angle*6.28/1024.0 - 3.14;		    
            	read_len -= 13;
		tempData = strtok(NULL, "   ");
	    }
	    else if(strncmp(tempData, "0002", 4)==0)
	    {
		data_counter|=8;
		tempData = strtok(NULL, "\r\n");
		angle = strtol(tempData,NULL,16);
		pendant_l2 = (double)angle*6.28/1024.0 - 3.14;		    
            	read_len -= 13;
		tempData = strtok(NULL, "   ");
	    }
	    else if(strncmp(tempData, "0004", 4)==0)
	    {
		data_counter|=16;
		tempData = strtok(NULL, "\r\n");
		angle = strtol(tempData,NULL,16);
		pendant_l4 = (double)angle*6.28/1024.0 - 3.14;		    
            	read_len -= 13;	
	    }
	    else if(strncmp(tempData, "0006", 4)==0)
	    {
		data_counter|=32;
		tempData = strtok(NULL, "\r\n");
		angle = strtol(tempData,NULL,16);
		pendant_l6 = (double)angle*6.28/1024.0 - 3.14;		    
            	read_len -= 13;
		tempData = strtok(NULL, "   ");	
	    }
	    else if(strncmp(tempData, "0007", 4)==0)
	    {
		data_counter|=64;
		tempData = strtok(NULL, "\r\n");
		angle = strtol(tempData,NULL,16);
		pendant_r7 = (double)angle*6.28/1024.0 - 3.14;		    
            	read_len -= 13;		
	    }
	    else if(strncmp(tempData, "0009", 4)==0)
	    {
		data_counter|=128;
		tempData = strtok(NULL, "\r\n");
		angle = strtol(tempData,NULL,16);
		pendant_r9 = (double)angle*6.28/1024.0 - 3.14;			    
            	read_len -= 13;
		tempData = strtok(NULL, "   ");	
	    }
	    else if(strncmp(tempData, "000B", 4)==0)
	    {
		data_counter|=256;
		tempData = strtok(NULL, "\r\n");
		angle = strtol(tempData,NULL,16);
		pendant_r11 = (double)angle*6.28/1024.0 - 3.14;			    
            	read_len -= 13;	
	    }
	    else if(strncmp(tempData, "0008", 4)==0)
	    {
		data_counter|=512;
		tempData = strtok(NULL, "\r\n");
		angle = strtol(tempData,NULL,16);
		pendant_l8 = (double)angle*6.28/1024.0 - 3.14;		    
            	read_len -= 13;
		tempData = strtok(NULL, "   ");		
	    }
	    else if(strncmp(tempData, "000A", 4)==0)
	    {
		data_counter|=1024;
		tempData = strtok(NULL, "\r\n");
		angle = strtol(tempData,NULL,16);
		pendant_l10 = (double)angle*6.28/1024.0 - 3.14;			    
            	read_len -= 13;	
	    }
	    else if(strncmp(tempData, "000C", 4)==0)
	    {
		data_counter|=2048;
		tempData = strtok(NULL, "\r\n");
		angle = strtol(tempData,NULL,16);
		pendant_l12 = (double)angle*6.28/1024.0 - 3.14;			    
            	read_len -= 13;	
	    }
            
	    else if(strncmp(tempData, "000D", 4)==0)
	    {
		data_counter|=4096;
		tempData = strtok(NULL, "\r\n");
		angle = strtol(tempData,NULL,16);
		pendant_r13 = (double)angle*6.28/1024.0 - 3.14;			    
            	read_len -= 13;
		tempData = strtok(NULL, "   ");	
	    }
	    else if(strncmp(tempData, "000F", 4)==0)
	    {
		data_counter|=8192;
		tempData = strtok(NULL, "\r\n");
		angle = strtol(tempData,NULL,16);
		pendant_r15 = (double)angle*6.28/1024.0 - 3.14;		    
            	read_len -= 13;
		tempData = strtok(NULL, "   ");
	    }
	    else if(strncmp(tempData, "0011", 4)==0)
	    {
		data_counter|=16384;
		tempData = strtok(NULL, "\r\n");
		angle = strtol(tempData,NULL,16);
		pendant_r17 = (double)angle*6.28/1024.0 - 3.14;	    
            	read_len -= 13;
		tempData = strtok(NULL, "   ");
	    }
	    else if(strncmp(tempData, "000E", 4)==0)
	    {
		data_counter|=32768;
		tempData = strtok(NULL, "\r\n");
		angle = strtol(tempData,NULL,16);
		pendant_l14 = (double)angle*6.28/1024.0 - 3.14;		    
            	read_len -= 13;
		tempData = strtok(NULL, "   ");
	    }
	    else if(strncmp(tempData, "0010", 4)==0)
	    {
		data_counter|=65536;
		tempData = strtok(NULL, "\r\n");
		angle = strtol(tempData,NULL,16);
		pendant_l16 = (double)angle*6.28/1024.0 - 3.14;		    
            	read_len -= 13;
		tempData = strtok(NULL, "   ");
	    }
	    else if(strncmp(tempData, "0012", 4)==0)
	    {
		data_counter|=131072;
		tempData = strtok(NULL, "\r\n");
		angle = strtol(tempData,NULL,16);
		pendant_l18 = (double)angle*6.28/1024.0 - 3.14;		    
            	read_len -= 13;
		tempData = strtok(NULL, "   ");
	    }
	    else
	    {
		read_len = 0;
	    }	
	    if(data_counter == 262143)
	    {            
                   
               //ROS_INFO("Got this reply for hands:  sensor1 : %f ,sensor3 : %f ,sensor5, sensor2 : %f ,sensor4 : %f ,sensor6 : %f", pendant_r1,pendant_r3,pendant_r5, pendant_l2,pendant_l4,pendant_l6);
	       ROS_INFO("All sensor data updated!!!");
	       data_counter = 0;
	    }
	       //update joint_state
	       joint_state.header.stamp = ros::Time::now();
	       joint_state.name.resize(19);
	       joint_state.position.resize(19);
	       joint_state.name[0] ="base_to_right_shoulder";
	       joint_state.position[0] = pendant_r1;
	       joint_state.name[1] ="base_to_left_shoulder";
	       joint_state.position[1] = pendant_l2;
	       joint_state.name[2] ="right_shoulder_to_upper";
	       joint_state.position[2] = pendant_r3;
	       joint_state.name[3] ="left_shoulder_to_upper";
	       joint_state.position[3] = pendant_l4;
	       joint_state.name[4] ="right_elbow";
	       joint_state.position[4] = pendant_r5;
	       joint_state.name[5] ="left_elbow";
	       joint_state.position[5] = pendant_l6;
	       joint_state.name[6] ="base_to_right_hip";
	       joint_state.position[6] = pendant_r7;
	       joint_state.name[7] ="base_to_left_hip";
	       joint_state.position[7] = pendant_l8;
	       joint_state.name[8] ="right_hip_to_split";
	       joint_state.position[8] = pendant_r9;
	       joint_state.name[9] ="left_hip_to_split";
	       joint_state.position[9] = pendant_l10;
	       joint_state.name[10] ="right_hip_split_to_upper";
	       joint_state.position[10] = pendant_r11;
	       joint_state.name[11] ="left_hip_split_to_upper";
	       joint_state.position[11] = pendant_l12;
	       joint_state.name[12] ="right_knee";
	       joint_state.position[12] = pendant_r13;
	       joint_state.name[13] ="left_knee";
	       joint_state.position[13] = pendant_l14;
	       joint_state.name[14] ="right_ankle";
	       joint_state.position[14] = pendant_r15;
	       joint_state.name[15] ="left_ankle";
	       joint_state.position[15] = pendant_l16;
	       joint_state.name[16] ="right_ankle_roll";
	       joint_state.position[16] = pendant_r17;
	       joint_state.name[17] ="left_ankle_roll";
	       joint_state.position[17] = pendant_l18;
	       joint_state.name[18] ="base_to_head";
	       joint_state.position[18] = 0.0;
	       //send the joint state and transform
               joint_pub.publish(joint_state);
	}
        // This will adjust as needed per iteration
        loop_rate.sleep();
    }   
  return 0;
}

