#include <ros/ros.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "cereal_port/CerealPort.h"

#define REPLY_SIZE 30
#define TIMEOUT 20

// This example opens the serial port and sends a request 'R' at 1Hz and waits for a reply.
int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_tf_publisher");
    ros::NodeHandle n;

    cereal::CerealPort device;
    char reply[REPLY_SIZE];
    char* tempData;
    int read_len = 0;
    int data_counter = 0;
    double imu_x, imu_y, imu_z, imu_w = 0.0;
    char* head_ID = "4-2";
    char* right_hand_ID = "6-1";
    char* left_hand_ID = "6-3";

    // Change the next line according to your port name and baud rate
    try{ device.open("/dev/ttyUSB0", 115200); }
    catch(cereal::Exception& e)
    {
        ROS_FATAL("Failed to open the serial port!!!");
        ROS_BREAK();
    }
    ROS_INFO("The serial port is opened.");

    ros::Rate r(500);

    tf::TransformBroadcaster br;
    tf::TransformListener ls;
    std::string transformString1;
    std::string transformString2;
    tf::StampedTransform stamped_transform;

    while(ros::ok())
    {
        // Send 'R' over the serial port
        //device.write("3");

        // Get the reply, the last value is the timeout in ms
        try{ read_len = device.read(reply, REPLY_SIZE, TIMEOUT); }
        catch(cereal::TimeoutException& e)
        {
            ROS_ERROR(e.what());
	    device.open("/dev/ttyUSB0", 115200); 
        }
	if(read_len > 0)
	{
	    tempData = strtok(reply, ",");
	    data_counter = 0;
	    if(tempData = strtok(NULL, ","))
	    {
		data_counter++;
		imu_x = (double)atoi(tempData)/10000.0;
	    }
	    if(tempData = strtok(NULL, ","))
	    {
		data_counter++;
		imu_y = (double)atoi(tempData)/10000.0;
	    }
	    if(tempData = strtok(NULL, ","))
	    {
		data_counter++;
		imu_z = (double)atoi(tempData)/10000.0;
	    }
	    if(tempData = strtok(NULL, ","))
	    {
		data_counter++;
		imu_w = (double)atoi(tempData)/10000.0;
	    }	    

	    if(data_counter == 4 && strncmp(reply, head_ID, 3)==0)
	    {
		  transformString1 = "neck";
		  transformString2 = "head";
		  try
		  {
			ls.waitForTransform(transformString1,transformString2,ros::Time(0),ros::Duration(1.0));
			ls.lookupTransform(transformString1,transformString2,ros::Time(0),stamped_transform);
		  }
		  catch(tf::TransformException const &ex)
		  {
			ROS_DEBUG_STREAM(ex.what());
    			ROS_WARN_STREAM("Couldn't get neck to head transform!");
		  }

		  stamped_transform.setRotation(tf::Quaternion(imu_x,imu_y,imu_z,imu_w));
		  br.sendTransform(tf::StampedTransform(stamped_transform, ros::Time::now(),stamped_transform.frame_id_,"imu_head"));
            	  ROS_INFO("Got this reply for head:   x : %f ,y : %f ,z : %f ,w : %f", imu_x, imu_y, imu_z, imu_w);

	    }
	    else if(data_counter == 4 && strncmp(reply, right_hand_ID, 3)==0)
	    {
              transformString1 = "right_elbow";
              transformString2 = "right_hand";
              try
              {
                    ls.waitForTransform(transformString1,transformString2,ros::Time(0),ros::Duration(1.0));
                    ls.lookupTransform(transformString1,transformString2,ros::Time(0),stamped_transform);
              }
              catch(tf::TransformException const &ex)
              {
                    ROS_DEBUG_STREAM(ex.what());
                    ROS_WARN_STREAM("Couldn't get right elbow to right hand transform!");
              }

              stamped_transform.setRotation(tf::Quaternion(imu_x,imu_y,imu_z,imu_w));
              br.sendTransform(tf::StampedTransform(stamped_transform, ros::Time::now(),stamped_transform.frame_id_,"imu_right_hand"));
              ROS_INFO("Got this reply for right hand: x : %f ,y : %f ,z : %f ,w : %f", imu_x, imu_y, imu_z, imu_w);
	    }
	    else if(data_counter == 4 && strncmp(reply, left_hand_ID, 3)==0)
	    {
              transformString1 = "left_elbow";
              transformString2 = "left_hand";
              try
              {
                    ls.waitForTransform(transformString1,transformString2,ros::Time(0),ros::Duration(1.0));
                    ls.lookupTransform(transformString1,transformString2,ros::Time(0),stamped_transform);
              }
              catch(tf::TransformException const &ex)
              {
                    ROS_DEBUG_STREAM(ex.what());
                    ROS_WARN_STREAM("Couldn't get left elbow to left hand transform!");
              }

              //transform.setOrigin(tf::Vector3(0.0,0.0,0.0));
              stamped_transform.setRotation(tf::Quaternion(imu_x,imu_y,imu_z,imu_w));
              br.sendTransform(tf::StampedTransform(stamped_transform, ros::Time::now(),stamped_transform.frame_id_,"imu_left_hand"));
              ROS_INFO("Got this reply for left hand:  x : %f ,y : %f ,z : %f ,w : %f", imu_x, imu_y, imu_z, imu_w);
	    }	    
	    
            read_len = 0;	    	
	}

        ros::spinOnce();
        r.sleep();
    }   
}
