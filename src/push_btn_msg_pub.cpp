#include "ros/ros.h"
#include <sstream>
#include <iostream>
#include <ur_msgs/IOStates.h>
#include "std_msgs/String.h"
#include "message_synch/di_msg.h"
#include <ur_msgs/SetIO.h>

class Node{
public:
  Node(){
        double wait_time;
        ur10_io_sub_= nh_.subscribe("/ur_driver/io_states", 100, &Node::callback, this);
        nh_.param("wait_time", wait_time, 10.0);
        ini_srv();
        pin1_msg_pub = nh_.advertise<message_synch::di_msg>("/ur_di_pin1",100);
  }

 void ini_srv(){
    ros::ServiceClient io_client = nh_.serviceClient<ur_msgs::SetIO>("/ur_hardware_interface/set_io");
    ur_msgs::SetIO io_service;
    io_service.request.fun = 0;
    io_service.request.pin = 0;
  }

void callback(const ur_msgs::IOStatesConstPtr& io_state)
       {
          //ROS_INFO("In callback");
          ur_msgs::IOStates io_state_r=*io_state;
          message_synch::di_msg msg;
          msg.value_di = io_state_r.digital_in_states[1].state;
          if (msg.value_di){
            ROS_INFO("Muscle Fatigue is reached!");
          }
          msg.header.frame_id = "ur_di_pin1";
          msg.header.stamp = ros::Time::now();
          //ROS_INFO("%s", msg.data.c_str());
          pin1_msg_pub.publish(msg);
        }

private:
   ros::NodeHandle nh_;
   ros::Subscriber ur10_io_sub_;
   ros::Publisher pin1_msg_pub;
};
///////////////////////////////////////////////////////////////////////
////////////////////////(       Main        )//////////////////////////
///////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
  ROS_INFO("Start ur_di_pin1 msg node!");
    ros::init(argc, argv, "ur_di_msg_pub");
    Node NH;
    ROS_INFO("di msg publisher initialised!");
//    ros::Rate loop_rate(1000);
    while (ros::ok())
    {
        ros::spin();
        //ROS_INFO("checking ...");
    }
    ros::shutdown();
    return 0;
}
