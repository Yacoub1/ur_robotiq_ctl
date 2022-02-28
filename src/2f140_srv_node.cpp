#include "ros/ros.h"
#include <sstream>
#include <iostream>
#include <ur_msgs/IOStates.h>
#include "std_msgs/String.h"
#include "../../../devel/include/ur_robotiq_ctl/grip_service.h"
#include <ur_msgs/SetIO.h>
#include "../include/robotiq_140_control/ur_robotiq_cmd.h"


class Node{
public:
  Node(){
        double wait_time;
        urs_commander_= nh_.advertiseService("grip_service", &Node::callback,this);
        nh_.param("wait_time", wait_time, 10.0);
        ur_robotiq_.UR_feedback(15);
        ur_robotiq_.init();
        ur_robotiq_.setTimeOut(10);
  }


bool callback(ur_robotiq_ctl::grip_service::Request &req, ur_robotiq_ctl::grip_service::Response &res)
       {
        ROS_INFO("In Service callback function");
        ur_robotiq_.setForce(req.force);
        ur_robotiq_.setSpeed(req.speed);
        ////////////////////////////
        // Send URS cmd to UR CB controller
        ur_robotiq_.moveto(req.pose);

        return true;
        }

private:
   ros::NodeHandle nh_;
   ros::ServiceServer urs_commander_;
   ros::Publisher gripper_status_;
   ur_robotiq_cmd ur_robotiq_;
};


int main(int argc, char* argv[]){

    ros::init(argc, argv, "gripper_test"); // init ROS nodes
    Node NH;

    ROS_INFO("UR Robotiqe Commander is ready!");

    while (ros::ok())
    {
        ros::spin();
        ROS_INFO("checking ...");

    }
    ros::shutdown();
    return 0;
}
