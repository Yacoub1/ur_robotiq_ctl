#include "ros/ros.h"
#include <sstream>
#include <iostream>
#include <ur_msgs/IOStates.h>
#include "std_msgs/String.h"
#include "../../../devel/include/ur_robotiq_ctl/grip_service.h"
#include <ur_msgs/SetIO.h>
#include "../include/robotiq_140_control/ur_robotiq_cmd.h"
#include <cstdlib>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ur_robotiq_srv_client_node");
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<ur_robotiq_ctl::grip_service>("grip_service");
    ur_robotiq_ctl::grip_service srv;
    srv.request.speed = 10;
    srv.request.force = 100;
    srv.request.pose = 50;
    client.call(srv);
    //ROS_INFO(srv.request);
    if (client.call(srv))
    {
      ROS_INFO("Gripper Status: %ld", (long int)srv.response.state);
    }
    else
    {
      ROS_ERROR("Failed to call service grip_service");
      return 1;
    }
}
