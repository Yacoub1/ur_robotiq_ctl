#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TransformStamped.h>
#include "muscle_activity/muscle_eg_signal.h"
#include "ros/ros.h"
#include <sstream>
#include <iostream>


using namespace geometry_msgs;
using namespace message_filters;
//using namespace sensor_msgs;

class Node{
public:
	Node():
		cpose_sub_(nh_,"/imu_00341629",100),
		vicon_sub_(nh_,"/vicon/ur10_ee/ur10_ee",100),
		vicon_pose_pub_(nh_.advertise<TransformStamped>("Vicon_pose_sync",100)),
		cpose_ee_Synch_pub_(nh_.advertise<sensor_msgs::Imu>("imu_pose_Synch",100)),
        sync(MySyncPolicy(1000),  cpose_sub_,vicon_sub_){
             sync.registerCallback(boost::bind(&Node::callback, this,_1, _2));
        }


void callback(const sensor_msgs::ImuConstPtr &cpose,const TransformStampedConstPtr &vicon_s)
       {
	   	  ROS_INFO("In callback");
	   	  sensor_msgs::Imu muscle_eg_sync1=*cpose;
          TransformStamped vicon_sych = *vicon_s;
          cpose_ee_Synch_pub_.publish(muscle_eg_sync1);
          vicon_pose_pub_.publish(vicon_sych);

        }
private:
   ros::NodeHandle nh_;
   typedef message_filters::Subscriber<sensor_msgs::Imu> cpose_sub;
   cpose_sub cpose_sub_;
   typedef message_filters::Subscriber<TransformStamped> vicon_sub;
   vicon_sub vicon_sub_;
   typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu,TransformStamped> MySyncPolicy;
   message_filters::Synchronizer< MySyncPolicy > sync;
   ros::Publisher cpose_ee_Synch_pub_;
   ros::Publisher vicon_pose_pub_;
};

///////////////////////////////////////////////////////////////////////
////////////////////////(       Main        )//////////////////////////
///////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
	ROS_INFO("Start ROS SYNCH node!");
    ros::init (argc, argv, "Imu_Vicon_synch_node");
    Node NH;
    ROS_INFO("Message Synchroniser Initialised!");

    while (ros::ok())
    {
        ros::spin();
        ROS_INFO("checking ...");

    }
    ros::shutdown();
    return 0;
}
