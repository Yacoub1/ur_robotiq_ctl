#ifndef UR_ROBOTIQ_CMD_H
#define UR_ROBOTIQ_CMD_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ur_msgs/SetIO.h>
#include <ur_msgs/IOStates.h>
#include <sstream>


class ur_robotiq_cmd
{

private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;
    ros::ServiceClient client;

    std_msgs::String msg_out;
    ur_msgs::IOStates msg_in;
    ur_msgs::SetIO srv;

    const std::string HEADER="def gripper_action():\n socket_close(\"G\")\n sync()\n socket_open(\"127.0.0.1\",63352,\"G\")\n sync()\n";
    std::string FEEDBACK;//const std::string FEEDBACK=" set_configurable_digital_out(7,True)\n"; Now you can change the configurable output.
    const std::string ENDING=" socket_close(\"G\")\nend\n";

    void reset();

    bool wait();

    void publisher();

    void updater();

    void writeFeedback(int id);

    std::string writeSpeed();

    std::string writeForce();

    std::string writePose(int pose);

    std::string writeTimeOut();

    std::string writeWhile(int pose, int t);

    std::string writeWhileOpen();

    std::string writeWhileClose();

    bool flag;
    bool urs_cmd_initialized;
    int pinID;
    int speed;
    int force;
    int pose;
    int tolerance;
    int time_out;

    std::string urs_cmd;
    std::stringstream aux;


public:
    ur_robotiq_cmd();

    void gripper140_statusCallback(const ur_msgs::IOStates::ConstPtr& msg_in);

    void init();

    void open();

    void close();

    void moveto(int goal);

    void setSpeed(int s=255);

    void setForce(int f=0);

    void setPoseTolerance(int tol=0);

    void setTimeOut(int timeOut=8);

    void UR_feedback(int address=15);

    int getSpeed();

    int getForce();

    int getPose();

    int getPoseTolerance();

    int getTimeOut();

    int getCheckpointAddress();
};

#endif // UR_ROBOTIQ_CMD_H
