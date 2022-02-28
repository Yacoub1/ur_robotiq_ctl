#include "../include/robotiq_140_control/ur_robotiq_cmd.h"

ur_robotiq_cmd::ur_robotiq_cmd(){
    /*
     * robotiq_cmd::ur_robotiq_cmd() initialize ros publisher/subscriber
     * msg_type: URdriver/URScript
    */
    pub = nh.advertise<std_msgs::String>("/ur_driver/URScript",5);
    sub = nh.subscribe("/ur_driver/io_states",10, &ur_robotiq_cmd::gripper140_statusCallback,this);
    client = nh.serviceClient<ur_msgs::SetIO>("/ur_driver/set_io");
    ros::Duration(1.0).sleep();
    UR_feedback();
    setTimeOut();
    setPoseTolerance();
    setSpeed();
    setForce();
    flag = 0;
    urs_cmd_initialized = 0;
    urs_cmd = "";

}

void ur_robotiq_cmd::gripper140_statusCallback(const ur_msgs::IOStates::ConstPtr& msg_in){
    /*
     * ur_robotiq_cmd::gripper140_statusCallback() is robotiq_cmd::ur_robotiq_cmd() Callback,
     * this function reads updates from UR controller and use it as a feedback from the UR controller.
    */
    flag = msg_in->digital_out_states[pinID].state;
}

void ur_robotiq_cmd::init(){
    /*
     * ur_robotiq_cmd::init() initialize communication with gripper
     * after execution gripper will open
    */
    reset();
    ROS_WARN("gripper initialization");
    urs_cmd = HEADER;
    urs_cmd+=	" socket_set_var(\"ACT\",1,\"G\")\n";
    urs_cmd+= " sync()\n";
    urs_cmd+= " t=0\n";
    urs_cmd+= " while (socket_get_var(\"STA\",\"G\")!=3 and t<25):\n";
    urs_cmd+= "  sync()\n";
    urs_cmd+= "  sleep(0.25)\n";
    urs_cmd+= "  t=t+1\n";
    urs_cmd+= " end\n";
    urs_cmd+= " if(t>=25):\n";
    urs_cmd+= "  popup(\"Something went wrong with gripper activation! TIME OUT\")\n";
    urs_cmd+= "  halt\n";
    urs_cmd+= " end\n";
    urs_cmd+= " socket_set_var(\"GTO\",1,\"G\")\n";
    urs_cmd+= " sync()\n";
    urs_cmd+= " socket_set_var(\"SPE\",255,\"G\")\n";
    urs_cmd+= " sync()\n";
    urs_cmd+= " socket_set_var(\"FOR\",0,\"G\")\n";
    urs_cmd+= " sync()\n";

    urs_cmd+= " socket_set_var(\"POS\",0,\"G\")\n";
    urs_cmd+= " sync()\n";
    urs_cmd+= " t=0\n";
    urs_cmd+= " while (socket_get_var(\"POS\",\"G\")>3 and t<25):\n";
    urs_cmd+= "  sync()\n";
    urs_cmd+= "  sleep(0.25)\n";
    urs_cmd+= "  t=t+1\n";
    urs_cmd+= " end\n";
    urs_cmd+= " if(t>=25):\n";
    urs_cmd+= "  popup(\"Something went wrong with gripper activation! TIME OUT\")\n";
    urs_cmd+= "  halt\n";
    urs_cmd+= " end\n";

    urs_cmd+= FEEDBACK;
    urs_cmd+= ENDING;
    publisher();
    if(wait()){
        ROS_INFO("URS commander have been initialized");
        urs_cmd_initialized = 1;
    }
    updater();
}

void ur_robotiq_cmd::open(){
    /*
     * ur_robotiq_cmd::open(): open gripper with maximum speed 250 mm/s
     *  == 0.25 cm/s with force specified through writeForce()
    */
    if(!urs_cmd_initialized){
        ROS_WARN("Initialize gripper first");
        goto end;
    }
    urs_cmd=HEADER;
    urs_cmd+= writeSpeed();
    urs_cmd+= " sync()\n";
    urs_cmd+= writeForce();
    urs_cmd+= " sync()\n";
    urs_cmd+= " socket_set_var(\"POS\",0,\"G\")\n";
    urs_cmd+= " sync()\n";
    urs_cmd+= " t=0\n";
    urs_cmd+= writeWhileOpen();
    urs_cmd+= "  sync()\n";
    urs_cmd+= "  sleep(0.25)\n";
    urs_cmd+= "  t=t+1\n";
    urs_cmd+= " end\n";
    urs_cmd+= writeTimeOut();
    urs_cmd+= "  popup(\"TIME OUT\")\n";
    urs_cmd+= "  halt\n";
    urs_cmd+= " end\n";
    urs_cmd+= FEEDBACK;
    urs_cmd+= ENDING;
    publisher();
    if(wait())
        ROS_INFO("Gripper succesfully opened");
    updater();

    end:;
}

void ur_robotiq_cmd::close(){
    /*
     * ur_robotiq_cmd::close(): close gripper with speed specified using writeSpeeed()
     * with force specified through writeForce()
    */
    if(!urs_cmd_initialized){
        ROS_WARN("Initialize gripper first");
        goto end;
    }
    urs_cmd=HEADER;
    urs_cmd+= writeSpeed();
    urs_cmd+= " sync()\n";
    urs_cmd+= writeForce();
    urs_cmd+= " sync()\n";
    urs_cmd+= " socket_set_var(\"POS\",255,\"G\")\n";
    urs_cmd+= " sync()\n";
    urs_cmd+= " t=0\n";
    urs_cmd+= writeWhileClose();
    urs_cmd+= "  sync()\n";
    urs_cmd+= "  sleep(0.25)\n";
    urs_cmd+= "  t=t+1\n";
    urs_cmd+= " end\n";
    urs_cmd+= writeTimeOut();
    urs_cmd+= "  popup(\"TIME OUT\")\n";
    urs_cmd+= "  halt\n";
    urs_cmd+= " end\n";
    urs_cmd+= FEEDBACK;
    urs_cmd+= ENDING;
    publisher();
    if(wait())
        ROS_INFO("Gripper succesfully closed");
    updater();

    end:;
}

void ur_robotiq_cmd::moveto(int goal){
    /*
     * ur_robotiq_cmd::moveto(int goal): move gripper to a predefied pose (goal)
     * with speed specified using writeSpeeed(), force specified through writeForce()
    */
    if(!urs_cmd_initialized){
        ROS_WARN("Initialize gripper first");
        goto end;
    }
    urs_cmd=HEADER;
    urs_cmd+= writeSpeed();
    urs_cmd+= " sync()\n";
    urs_cmd+= writeForce();
    urs_cmd+= " sync()\n";
    urs_cmd+= writePose(goal);
    urs_cmd+= " sync()\n";
    urs_cmd+= " t=0\n";
    urs_cmd+= writeWhile(goal,tolerance);
    urs_cmd+= "  sync()\n";
    urs_cmd+= "  sleep(0.25)\n";
    urs_cmd+= "  t=t+1\n";
    urs_cmd+= " end\n";
    urs_cmd+= writeTimeOut();
    urs_cmd+= "  popup(\"TIME OUT\")\n";
    urs_cmd+= "  halt\n";
    urs_cmd+= " end\n";
    urs_cmd+= FEEDBACK;
    urs_cmd+= ENDING;
    publisher();
    if(wait()){
        if(goal>255)
            ROS_INFO("Gripper pose goal: %i reached",255);
        else if(goal<0)
            ROS_INFO("Gripper pose goal: %i reached",0);
        else
            ROS_INFO("Gripper pose goal: %i reached",goal);
    }
    updater();

    end:;
}
/*
 * Functions bellow are Setter/getters functions for the robotiq private variables
*/

void ur_robotiq_cmd::setSpeed(int s){
    /*
     * Speed for 2f140 is between 30-250 mm/s in the URS command in m/s
*/
    if(s>250)
        speed = 250;
    else if(s<0)
        speed = 0;
    else
        speed = s;
    ROS_INFO("Gripper closing speed set to ->%i",speed);
}

void ur_robotiq_cmd::setForce(int f){
    /*
     * Adjustable grip force for 2f140 is between 10-125 N
*/
    if(f>125)
        force = 125;
    else if(f<0)
        force = 0;
    else
        force = f;
    ROS_INFO("Gripper force set to ->%i",force);
}

void ur_robotiq_cmd::setPoseTolerance(int tol){
    tolerance = tol;
    ROS_INFO("Pose Tolerance ->%i",tolerance);
}

void ur_robotiq_cmd::setTimeOut(int timeOut){
    if(timeOut>0)
        time_out = timeOut/0.25;
    else
        time_out = 8;
    ROS_INFO("Gripper's time-out set to %i seconds",time_out);
}

void ur_robotiq_cmd::UR_feedback(int address){
    /*
     *
    */
    if(address>15 || address<8){
        srv.request.fun = 1;
        srv.request.pin = 15;
        srv.request.state = 0;
        pinID=15;
        writeFeedback(pinID);
        ROS_WARN("Digital IO outputs IDs must between 8 to 15");
    }
    else{
        srv.request.fun = 1;
        srv.request.pin = address;
        srv.request.state = 0;
        pinID=address;
        writeFeedback(pinID);
        if(pinID!=15)
            ROS_INFO("Digital output pin changed to %i", address);
        else
            ROS_INFO("Default Checkpoint pin is used");
    }
}

int ur_robotiq_cmd::getSpeed(){
    return speed;
}

int ur_robotiq_cmd::getForce(){
    return force;
}

int ur_robotiq_cmd::getPoseTolerance(){
    return tolerance;
}

int ur_robotiq_cmd::getPose(){
    return pose;
}
int ur_robotiq_cmd::getTimeOut(){
    return time_out*0.25;
}

int ur_robotiq_cmd::getCheckpointAddress(){
    return pinID;
}


void ur_robotiq_cmd::reset(){
    urs_cmd= HEADER;
    urs_cmd+= " socket_set_var(\"ACT\",0,\"G\")\n";
    urs_cmd+= " sync()\n";
    /*urs_cmd+= " socket_set_var(\"GTO\",0,\"G\")\n"; Works better without this.
    urs_cmd+= " sync()\n";
    urs_cmd+= " socket_set_var(\"SPE\",0,\"G\")\n";
    urs_cmd+= " sync()\n";
    urs_cmd+= " socket_set_var(\"FOR\",0,\"G\")\n";
    urs_cmd+= " sync()\n";*/
    //urs_cmd+= " socket_set_var(\"POS\",0,\"G\")\n";
    //urs_cmd+= " socket_set_var(\"PRE\",0,\"G\")\n";
    urs_cmd+= " sync()\n";
    urs_cmd+= " sleep(0.5)\n";
    urs_cmd+= FEEDBACK;
    urs_cmd+= ENDING;
    publisher();
    if(wait())
        urs_cmd_initialized = 0;
    updater();
}

bool ur_robotiq_cmd::wait(){
    int time=0;
    while(!flag && ros::ok() && time<time_out){
        ros::spinOnce();
        ros::Duration(0.25).sleep();
        time++;
    }
    if(time>=time_out){
        ROS_ERROR("urs_cmd TIME OUT");
        return 0;
    }
    return 1;
}

void ur_robotiq_cmd::publisher(){
    msg_out.data = urs_cmd;
    pub.publish(msg_out);
    urs_cmd="";
}

void ur_robotiq_cmd::updater(){
    if (!client.call(srv))
        ROS_ERROR("Unable to update controller data");
    ros::Duration(0.5).sleep();
    ros::spinOnce();
}

void ur_robotiq_cmd::writeFeedback(int id){
    aux.str("");
    aux <<" set_configurable_digital_out("<<id-8<<",True)\n";
    FEEDBACK = aux.str();
}

std::string ur_robotiq_cmd::writeSpeed(){
    aux.str("");
    aux << " socket_set_var(\"SPE\","<<speed<<",\"G\")\n";
    return aux.str();
}

std::string ur_robotiq_cmd::writeForce(){
    aux.str("");
    aux << " socket_set_var(\"FOR\","<<force<<",\"G\")\n";
    return aux.str();
}

std::string ur_robotiq_cmd::writePose(int pose){
    aux.str("");
    if(pose>255){
        aux << " socket_set_var(\"POS\","<<255<<",\"G\")\n";
        ROS_WARN("Setting max valid position goal 255");
    }
    else if(pose<0){
        aux << " socket_set_var(\"POS\","<<0<<",\"G\")\n";
        ROS_WARN("Setting min valid position goal 0");
    }
    else
        aux << " socket_set_var(\"POS\","<<pose<<",\"G\")\n";
    return aux.str();
}

std::string ur_robotiq_cmd::writeTimeOut(){
    aux.str("");
    aux << " if(t>="<<time_out<<"):\n";
    return aux.str();
}

std::string ur_robotiq_cmd::writeWhileOpen(){
    aux.str("");
    aux << " while (socket_get_var(\"POS\",\"G\")>3 and socket_get_var(\"OBJ\",\"G\")!=1 and socket_get_var(\"OBJ\",\"G\")!=2 and t<"<<time_out<<"):\n";
    return aux.str();
}

std::string ur_robotiq_cmd::writeWhileClose(){
    aux.str("");
    aux << " while (socket_get_var(\"POS\",\"G\")<227 and socket_get_var(\"OBJ\",\"G\")!=1 and socket_get_var(\"OBJ\",\"G\")!=2 and t<"<<time_out<<"):\n";
    return aux.str();
}

std::string ur_robotiq_cmd::writeWhile(int pose, int t){
    aux.str("");
    if(pose>230)
        aux << " while (socket_get_var(\"POS\",\"G\")<"<<227-t<<" and socket_get_var(\"OBJ\",\"G\")!=1 and socket_get_var(\"OBJ\",\"G\")!=2 and t<"<<time_out<<"):\n";
    else if(pose<3)
        aux << " while (socket_get_var(\"POS\",\"G\")>"<<3+t<<" and socket_get_var(\"OBJ\",\"G\")!=1 and socket_get_var(\"OBJ\",\"G\")!=2 and t<"<<time_out<<"):\n";
    else
        aux << " while ((socket_get_var(\"POS\",\"G\")<"<<pose-t<<" or socket_get_var(\"POS\",\"G\")>"<<pose+t<<") and socket_get_var(\"OBJ\",\"G\")!=1 and socket_get_var(\"OBJ\",\"G\")!=2 and t<"<<time_out<<"):\n";
    return aux.str();
}

