#include <iostream>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <sensor_msgs/LaserScan.h>

class controlRobot {
    public:
    controlRobot(ros::NodeHandle& nh) : nh_(nh) {
        robotController = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    }

    void control(float cmdVel, float cmdAng) {
        geometry_msgs::Twist moveCmd;
        moveCmd.linear.x = cmdVel;
        moveCmd.angular.z = cmdAng;

        // Assuming robotController is a member of controlRobot
        robotController.publish(moveCmd);
    }

    private:
    ros::NodeHandle nh_;
    ros::Publisher robotController;  // Define your publisher here
};

class receiveData{
    public:
    receiveData(ros::NodeHandle& nh) : nh_(nh){
        laserSub = nh_.subscribe("/scan", 10, &receiveData::callbackLaser, this);
    }
    ~receiveData(){}

    sensor_msgs::LaserScan receiveLaser(ros::Rate freq){
        ros::spinOnce;
        freq.sleep();
        return msg;
    }

    void callbackLaser(const sensor_msgs::LaserScan::ConstPtr& msg){
        ROS_INFO("I received: [%s]", msg->data.c_str());
    }

    private:
    ros::NodeHandle nh_;
    ros::Subscriber laserSub;
};

class testMsgs {
    public:
    testMsgs(ros::NodeHandle& nh) : nh_(nh) {
        testPub = nh_.advertise<std_msgs::String>("testSubject", 1000);
        testSub = nh_.subscribe("testSubject", 1000, &testMsgs::callbackRcv, this);
    }

    void callbackRcv(const std_msgs::String::ConstPtr& msg) {
        ROS_INFO("I received: [%s]", msg->data.c_str());
    }

    void publisher(std::string msg) {
        std_msgs::String pblMsg;
        std::stringstream ss;
        ss << msg << i;
        pblMsg.data = ss.str();
        ROS_INFO("%s", pblMsg.data.c_str());
        testPub.publish(pblMsg);
        i++;
    }

    std_msgs::String subscriber(ros::Rate freq) {
        ros::spinOnce();  // Typically you would spin to handle callbacks
        freq.sleep();
        return receivedMsg;  // Return the received message
    }

    private:
    ros::NodeHandle nh_;
    ros::Publisher testPub;
    ros::Subscriber testSub;
    std_msgs::String receivedMsg;  // Store received message
    int i = 0;
};

class EKFL{
    public:
    EKFL(){};
    ~EKFL(){};

}

int main(int argc, char** argv) {
    std::cout << "starting up\n";
    ros::init(argc, argv, "CPU");  // Initialize ROS
    ros::NodeHandle nh;

    //testMsgs myTest(nh);
    controlRobot controller(nh);

    while(ros::ok()){
        //myTest.publisher("test");
        controller.control(0.2, 0);
        //std_msgs::String received = myTest.subscriber(10);
        //ROS_INFO("Received message: %s", received.data.c_str());
        ros::spinOnce();
    }


    return 0;
}
