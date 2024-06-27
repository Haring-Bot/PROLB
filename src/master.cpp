#include <iostream>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

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

int main(int argc, char** argv) {
    std::cout << "starting up\n";
    ros::init(argc, argv, "my_node");  // Initialize ROS
    ros::NodeHandle nh;

    testMsgs myTest(nh);
    controlRobot controller(nh);

    while(ros::ok()){
        myTest.publisher("test");
        controller.control(1.0, 0);
        std_msgs::String received = myTest.subscriber(10);
        ROS_INFO("Received message: %s", received.data.c_str());
        ros::spinOnce();
    }


    return 0;
}
