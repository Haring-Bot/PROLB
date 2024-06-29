#include <iostream>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include <rosgraph_msgs/Clock.h>

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
        odomSub = nh_.subscribe("/odom", 10, &receiveData::callbackOdom, this);
        mapSub = nh_.subscribe("/map", 10, &receiveData::callbackMap, this);
    }
    ~receiveData(){}

    sensor_msgs::LaserScan receiveLaser(){
        return latestLaser;
    }

    nav_msgs::Odometry receiveOdom(){
        return latestOdom;
    }

    nav_msgs::OccupancyGrid receiveMap(){
        return latestMap;
    }

    private:
    ros::NodeHandle nh_;
    ros::Subscriber laserSub;
    ros::Subscriber odomSub;
    ros::Subscriber mapSub;
    sensor_msgs::LaserScan latestLaser;
    nav_msgs::Odometry latestOdom;
    nav_msgs::OccupancyGrid latestMap;
    
    void callbackLaser(const sensor_msgs::LaserScan::ConstPtr& msg){
        latestLaser = *msg;
    }

    void callbackOdom(const nav_msgs::Odometry::ConstPtr& msg){
        latestOdom = *msg;
    }

    void callbackMap(const nav_msgs::OccupancyGrid::ConstPtr& msg){
        latestMap = *msg;
    }
};

class testMsgs {
    public:
    testMsgs(ros::NodeHandle& nh) : nh_(nh) {
        testPub = nh_.advertise<std_msgs::String>("testSubject", 1000);
        testSub = nh_.subscribe("testSubject", 1000, &testMsgs::callbackRcv, this);
        clockPub = nh_.advertise<rosgraph_msgs::Clock>("/clock", 1);
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

    void pubClock(){
        curTime.clock = ros::Time::now();
        clockPub.publish(curTime);
    }

    std_msgs::String subscriber() {
        return receivedMsg;  // Return the received message
    }

    private:
    ros::NodeHandle nh_;
    ros::Publisher testPub;
    ros::Publisher clockPub;
    ros::Subscriber testSub;
    std_msgs::String receivedMsg;  // Store received message
    rosgraph_msgs::Clock curTime;

    int i = 0;
};

class EKFL{
    public:
    EKFL(){};
    ~EKFL(){};

};

int main(int argc, char** argv) {
    std::cout << "starting up\n";
    ros::init(argc, argv, "CPU");  // Initialize ROS
    ros::NodeHandle nh;
    ros::Rate loopRate(10);
    size_t round = 0;

    //testMsgs myTest(nh);
    controlRobot controller(nh);
    receiveData receiver(nh);
    testMsgs misc(nh);

    while(ros::ok()){
        ros::spinOnce();
        round++;
        //myTest.publisher("test");
        controller.control(-1, 0);
        //std_msgs::String received = myTest.subscriber(10);
        //ROS_INFO("Received message: %s", received.data.c_str());
        sensor_msgs::LaserScan latestLaser = receiver.receiveLaser();
        if (!latestLaser.ranges.empty()) {
            // Log laser scan data
            ROS_INFO("\nthis is round no [%zu]", round);
            ROS_INFO("Received laser data: [%f]", latestLaser.ranges[1]);
        }

        nav_msgs::Odometry latestOdom = receiver.receiveOdom();     //velocity
        ROS_INFO("Received odom x: [%f]", latestOdom.twist.twist.linear.x);
        misc.pubClock();
        loopRate.sleep();
    }
    return 0;
}
