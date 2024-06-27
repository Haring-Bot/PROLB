#include "ros/ros.h"
#include "std_msgs/String.h"

void callbackRcv(const std_msgs::String::ConstPtr& msg){
    ROS_INFO("I received: [%s]", msg->data.c_str());
}

void talkToMe(int argc, char **argv, std::string msg){
    ros::init(argc, argv, "publisherTest");
    ros::NodeHandle n;
    ros::Publisher testPub = n.advertise<std_msgs::String>("testSubject", 1000);
    ros::Rate loop_rate(10);
    int i = 0;
    while(ros::ok()){
        std_msgs::String pblMsg;
        std::stringstream ss;
        ss << msg << i;
        pblMsg.data = ss.str();
        ROS_INFO("%s", pblMsg.data.c_str());
        testPub.publish(pblMsg);  // Publish std_msgs::String, not std::string
        ros::spinOnce();
        loop_rate.sleep();
        i++;
    }
}

std_msgs::String listenToMe(int argc, char **argv){
    std_msgs::String msg;
    ros::init(argc, argv, "subscriberTest");
    ros::NodeHandle n;
    ros::Subscriber testSub = n.subscribe("testSubject", 1000, callbackRcv);
    //msg = msg;
    ros::spin();


    //return msg;
}


int main(int argc, char** argv){
    std::cout << "starting up\n";
    talkToMe(argc, argv, "This is a test message");
    listenToMe(argc, argv);                                                                                        //Single-threaded spinning
    return 0;
}

