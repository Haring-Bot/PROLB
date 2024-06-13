
#include "robot_class.h"

MyClass::MyClass(ros::NodeHandle &N):NH(N){
    ROS_INFO_STREAM("Init My ROS Package");
    pub = NH.advertise< std_msgs::String >("pub_topic", 1 );
    check_params(NH);
}

MyClass::~MyClass( void ){}

void MyClass::my_callback( const std_msgs::String::ConstPtr& msg ) {
    ROS_INFO_STREAM( "Got Callback Message" );
 }

void MyClass::timer_callback(const ros::TimerEvent& event){
    ROS_INFO_STREAM( "Timer Callback" );
    my_message.data = "mymessage";
    pub.publish( my_message );
 }

void MyClass::check_params(ros::NodeHandle &N){
    ROS_INFO_STREAM( "Check Rate" );
    N.getParam("node_rate" , pub_rate);
    if(pub_rate > 10 || pub_rate < 1){
        pub_rate = 1;
    } 
    std::cout << "Publishing rate:" << pub_rate << std::endl;
 }
