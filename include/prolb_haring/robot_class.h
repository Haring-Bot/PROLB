
#ifndef ROBOT_CLASS_H_
#define ROBOT_CLASS_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "eigen3/Eigen/Dense"

class MyClass{
    public:
        MyClass(ros::NodeHandle &N);
        ~MyClass( void );
        void my_callback( const std_msgs::String::ConstPtr& msg);
        void timer_callback(const ros::TimerEvent& event);
        void check_params(ros::NodeHandle &N);
        int pub_rate;

    private:
        ros::NodeHandle NH;
        ros::Publisher pub;
        ros::Subscriber sub = NH.subscribe("sub_topic", 1, &MyClass::my_callback, this);
        std_msgs::String my_message;
        Eigen::Matrix<double, 3, 3> A;
};

#endif
