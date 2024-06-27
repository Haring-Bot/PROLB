#include "robot_class.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "my_ros_node");                                                               //Initializes node
    ros::NodeHandle nh("~");                                                                            //Initialize node in private namespace
    MyClass robot(nh);                                                                                  //Create object of class MyClass
    ros::Timer timer = nh.createTimer(ros::Duration(robot.pub_rate), &MyClass::timer_callback, &robot); //Create a ROS Timer
    ros::spin();                                                                                        //Single-threaded spinning
    return 0;
}
