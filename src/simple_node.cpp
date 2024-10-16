#include <ros/ros.h>

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "simple_node");
    ros::NodeHandle nh;

    // Print the message to the ROS console
    ROS_INFO("roslaunch started");

    // Keep the node alive
    ros::spin();
    
    return 0;
}
