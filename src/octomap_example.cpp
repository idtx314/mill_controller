// Includes
#include<ros/ros.h>
#include <octomap_msgs/Octomap.h>







void callback(const octomap_msgs::OctomapConstPtr & msg)
{

}




int main(int argc, char** argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "ros_octomap_test_node");
    ros::NodeHandle nh;

    // Subscribe to octomap topic
    ros::Subscriber sub = nh.subscribe("input", 1, callback);

    // Announce publisher
    ros::Publisher pub = nh.advertise<octomap_msgs::Octomap>("output", 1);

    ros::spin();

    return 0;
}
