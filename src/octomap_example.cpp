// Includes
#include<ros/ros.h>
#include<octomap_msgs/Octomap.h>
#include<octomap_msgs/conversions.h>
#include<iostream>
#include<typeinfo>




// Meet corey
// order groceries
// clean out lab

ros::Publisher pub;



void callback(const octomap_msgs::Octomap &msg)
{
    octomap::AbstractOcTree* treeptr = octomap_msgs::binaryMsgToMap(msg);
    octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(treeptr);
    octomap_msgs::Octomap msg2;

    std::cout << typeid(msg).name() << std::endl;
    if(msg.binary)
    {
        std::cout << "True" << std::endl;
        octomap_msgs::binaryMapToMsg(*octree, msg2);
    }
    else
    {
        std::cout << "False" << std::endl;
        octomap_msgs::fullMapToMsg(*octree, msg2);
    }

    pub.publish(msg2);

}




int main(int argc, char** argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "ros_octomap_test_node");
    ros::NodeHandle nh;

    // Subscribe to octomap topic
    ros::Subscriber sub = nh.subscribe("input", 1, callback);

    // Announce publisher
    pub = nh.advertise<octomap_msgs::Octomap>("output", 1);

    ros::spin();

    return 0;
}
