// Includes
#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<sensor_msgs/Image.h>

// Declare publisher
ros::Publisher pub;


void callback(const sensor_msgs::Image &msg)
{
    printf("call");
}


int main(int argc, char **argv)
{
    // Initialize node and declare NodeHandle
    ros::init(argc,argv,"node_name");
    ros::NodeHandle nh;

    // Subscribe to topic with 1 message buffer
    ros::Subscriber sub = nh.subscribe("input_topic", 1, callback);

    // Announce publisher with 1 message buffer
    pub = nh.advertise<sensor_msgs::PointCloud2>("output_topic", 1);

    // Spin until shutdown
    ros::spin();

    return 0;
}
