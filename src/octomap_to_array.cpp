// Includes
#include<ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include<octomap_msgs/Octomap.h>
#include<octomap_msgs/conversions.h>
    /*
    Octomap messages
    */

/* TODO
input topic
publisher message type, output topic
figure out width, height, depth
set up package.xml
*/




void callback(const octomap_msgs::Octomap &msg)
{
    // Receive Octomap

    // Allocate memory for a 3 dimenion char array of size length, height, width/8

    //Cycle through i, j, k as x,y,z of both octomap and array. If octomap cell is occupied, bit is 1, else bit is 0.
    // Using coordinates x columns (left to right), y rows (top to bottom), z depth(into plane)
    // Cycle columns
    // for(int x=0; x<width; x++)
    //     // Cycle rows
    //     for(int y=0; y<height; y++)
    //         // Cycle depths
    //         for(int z=0; z<depth; z++)
    //         {
    //             unsigned char byte = array[x][y][z/8]
    //             bool value = byte<<z%8 & 0b00000001
    //         }

    // Translate the array into some sort of message

    //Output the array
}





int main(int argc, char** argv)
{
    // Initilize variables

    // Initialize ROS node
    ros::init(argc, argv, "map_to_array_node");
    ros::NodeHandle nh;

    // Subscribe to octomap topic
    ros::Subscriber sub = nh.subscribe("input", 1, callback);

    // Announce publisher
    ros::Publisher pub = nh.advertise<octomap_msgs::Octomap>("output", 1);

    // Spin until shut down
    ros::spin();
}





