// Includes
#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<sensor_msgs/Image.h>
#include<cv_bridge/cv_bridge.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>

//TODO
/*
Probably don't need to copy image
*/

// Declare publisher
ros::Publisher pub;

// Declare image pointer
cv_bridge::CvImagePtr cv_ptr;


void callback(const sensor_msgs::Image &msg)
{
    // Accepts an image message, generates a pointcloud from it, translates that pointcloud into a pointcloud message, and publishes it.

    try
    {
        // Returns a cv::Mat
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    ROS_INFO("Received");
    cv::Vec3b intensity = cv_ptr->image.at<cv::Vec3b>(400, 600); // Row, Col

    ROS_INFO("Val: %d  Col: %d  Row: %d", intensity.val[0], cv_ptr->image.cols, cv_ptr->image.rows);


    pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ basic_point;



    // Translate and publish the cv_image
    // pub.publish(cv_ptr->toImageMsg());

}


int main(int argc, char **argv)
{
    // Initialize node and declare NodeHandle
    ros::init(argc,argv,"node_name");
    ros::NodeHandle nh;

    // Subscribe to topic with 1 message buffer
    ros::Subscriber sub = nh.subscribe("processed_image", 1, callback);

    // Announce publisher with 1 message buffer
    pub = nh.advertise<sensor_msgs::PointCloud2>("output_topic", 1);

    // Spin until shutdown
    ros::spin();

    return 0;
}
