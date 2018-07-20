// Includes
#include<ros/ros.h>
#include<octomap_msgs/Octomap.h>
#include<octomap_msgs/conversions.h>

// DEBUG
#include<iostream>
#include<typeinfo>




ros::Publisher pub;



void callback(const octomap_msgs::Octomap &msg)
{
    // The de-serialization produces a very general data type
    octomap::AbstractOcTree* treeptr = octomap_msgs::msgToMap(msg);
    // This can be cast to other types. An octree seems to be appropriate for publishing
    octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(treeptr);
    // Make a new octomap message to publish
    octomap_msgs::Octomap msg2;

    // Create some variables and collect some data from the tree
    double res = octree->getResolution();
    unsigned int max_depth = octree->getTreeDepth();
    int count = 0;
    int inner_count = 0;
    size_t method_count = octree->getNumLeafNodes();

    // // Cycle through leaf nodes and count. Count leaf nodes not at max depth separately.
    // for(octomap::OcTree::leaf_iterator it = octree->begin_leafs(),
    //     end = octree->end_leafs();
    //     it != end;
    //     ++it)
    // {
    //     count++;
    //     if(it.getDepth() != max_depth)
    //     {
    //         inner_count++;
    //     }
    // }


    // // Print out data for the unexpanded Octree
    // std::cout << "Res: " << res << std::endl << "Depth: " << max_depth << std::endl;
    // std::cout << "Number of counted leaves: " << count << std::endl;
    // std::cout << "Number of property leaves: " << method_count << std::endl;
    // std::cout << "Number of inner leaves: " << inner_count << std::endl;

    // Expand the tree, so that all leaves are at maximum depth.
    octree->expand();

    // Reset the counters
    count = 0;
    inner_count = 0;
    method_count = octree->getNumLeafNodes();

    // Cycle through the leaves of the tree and count them. Count any leaves that aren't at maximum depth.
    for(octomap::OcTree::leaf_iterator it = octree->begin_leafs(),
        end = octree->end_leafs();
        it != end;
        ++it)
    {
        count++;
        if(it.getDepth() != max_depth)
            inner_count++;
    }

    // Print out the number of leaves counted, the number returned by the tree method getNumLeafNodes() and the number of leaves not at maximum depth.
    std::cout << "Number counted after expanding: " << count << std::endl;
    std::cout << "Number of property after expanding: " << method_count << std::endl;
    std::cout << "Number of inner after expanding: " << inner_count << std::endl;


/*
    // Print out some data type information
    // std::cout << typeid(msg).name() << std::endl << typeid(msg2).name() << std::endl;

    // Translate octree back into a ROS message
    if(msg.binary)
    {
        std::cout << "True" << std::endl;
        // Pass in octree and message. These are automatically passed in as references?
        octomap_msgs::binaryMapToMsg(*octree, msg2);
    }
    else
    {
        std::cout << "False" << std::endl;
        octomap_msgs::fullMapToMsg(*octree, msg2);
    }


    // Publish the octree over ROS
    pub.publish(msg2);
*/
}




int main(int argc, char** argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "ros_octomap_test_node");
    ros::NodeHandle nh;

    // Subscribe to octomap topic
    ros::Subscriber sub = nh.subscribe("octomap_full", 1, callback);

    // Announce publisher
    pub = nh.advertise<octomap_msgs::Octomap>("octomap_conversion_test", 1);

    ros::spin();

    return 0;
}
