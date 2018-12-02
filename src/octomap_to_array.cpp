/*
This script contains a ROS node that receives Octomap messages over the topic 'octomap_full' and builds several representations of the octomap data in alternate formats.
    Marker message: A points message is built based on the occupied nodes in the octomap message.
    Multidimensional array: A binary 3D array is built based on the occupied and unoccpied cells in the octomap message. This is printed to a console, provided a console is available. Note: This does not work with a launch file right now.
    Occupancy message: This message is used to record the indexes of all points in the array representation that are 0 (not occupied). This message is then published on the topic 'output2'. This permits the array to be rebuilt in other nodes by implicitly encoding the status of all indices in the array. See Occupany.msg in the msgs directory for more information on using this message type.
*/

/* TODO
figure out width, height, depth. May be backwards right now
set up package.xml
*/

// Includes
#include<cmath>
#include<ros/ros.h>
#include<mill_controller/Occupancy.h>
#include<sensor_msgs/PointCloud2.h>
#include<visualization_msgs/Marker.h>
#include<geometry_msgs/Point.h>
#include<octomap_msgs/Octomap.h>
#include<octomap_msgs/conversions.h>

// Declare Marker message and both publishers
visualization_msgs::Marker rviz_msg;
ros::Publisher pub;
ros::Publisher pub2;



void callback(const octomap_msgs::Octomap &msg)
{
    // Receive Octomap as octree
    octomap::AbstractOcTree* treeptr = octomap_msgs::msgToMap(msg);
    octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(treeptr);

    // Expand the tree, so that all leaves are at maximum depth.
    octree->expand();

    // get tree information
    size_t method_count = octree->getNumLeafNodes();
    double res = octree->getResolution();
    unsigned int max_depth = octree->getTreeDepth();

    // Determine x,y,z dimensions and translate into needed array size
    double x, y, z;
    double xmin, ymin, zmin;
    char s[100];
    octree->getMetricSize(x, y, z);
    octree->getMetricMin(xmin, ymin, zmin);
    x = x/res;
    y = y/res;
    z = z/res;
    // I need to convert x,y,z into integers to provide the size of the array. In hypothesis they will always be integers, but due to precision limits they will sometimes be very slightly lower than than the appropriate integer value. This causes them to be truncated to a lower value when converted.
    // So far I have been adding a small amount before casting, but this feels imprecise.
    // I want to come away with a number that is equal to or larger than the dimensions of the octomap
    // Can I assume that octomap sizes/res will always be an int?


    // Allocate memory for a 3 dimension char array with dimensions based on octomap size. Coordinate axes follow standard depth image protocol. From reference viewpoint: x to right, y down, z into image.
        // size of array dimension is maximum index + 1 || MetricSize/res
        // Add very small amount to eliminate float precision issue. Floats were being truncated to a number below their proper value.
    char arr[(int)(x+.000001)][(int)(y+.000001)][(int)(z+.000001)];

    // Loop through array and set all values to 0. Both unknown and empty space will be 0 this way.
    memset(arr, '\0', (int)(x+.000001)*(int)(y+.000001)*(int)(z+.000001));

    // Create index values
    int xind=0, yind=0, zind=0;
    // Create Occupancy message
    mill_controller::Occupancy oc_msg;

    // for each leaf of octree (based on leaf iterators)
    for(octomap::OcTree::leaf_iterator it = octree->begin_leafs(),
        end = octree->end_leafs();
        it != end;
        ++it)
    {
        // convert the node center coords to integer indexes for the array.
            // index = (value-MetricMin)/res-0.5
        xind = (it.getX()-xmin)/res - 0.499999;
        yind = (it.getY()-ymin)/res - 0.499999;
        zind = (it.getZ()-zmin)/res - 0.499999;

        // if node is occupied
        if(octree->isNodeOccupied(*it))
        {
            // set the char at those index values to 1
            arr[xind][yind][zind] = 1;
        }
        else
        {
            // set the char at those index values to 0
            // TODO This is not necessary if all values are initialized to 0.
            arr[xind][yind][zind] = 0;
            // Add the point to occupancy message
            oc_msg.column.push_back(xind);
            oc_msg.row.push_back(yind);
            oc_msg.layer.push_back(zind);
        }
    }

    // Declare a point object for recording Marker message information
    geometry_msgs::Point p;

    // Print array and populate marker message
    for(int dep=0; dep<(int)(z+.000001); dep++) //Cycle through depths
    {
        for(int row=0; row<(int)(y+.000001); row++) //Cycle through rows
        {
            for(int col=0; col<(int)(x+.000001); col++) //Cycle through columns
            {
                // Print value at this index to console
                printf("%d",arr[col][row][dep]);
                // If the index represents occupied space
                if(arr[col][row][dep])
                {
                    // Add to Marker message
                    p.x = col*res;
                    p.y = row*res;
                    p.z = dep*res;
                    rviz_msg.points.push_back(p);
                }
            }
            // Start new row in console
            printf("\n");
        }
        // Start new layer in console
        printf("\n\n");
    }

    // Add the array dimensions to the message
    oc_msg.width = (int)(x+.000001);
    oc_msg.height = (int)(y+.000001);
    oc_msg.depth = (int)(z+.000001);

    // Time stamp marker message
    rviz_msg.header.stamp = ros::Time(); //Sets to time zero, if not displaying try ros::Time::now()

    // Publish messages
    pub.publish(rviz_msg);
    pub2.publish(oc_msg);
}



int main(int argc, char** argv)
{
    std::lround(3.5);

    // Initilize marker message
    rviz_msg.header.frame_id = "base";
    rviz_msg.id = 10;
    rviz_msg.type = visualization_msgs::Marker::POINTS;
    rviz_msg.pose.position.x = 0;
    rviz_msg.pose.position.y = 0;
    rviz_msg.pose.position.z = 0;
    rviz_msg.pose.orientation.x = 0.0;
    rviz_msg.pose.orientation.y = 0.0;
    rviz_msg.pose.orientation.z = 0.0;
    rviz_msg.pose.orientation.w = 1.0;
    rviz_msg.scale.x = .01;
    rviz_msg.scale.y = .01;
    rviz_msg.scale.z = .01;
    rviz_msg.color.a = 1.0; // Don't forget to set the alpha!
    rviz_msg.color.r = 0.0;
    rviz_msg.color.g = 1.0;
    rviz_msg.color.b = 0.0;


    // Initialize ROS node
    ros::init(argc, argv, "octomap_to_array_node");
    ros::NodeHandle nh;

    // Subscribe to octomap topic
    ros::Subscriber sub = nh.subscribe("octomap_full", 1, callback);

    // Initialize publishers
    pub = nh.advertise<visualization_msgs::Marker>("output", 1);
    pub2 = nh.advertise<mill_controller::Occupancy>("output2", 1);

    // Spin until shut down
    ros::spin();
}





