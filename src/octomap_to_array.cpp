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
#include<vector>
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
    long xmax, ymax, zmax;
    char s[100];
    octree->getMetricSize(x, y, z);
    octree->getMetricMin(xmin, ymin, zmin);
    x = x/res;
    y = y/res;
    z = z/res;

    xmax = std::lround(x);
    ymax = std::lround(y);
    zmax = std::lround(z);


    // Allocate memory for a 3 dimension char array with dimensions based on octomap size. Coordinate axes follow standard depth image protocol. From reference viewpoint: x to right, y down, z into image.
        // size of array dimension is maximum index + 1 || MetricSize/res
        // Add very small amount to eliminate float precision issue. Floats were being truncated to a number below their proper value.
    char arr[xmax][ymax][zmax];

    // Create vector of vectors to hold occupancy data.
    std::vector<std::vector<std::vector<char>>> arr2;

    // Set dimensions of array.
    arr2.resize(xmax);
    for(int i=0; i < xmax; i++)
    {
        arr2[i].resize(ymax);
        for(int j=0; j < ymax; j++)
        {
            arr2[i][j].resize(zmax);
            // Set contents of each vector to 1
            arr2[i][j].assign(zmax,1);
        }
    }

    for(int k=0; k < zmax; k++)
    {
        for(int j=0; j < ymax; j++)
        {
            for(int i=0; i < xmax; i++)
            {
                printf("%d",arr2[i][j][k]);
            }
            printf("\n");
        }
        printf("\n\n");
    }
    printf("%ld %ld %ld\n",xmax,ymax,zmax);


    // Loop through array and set all values to 0. Both unknown and empty space will be 0 this way.
    memset(arr, '\0', xmax*ymax*zmax);

    // Create index values
    long xind=0, yind=0, zind=0;
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
        xind = std::lround((it.getX()-xmin)/res - 0.5);
        yind = std::lround((it.getY()-ymin)/res - 0.5);
        zind = std::lround((it.getZ()-zmin)/res - 0.5);

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
    for(int dep=0; dep<zmax; dep++) //Cycle through depths
    {
        for(int row=0; row<ymax; row++) //Cycle through rows
        {
            for(int col=0; col<xmax; col++) //Cycle through columns
            {
                // Print value at this index to console
                // printf("%d",arr[col][row][dep]);
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
            // printf("\n");
        }
        // Start new layer in console
        // printf("\n\n");
    }

    // Add the array dimensions to the message
    oc_msg.width = xmax;
    oc_msg.height = ymax;
    oc_msg.depth = zmax;

    // Time stamp marker message
    rviz_msg.header.stamp = ros::Time(); //Sets to time zero, if not displaying try ros::Time::now()

    // Publish messages
    pub.publish(rviz_msg);
    pub2.publish(oc_msg);
}



int main(int argc, char** argv)
{

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





