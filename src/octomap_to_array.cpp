/*
This script contains a ROS node that receives Octomap messages over the topic 'octomap_full' and builds several representations of the octomap data in alternate formats.
    Marker message: A points message is built based on the occupied nodes in the octomap message.
    Multidimensional array: A binary 3D array is built based on the occupied and unoccpied cells in the octomap message. This is printed to a console, provided a console is available. Note: This does not work with a launch file right now.
    Occupancy message: This message is used to record the indexes of all points in the array representation that are 0 (not occupied). This message is then published on the topic 'output2'. This permits the array to be rebuilt in other nodes by implicitly encoding the status of all indices in the array. See Occupany.msg in the msgs directory for more information on using this message type.
*/

/* TODO
Test equivalency of the output arrays.
Test equivalency of the marker messages.
Test equivalency of the occupancy messages.
Remove the array based multidimensional array and all related code.

Implement a solution for octomap size changing based on "empty" pixels in the input image.
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
ros::Publisher pub3;


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


    // Loop through array and set all values to 0. Both unknown and empty space will be 0 this way.
    memset(arr, '\0', xmax*ymax*zmax);

    // Create index values
    long xind=0, yind=0, zind=0;
    // Create Occupancy message
    mill_controller::Occupancy oc_msg;
    mill_controller::Occupancy oc_msg2;

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
    // Clear the Rviz message for new data.
    rviz_msg.points.clear();


    double xcoord, ycoord, zcoord;

    // For each index coordinate in the vector array, set the coordinate based on octomap data.
    // For each depth
    for(int k=0; k < zmax; k++)
    {
        // For each row
        for(int j=0; j < ymax; j++)
        {
            // For each column
            for(int i=0; i < xmax; i++)
            {
                // Calculate corresponding node centerpoint in x,y,z coords
                xcoord = (i + 0.5)*res + xmin;
                ycoord = (j + 0.5)*res + ymin;
                zcoord = (k + 0.5)*res + zmin;

                // Search the octomap for a node at this centerpoint
                octomap::OcTreeNode* result;
                result = octree->search(xcoord, ycoord, zcoord);

                // If node exists
                if(result != NULL)
                {
                    // If node is unoccupied
                    if(!octree->isNodeOccupied(result))
                    {
                        // Change element to 0
                        arr2[i][j][k] = 0;

                        // Add the point to occupancy message
                        oc_msg2.column.push_back(xind);
                        oc_msg2.row.push_back(yind);
                        oc_msg2.layer.push_back(zind);
                    }
                    else // Node is occupied
                    {
                        // Add to Marker message
                        p.x = xcoord;
                        p.y = ycoord;
                        p.z = zcoord;
                        rviz_msg.points.push_back(p);
                    }
                }
                else // Assume node is unoccupied
                {
                    arr2[i][j][k] = 0;

                    // Add the point to occupancy message
                    oc_msg2.column.push_back(xind);
                    oc_msg2.row.push_back(yind);
                    oc_msg2.layer.push_back(zind);
                }
            }
        }
    }

    // Debug vector array print
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

    // Add the array dimensions to the message
    oc_msg.width = xmax;
    oc_msg.height = ymax;
    oc_msg.depth = zmax;

    // Time stamp marker message
    // ros::Time() Sets to time zero
    // If not displaying, try ros::Time::now()
    rviz_msg.header.stamp = ros::Time();

    // Publish messages
    pub.publish(rviz_msg);
    pub2.publish(oc_msg);

    if((oc_msg.column.size() != oc_msg2.column.size())||
        (oc_msg.row.size() != oc_msg2.row.size())||
        (oc_msg.layer.size() != oc_msg2.layer.size()))
    {
        printf("Size Mismatch!");
    }
    else
    {
        for(int i=0; i<oc_msg.column.size(); i++)
        {
            if(oc_msg.column[i] != oc_msg2.column[i])
            {
                printf("Column Mismatch!");
            }
        }
        for(int i=0; i<oc_msg.row.size(); i++)
        {
            if(oc_msg.row[i] != oc_msg2.row[i])
            {
                printf("Row Mismatch!");
            }
        }
        for(int i=0; i<oc_msg.layer.size(); i++)
        {
            if(oc_msg.layer[i] != oc_msg2.layer[i])
            {
                printf("Layer Mismatch!");
            }
        }
    }

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
    pub3 = nh.advertise<visualization_msgs::Marker>("output3", 1);
    pub2 = nh.advertise<mill_controller::Occupancy>("output2", 1);

    // Spin until shut down
    ros::spin();
}





