/*
This script contains a ROS node that receives Octomap messages over the topic 'octomap_full' and builds several representations of the octomap data in alternate formats.
    Marker message: A points message is built based on the occupied nodes in the octomap message. This is published on the topic "octomap_to_cloud" and should match the node centers pointcloud published by octomap itself.
    Multidimensional array: A binary 3D array is built based on the occupied and unoccpied cells in the octomap message. This is printed to a console, provided a console is available. Note: This does not work with a launch file right now.
    Occupancy message: This message is used to record the indexes of all points in the array representation that are 0 (not occupied). This message is then published on the topic 'octomap_to_occupancy'. This permits the array to be rebuilt in other nodes by implicitly encoding the status of all indices in the array. See Occupany.msg in the msgs directory for more information on using this message type.
*/

/* TODO
Implement a solution for octomap size changing based on "empty" pixels in the input image.
Set the region of interest to encompass the appropriate material/image dimensions
Consider switching from index math to incrementing by resolution when determining xcoord, ycoord, and zcoord.
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
    double res = octree->getResolution();

    // Determine x,y,z dimensions and translate into needed array size
    double x, y, z;
    double xmin, ymin, zmin;
    long xmax, ymax, zmax;
    octree->getMetricSize(x, y, z);
    octree->getMetricMin(xmin, ymin, zmin);
    // Convert from measurement size to node numbers.
    x = x/res;
    y = y/res;
    z = z/res;

    // Round and convert to integers.
    xmax = std::lround(x);
    ymax = std::lround(y);
    zmax = std::lround(z);


    // Create a 3 layer vector array to hold occupancy data.
    std::vector<std::vector<std::vector<char>>> arr;

    // Set dimensions of array. Coordinate axes follow standard depth image protocol. From reference viewpoint: x to right, y down, z into image.
    // size of array dimension is maximum index + 1 || MetricSize/res
    arr.resize(xmax);
    for(int i=0; i < xmax; i++)
    {
        arr[i].resize(ymax);
        for(int j=0; j < ymax; j++)
        {
            arr[i][j].resize(zmax);
            // Set contents of each vector to 1
            arr[i][j].assign(zmax,1);
        }
    }

    // Create Occupancy message
    mill_controller::Occupancy oc_msg;

    // Declare a point object for recording Marker message information
    geometry_msgs::Point p;
    // Clear the Rviz message for new data.
    rviz_msg.points.clear();


    double xcoord, ycoord, zcoord;

    // Cycle through indexes in the vector array and set their value based on octomap data.
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
                        arr[i][j][k] = 0;

                        // Add the indexes to occupancy message
                        oc_msg.column.push_back(i);
                        oc_msg.row.push_back(j);
                        oc_msg.layer.push_back(k);
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
                    arr[i][j][k] = 0;

                    // Add the indexes to occupancy message
                    oc_msg.column.push_back(i);
                    oc_msg.row.push_back(j);
                    oc_msg.layer.push_back(k);
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
                printf("%d",arr[i][j][k]);
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
    pub = nh.advertise<visualization_msgs::Marker>("octomap_to_cloud", 1);
    pub2 = nh.advertise<mill_controller::Occupancy>("octomap_to_occupancy", 1);

    // Spin until shut down
    ros::spin();
}





