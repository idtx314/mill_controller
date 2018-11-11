// Includes
#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<visualization_msgs/Marker.h>
#include<geometry_msgs/Point.h>
#include<octomap_msgs/Octomap.h>
#include<octomap_msgs/conversions.h>
    /*
    Octomap messages
    */

/* TODO
figure out width, height, depth. May be backwards right now
set up package.xml
*/

visualization_msgs::Marker rviz_msg;
ros::Publisher pub;



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
    ROS_INFO("X: %f Y: %f Z: %f res: %f",x,y,z,res);
    octree->getMetricMin(xmin, ymin, zmin);
    ROS_INFO("Xm: %f Ym: %f Zm: %f",xmin,ymin,zmin);
    x = x/res;
    y = y/res;
    z = z/res;

    ROS_INFO("X: %f Y: %f Z: %f",x,y,z);
    ROS_INFO("X: %d Y: %d Z: %d",(int)(x+.000001),(int)(y+.000001),(int)(z+.000001));

    // Allocate memory for a 3 dimension char array with dimensions based on octomap sizes. Orientation is x to right, y down, z into page.
        // size of array dimension is maximum index + 1 || MetricSize/res

    char arr[(int)x][(int)y][(int)z];
    // char arr[128][96][1];

    // Loop through array and set all values to 0. Both unknown and empty space will be 0 this way.
    memset(arr, '\0', (int)x*(int)y*(int)z);

    // Create index values
    int xind=0, yind=0, zind=0;

    // for each leaf of octree (based on leaf iterators)
    // Segmentation fault ocurring in this loop, but not immediately
    int counter=0;
    ROS_INFO("Begin looping");
    for(octomap::OcTree::leaf_iterator it = octree->begin_leafs(),
        end = octree->end_leafs();
        it != end;
        ++it)
    {
        ROS_INFO("1");
        // convert the node center coords to integer indexes for the array.
            // index = (value-MetricMin)/res-0.5
        xind = (it.getX()-xmin)/res - 0.5;
        yind = (it.getY()-ymin)/res - 0.5;
        zind = (it.getZ()-zmin)/res - 0.5;

        ROS_INFO("2");
        counter++;
        ROS_INFO("Iteration number: %d",counter);
        ROS_INFO("\nXind: %d NodeX: %f\nYind: %d NodeY: %f\nZind: %d NodeZ: %f", xind,it.getX(),yind,it.getY(),zind,it.getZ());
        ROS_INFO("\nCombined Indices: %d",(xind+1)*(yind+1)*(zind+1));

        ROS_INFO("3");
        // if node is occupied
        ROS_INFO("Cell: %d",arr[xind][yind][zind]);

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
        }
        ROS_INFO("4");
    }
    ROS_INFO("Done Looping");

    // Output the array in some fashion
    geometry_msgs::Point p;

    // Print array
    for(int dep=0; dep<(int)z; dep++) //Cycle through depths
    {
        for(int row=0; row<(int)y; row++) //Cycle through rows
        {
            for(int col=0; col<int(x); col++) //Cycle through columns
            {
                // printf("%d ",arr[col][row][dep]);
                if(arr[col][row][dep])
                {
                    p.x = col*res;
                    p.y = row*res;
                    p.z = dep*res;
                    rviz_msg.points.push_back(p);
                }
            }
            // printf("\n");
        }
        // printf("\n");
    }



    rviz_msg.header.stamp = ros::Time(); //Sets to time zero, if not displaying try ros::Time::now()

    pub.publish(rviz_msg);

    // Finish callback



}





int main(int argc, char** argv)
{
    // Initilize variables
    rviz_msg.header.frame_id = "base";
    rviz_msg.id = 10;
    rviz_msg.type = visualization_msgs::Marker::POINTS;
    rviz_msg.pose.position.x = 0;
    rviz_msg.pose.position.y = 2;
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

    // Announce publisher
    pub = nh.advertise<visualization_msgs::Marker>("output", 1);

    // Spin until shut down
    ros::spin();
}





