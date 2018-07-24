// Includes
#include<ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include<octomap_msgs/Octomap.h>
#include<octomap_msgs/conversions.h>
    /*
    Octomap messages
    */

/* TODO
figure out width, height, depth. May be backwards right now
set up package.xml
*/




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
    octree->getMetricSize(x, y, z);
    octree->getMetricMin(xmin, ymin, zmin);
    x = x/res;
    y = y/res;
    z = z/res;

    // Allocate memory for a 3 dimension char array with dimensions based on octomap sizes. Orientation is x to right, y down, z into page.
        // size of array dimension is maximum index + 1 || MetricSize/res
    char arr[(int)x][(int)y][(int)z];

    // Loop through array and set all values to 0. Both unknown and empty space will be 0 this way.
    memset(arr, '\0', (int)x*(int)y*(int)z);

    // Create index values
    int xind=0, yind=0, zind=0;

    // for each leaf of octree (based on leaf iterators)
    for(octomap::OcTree::leaf_iterator it = octree->begin_leafs(),
        end = octree->end_leafs();
        it != end;
        ++it)
    {
        // convert the node center coords to integer indexes for the array.
            // index = (value-MetricMin)/res-0.5
        xind = (it.getX()-xmin)/res - 0.5;
        yind = (it.getY()-ymin)/res - 0.5;
        zind = (it.getZ()-zmin)/res - 0.5;

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
        }
    }

    // Output the array in some fashion
    // Print array
    for(int dep=0; dep<(int)z; dep++) //Cycle through depths
    {
        for(int row=0; row<(int)y; row++) //Cycle through rows
        {
            for(int col=0; col<int(x); col++) //Cycle through columns
            {
                printf("%d ",arr[col][row][dep]);
            }
            printf("\n");
        }
        printf("\n");
    }

    // Finish callback



    // Old outline
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
    ros::Subscriber sub = nh.subscribe("octomap_full", 1, callback);

    // Announce publisher
    ros::Publisher pub = nh.advertise<octomap_msgs::Octomap>("output", 1);

    // Spin until shut down
    ros::spin();
}





