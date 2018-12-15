# Introduction
What this package is, even.

## Build Instructions
    This package was developed on Linux Ubuntu 16.04 (Xenial Xerus), with ROS Kinetic Kame.

1. Install ROS on your system following the instructions listed here: http://wiki.ros.org/ROS/Installation.

2. Create a catkin workspace using the methods described here: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment. This directory will be referred to as "catkin_ws" during the rest of these instructions.

3. Install dependencies

4. Change directory into catkin_ws/src

5. Clone this package using `git clone` with a link from the "clone or download" button on the top right of this page.

6. `cd ..` to move back to the workspace root.

7. `catkin_make install` to build all the packages in the workspace.

If all dependencies are installed correctly, the package should now be ready to run.



## Run Instructions
    Lots of images and video in this section

### Calibrating The Workspace and Material
Assumptions in instructions:
1. Your catkin workspace is in the user's home folder and is named "catkin_ws"
2. You are using the bash interpreter to run your terminal environment.
3. You are using a USB camera compatible with the package camera driver.

It will be necessary to perform a NORMAL calibration of the camera each time you want to change the size or location of your material.  
If the position of the camera itself is adjusted, or the camera is detached and re-attached, then a FULL calibration will be necessary.  
If images of the material appear to be warped or misaligned, then try recalibrating the camera by following these steps.  
1. Have the camera plugged into the USB port. The camera driver used by the package is compatible with most USB cameras. 

2. Identify the port name assigned to your camera. This will be of the form "/dev/video1" on linux, with the number varying. By default, the package will assume that your camera is on /dev/video1, and we will often use that in examples. To identify the port of your usb camera:
     1. Disconnect the camera.
     2. Open a terminal and enter `ls /dev/video*`.
     3. Reconnect the camera.
     3. In the terminal, enter `ls /dev/video*` again.
     4. Compare the two lists. The name that appeared when you reconnected your camera is the camera's port name. If it isn't video1, then whenever you see the name "video1" in these instructions, you should change it to match your setup.

3. In your terminal, move into your catkin workspace by entering `cd ~/catkin_ws` 

4. Set up your terminal environment by entering `source devel/setup.bash`

5. Enter `roslaunch mill_controller image_calibration.launch`. If your video port is not /dev/video1, add the argument ` video_device:=/dev/video#`, replacing # with the number of your video port name. The launch file will begin a NORMAL calibration by default. If you wish to perform a FULL calibration, add the argument `calibration:=FULL`

6. The image alignment script will guide you through calibrating the location of the X-Carve cutting board and the material you intend to be working with in the image taken from your webcam. This information will be used in image processing so that output information will consist of just the working material rather than the whole image.

### Preparing a Trajectory Using .csv File Input.
Assumptions in instructions:
    1. Your catkin workspace is in the user's home folder and is named "catkin_ws"
    2. You 
While there are a number of possible input methods for the mill controller, the most straightforward and flexible one is to use .csv files to store your trajectory input. 
You will need to store csv files in the trajectories/ directory of the installed mill_controller package in your catkin workspace's src/ directory. To navigate a terminal here, for example, you might enter `cd ~/catkin_ws/src/mill_controller/trajectories/`.
These instructions will walk you through examining an example csv file, creating a simple csv file of your own, and having it read as trajectory input.

1. Navigate to the directory containing your package, inside of your catkin workspace. From there, open the "trajectories" directory. In addition to the commands above, you could also bring a terminal to this directory by entering `roscd mill_controller/trajectories/`.

2. In this directory there should already be a pair of csv files, "example.csv" and "fake.csv". To find out about fake.csv, refer to the notes on the black_box node in the "Nodes" section. Open example.csv in your preferred text editor.

3. The file should contain some simple formatted text.

   `0.1,0.2,0.3,4`  
   `0.5,0.6,0.7,8`

   This is the correct format for trajectory input. Each line represents a distinct x,y,z point along the trajectory at a given time t.  
   The value of each piece of information can be written as integers or decimal numbers representing percentage of material dimension and seconds, arranged in the format x,y,z,t.  
   The material origin is considered to be in bottom left corner of the material, with positive x toward the material's bottom right corner and positive y toward the material's top left corner.  
   The appropriate input ranges of x and y can be set when you start the mill controller. See the Launch Files and Arguments section for a more complete explanation of input range. By default the input ranges are from 0.0 to 1.0, representing percentage of the material's dimension along that axis.  
   This example file contains a 2 point trajectory, from (x=10%, y=20%, z=30%) at t=4.0 seconds to (x=50%, y=60%, z=70%) at t=8.0 seconds.  
   No spaces should be included in a line. Trajectories do not need to start at 0 seconds, although the mill controller will generally treat them as though they had.  

4. To create your own csv file, open an empty file and add lines to it following the format demonstrated in the last step. Once you are finished adding lines, save your file in the "trajectories/" directory as "your_file.csv"

5. Refer to the section below to have the mill controller run your trajectory on the X-Carve.

### Running a Trajectory
Assumptions in instructions:
1. Your catkin workspace is in the user's home folder and is named "catkin_ws"
2. You are using the bash interpreter to run your terminal environment.

1. Open a terminal and move into your catkin workspace, for example by entering `cd ~/catkin_ws/`
2. Source your setup.\*sh file, for example by entering `source devel/setup.bash`.
3. Have a trajectory csv file as described in [Preparing a Trajectory Using csv Input] section. This should be in the mill_controller/trajectories/ directory. We will call this "my_trajectory.csv" in these instructions.
4. Connect your X-Carve and USB camera to your computer. 
4. Calibrate your workspace and the location of the material for imaging by following the [Calibrating The Workspace and Material] instructions above. **You must calibrate the workspace for the position that you intend the working material to occupy during trajectory execution.** If none of your calibration data has changed since the last time you calibrated, then you may skip this step and the most recent data will be used.
5. Launch the mill controller by entering `roslaunch mill_controller mill_controller.launch`.  
By default, the mill controller will expect you to be placing an 11"x8.5" piece of paper in the lower left corner of the workspace, with the long side parallel to the workspace's bottom edge. You can customize the dimensions, location, and rotation of the material by adding input arguments to the command. These arguments are explained in detail in the [Launch Files and Arguments] section. For example: to change the position of the material's origin to 200mm on the x axis and 150mm on the y axis in the machine workspace, we would instead enter the command:  
`roslaunch mill_controller mill_controller.launch x_offset:=200 y_offset:=150`

   You should also change the usb port and video port if necessary using the appropriate arguments. Determining what video port your camera is on is described in the [Calibrating the Workspace and Material] section. You can identify the usb port your X-Carve has been assigned by using the same method with the command `ls /dev/ttyUSB*`.
6. Now that your material is in place, the workspace is calibrated, your trajectory is ready, and you have launched the mill controller with arguments to inform it of the material position and dimensions, it is time to run your trajectory. Since we did not launch the mill controller with a file name to run automatically (see [Launch Files and Arguments] for more on this option) we will need to publish a string message containing the name of our trajectory csv file. Open a new console and enter the following command, replacing the name of the trajectory file if necessary:  
`rostopic pub /csv_name_topic std_msgs/String "data: 'my_trajectory.csv'"`
7. The mill controller will now read my_trajectory.csv from the trajectories/ directory, translate it into a gcode file named "output.gcode", and stream those gcode commands to the X-Carve. Since we did not place the X-Carve in closed loop mode, it will run the entire trajectory before taking an image of the completed drawing and publishing its output.
8. The X-Carve should provide several visual representations of output along with an Occupancy message that encodes an array representing locations on the material that have and have not been worked. Understanding this output is covered in the [Interpreting Mill Controller Output] section.


### Interpreting Mill Controller Output
Using mill_controller.launch provides a set of 

/octomap_to_cloud is a marker message based on pointcloud centers. This corresponds to /octomap_point_cloud_centers published by the /octomap_server node.
/camera_depth_points is a pointcloud2 message representing every white pixel in the processed image. This represents the full data before it is passed to octomap to be, essentially, decimated.

/occupied_cells_vis_array is a MarkerArray topic that represents regions of the picture that contain white pixels with a green cube. 
/free_cells_vis_array is a MarkerArray topic that cells known to be unoccupied with a green cube. Because of how the octomap is generated, not all cells in the array are known. This greatly reduces the usefulness of the topic.

/octomap_to_occupancy is an Occupancy message as defined in the mill_controller package. This contains the dimensions of the full array

Interpreting Occupancy messages
        Explain the helper class if I've actually finished it.


### An Example in Video
    Example Run through, including input and output.




During calibration, suggest moving mill carriage to x250 y500 to show corners
Separate section for identifying the USB and video port?
Input option notes
Using the pen holder
Function and Node Notes
    Using the black_box node? Maybe just add to node explanations
Launch files and arguments
    The limits of the input ranges will correspond to the working material's dimensions on the x and y axes, with those dimensions also being possible launch arguments. 
    Explain main launch file and available arguments for changing.
Resetting the workspace location.


Known bugs
    Occupancy messages don't actually have the right dimensions.
    Occupancy messages don't actually have all the unoccupied points. Related!
    Camera driver is unreliable. Have to disconnect between calibration and running.
