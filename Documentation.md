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

### Preparing a trajectory using .csv input.
Assumptions in instructions:
    1. Your catkin workspace is in the user's home folder and is named "catkin_ws"
While there are a number of possible input methods for the control program, the most straightforward and flexible one is to use .csv files to store your trajectory input. 
You will need to store csv files in the trajectories/ directory of the installed mill_controller package in your catkin workspace's src/ directory. To navigate a terminal here, for example, you might enter `cd ~/catkin_ws/src/mill_controller/trajectories/`.
These instructions will walk you through examining an example csv file, creating a simple csv file of your own, and having it read as trajectory input.

1. Navigate to the directory containing your package, inside of your catkin workspace. From there, open the "trajectories" directory. In addition to the commands above, you could also bring a terminal to this directory by entering `roscd mill_controller/trajectories/`.

2. In this directory there should already be a pair of csv files, "example.csv" and "fake.csv". To find out about fake.csv, refer to the notes on the black_box node in the "Nodes" section. Open example.csv in your preferred text editor.

3. The file should contain some simple formatted text.

   `1,2,3,4`  
   `5,6,7,8`

   This is the correct format for trajectory input. Each line represents a distinct x,y,z point along the trajectory at a given time t.  
   The value of each piece of information can be written as integers or decimal numbers representing millimeters and seconds, arranged in the format x,y,z,t.  
   This example file contains a 2 point trajectory, from (x=1.0, y=2.0, z=3.0) at t=4.0 seconds to (x=5.0, y=6.0, z=7.0) at t=8.0 seconds.  
   No spaces should be included in a line. Trajectories do not need to start at 0 seconds, although the program will generally treat them as though they had.

4. To create your own csv file, open an empty file and add lines to it following the format demonstrated in the last step. Once you are finished adding lines, save your file in the "trajectories/" directory as "your_file.csv"

5. Refer to the section below to have the control program run your trajectory on the X-Carve.



    Command trajectory run with arguments
        Explanation of passing csv names to csv_parser.
        Explain main launch file and available arguments for changing.
        Explain what to expect from input and output.

    Using the black_box node? Maybe just add to node explanations

    Decoding output array
        Explain the nature of the output array
        Explain the helper class if I've actually finished it.

    Example Run through, including input and output.




Using the pen holder
Function and Node Notes
Launch files and arguments
