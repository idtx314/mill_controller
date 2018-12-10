



read trajectory from csv


Send the whole trajectory
or
Send the part of the trajectory from after the timestamp

The pipe can take trajectories from multiple sources. How am I going to implement receding horizon control?
    I want to bake receding horizon control into the pipe itself. It will accept a trajectory and then follow that trajectory until it reaches the correct timestamp. That cuttoff will have to exist in trajectory_parser, which will stop writing lines based on t > cutoff. cutoff 0 = open loop
    In normal mode node will receive a flag, read a trajectory from csv, create a Trajectory message, and publish it to trajectory parser. The csv will be updated before the next flag is sent?
    In fake mode, the node will receive a flag, read a trajectory from csv, create a trajectory message with each timestamp reduced by horizon*number of times the pipe has published a feedback message. If the stamp of a point is below zero it is excluded from the message. Each time a feedback message is published, the counter will be incremented and a new trajectory message will be made
        A separate node will subscribe to pipe output and publish on this node's flag channel. When launched it will read a .csv file, modify the timestamps by counter*horizon, and write the default input csv for this node. timestamps less than zero excluded, if no timestamps remain then end.


    If launched in quick launch mode the node will not wait for a flag before starting the first trajectory message. Should take a file name as a parameter. Name will have to be a null signal by default. If the name is null or does not exist, run in normal mode and use default file name






trajectory is sent
trajectory is followed until timestamp
feedback runs
new trajectory is sent based on result. Timestamp starts over?
trajectory is followed until timestamp
feedback runs

Where is Ahalya using this array?
Is there any standard format for an array there?
What about a .csv file instead of a message and helper class?
Visual demonstration, maybe the Rviz output
