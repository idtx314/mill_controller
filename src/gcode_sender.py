#!/usr/bin/env python

"""
This script contains a ROS node that waits for a bool variable to be published over the topic '/gcode_ready_flag'. When received, it opens a file called 'output.gcode' located in the current directory and sends the contents line by line over the serial port 'ttyUSB0'.
The heart of this script is simple_stream.py, a basic gcode streaming script under the MIT License. The License header is posted below.
"""

"""

---------------------
The MIT License (MIT)

Copyright (c) 2012 Sungeun K. Jeon

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
---------------------
"""

import rospy
import sys
import serial
import time
from std_msgs.msg import Bool


# Use this publisher to send the translated input
pub = rospy.Publisher('gcode_sent_flag',Bool,queue_size=1)


def callback(input):
    # Upon receiving a message, reads a GCode file and sends the commands within to a GRBL controller over a serial connection

    # Open grbl serial port
    s = serial.Serial('/dev/ttyUSB0',115200)

    # Open g-code file
    f = open('output.gcode','r');

    # Wake up grbl
    s.write("\r\n\r\n")
    # Wait for grbl to initialize
    time.sleep(2)
    # Flush startup text in serial input
    s.flushInput()

    # Stream g-code to grbl
    for line in f:
        # Strip all EOL characters for consistency
        l = line.strip()
        # Send g-code block to grbl
        s.write(l + '\n')

        """
        When grbl has finished parsing the g-code block, it will
        return an 'ok' or 'error' response. When the planner buffer is full,
        grbl will not send a response until the planner buffer clears space.

        G02/03 arcs are special exceptions, where they inject short line
        segments directly into the planner. So there may not be a response
        from grbl for the duration of the arc.
        """
        # Wait for grbl response with carriage return
        grbl_out = s.readline()


    # Close file and serial port
    f.close()
    s.close()

    # Publish notification message for next node
    flag = Bool()
    flag.data = True
    pub.publish(flag)





def main():
    # Initialize the ROS node and Subscriber
    rospy.init_node("gcode_sender_node")
    rospy.Subscriber('gcode_ready_flag',Bool,callback)

    # Spin until shut down
    rospy.spin()



if __name__ == '__main__':
    main()
