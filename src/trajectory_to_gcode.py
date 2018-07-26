# This file should contain a script that accepts trajectories in the form [[x,y,z,t],[x,y,z,t],...] and outputs a gcode file that approximates the given trajectory.

import sys





def main(arg):

    # Parse input from a string to a list of lists
    if(type(arg) != list):
        arg = preprocess(arg)


    # Open output file
    f = open('output.gcode', 'w')

    s = ''
    # Write machine warm up segment to output
    # Return to machine zero (home position)
    s = 'G28' + '\n'
    f.write(s)
    # Set Plane (17 for xy, 18 for zx, 19 for yz)
    s = 'G17'
    # Set units (G20 for inches, G21 for millimeters)
    s = s + ' G21'
    # Set absolute position mode (G90 for absolute, G91 for relative)
    s = s + ' G90'
    # Set feed rate per what (G94 for per minute, G95 for per revolution)
    s = s + ' 94'
    # Set work coordinate system, not sure what this actually means. (G54)
    s = s + ' 54' + '\n'
    # Set Feed rate and starting point
    s = 'G0' + ' X' + str(arg[0][0]) + ' Y' + str(arg[0][1]) + ' Z' + str(arg[0][2]) + ' F100'

    f.write(s)

    # Follow trajectory. Operating on the assumption that the trajectory can be approximated as a series of straight line motions from point to point. Improvement of this model will probably need example trajectories to test.
    for point in arg:
        # x, y, z, t = list[0], list[1], list[2], list[3]
        s = 'G1' + ' X' + str(point[0]) + ' Y' + str(point[1]) + ' Z' + str(point[2]) + '\n'
        # Write appropriate command to output
        f.write(s)

    # Write machine cool down segment to output
    # Move tool toward +z
    # Move to corner


    # Close output
    f.close()









def preprocess(msg):
    '''
    Process an input string into a list of lists
    Ex Input:
    '[[0.0,1.0,1.0,1.0],[0.1,1,2,1],[0.2,2,2,1]]'
    '''
    output = msg.replace('\n','')
    output = msg.replace(' ','')
    output = output.strip('[]')
    output = output.split('],[')

    for index in range(len(output)):
        output[index] = output[index].split(',')
        for subdex in range(len(output[index])):
            output[index][subdex] = float(output[index][subdex])

    return output



if __name__ == '__main__':
    if(len(sys.argv) == 2):
        main(sys.argv[1])
    else:
        print "Usage: generator.py <input list>"
