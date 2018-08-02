# This file should contain a script that accepts trajectories in the form [[x,y,z,t],[x,y,z,t],...] and outputs a gcode file that approximates the given trajectory.

import sys

# TODO
# Agree on input units
# add conditional
# review coordinates, positive may be outside workspace



def main(arg):

    easel = true

    # Parse input from a string to a list of lists
    if(type(arg) != list):
        arg = preprocess(arg)

    # Open output file
    f = open('output.gcode', 'w')

    # Header
    s = ''

    if(not easel):
        # Home
        s = s + '$22=1' + '\n'                  # Enable homing cycle
        s = s + '$H' + '\n'                     # Begin homing cycle

    # Basic Settings
    s = s + 'G90' + '\n'                    # Set absolute coordinates
    s = s + 'G21' + '\n'                    # Set mm
    s = s + 'G17' + '\n'                    # Set plane to x/y
    s = s + 'G94' + '\n'                    # Set feed rate mode
    s = s + 'G54' + '\n'                    # Use WCS G54

    if(not easel):
        # Establish new origin for WCS
        s = s + 'G10 L20 P1 X0.0Y0.0Z0.0' + '\n'# Reset G54 WCS origin to this position.

        # Set G28 reference
        s = s + 'G0 X1.0 Y1.0' + '\n'           # Move to safe position in XY plane. Leave Z up from homing.
        s = s + 'G28.1' + '\n'                  # Set reference point at this position.

        # # Create Work Coordinate System G54
        # s = s + 'G0 X1.0 Y1.0' + '\n'           # Move to work 0. This should be taken as a user setting.
        # s = s + 'G10 L20 P1 X0 Y0 Z0' + '\n'    # Reset G54 WCS origin to this position.

    f.write(s)                              # Write to file


    # Body
    # Set Feed rate and starting point
    s = 'G0' + ' X' + str(arg[0][0]) + ' Y' + str(arg[0][1]) + ' Z' + str(arg[0][2]) + ' F9.0'

    f.write(s)

    # Follow trajectory. Operating on the assumption that the trajectory can be approximated as a series of straight line motions from point to point. Improvement of this model will probably need example trajectories to test. Starting with fixed feed rate.
    for point in arg:
        # x, y, z, t = list[0], list[1], list[2], list[3]
        s = 'G1' + ' X' + str(point[0]) + ' Y' + str(point[1]) + ' Z' + str(point[2]) + ' F9' + '\n'
        # Write appropriate command to output
        f.write(s)


    # Footer
    s = ''
    s = s + 'G90' + '\n'                    # Set to absolute coordinates
    s = s + 'G20' + '\n'                    # Set to inches
    s = s + 'G17' + '\n'                    # Set plane to x/y
    s = s + 'G94' + '\n'                    # Set feed rate mode
    s = s + 'G54' + '\n'                    # Set WCS to G54
    s = s + 'G1 Z0.15000 F9.0' + '\n'       # Move bit out of harms way
    if(easel):
        s = s + 'G0 X0.0 Y0.0 Z0.0' + '\n'      # Move to work zero
    else:
        s = s + 'G28' + '\n'                    # Return to reference position
    s = s + 'G4 P0.1' + '\n'                # Dwell for a moment
    f.write(s)                              # Write to file


    # Close output
    f.close()









def preprocess(msg):
    '''
    Process an input string into a list of lists
    Ex Input:
    '[[0.0,1.0,1.0,1.0],[0.1,1,2,1],[0.2,2,2,1]]'
    '''

    # Remove extraneous characters
    output = msg.replace('\n','')
    output = msg.replace(' ','')

    # Strip brackets from both ends
    output = output.strip('[]')
    # Split the values into separate strings
    output = output.split('],[')

    for index in range(len(output)):
        output[index] = output[index].split(',')
        for subdex in range(len(output[index])):
            output[index][subdex] = float(output[index][subdex])

    return output




# Boilerplate
if __name__ == '__main__':
    if(len(sys.argv) == 2):
        main(sys.argv[1])
    else:
        print "Usage: generator.py <input list>"
