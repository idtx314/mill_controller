# This file should contain a script that accepts trajectories in the form [[x,y,z,t],[x,y,z,t],...] and outputs a gcode file that approximates the given trajectory.

import sys







def main(arg):

    # Parse input to a list of lists
    if(type(arg) != list):
        arg = preprocess(arg)
    input = arg


    # Open output file
    f = open('')

    # Write machine warm up segment to output
    # Home
    # Move to start position

    # For each list
        # x = list[0]
        # y = list[1]
        # z = list[2]
        # t = list[3]
        # Write appropriate command to output

    # Write machine cool down segment to output
    # Move tool toward +z
    # Move to corner


    # Close output









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
