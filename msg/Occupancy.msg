# This message is designed to hold the index values of unoccupied space in a 3 dimensional array representing a physical volume. Index origin = 0. A given index i in the column, row, and layer arrays will return an index value for the column, row, and layer of a point in the 3 dimensional array that is not occupied.
# Occupied space is represented implicitly. width, height, and depth are the number of columns, rows, and layers in the 3 dimensional array. Any set of indexes not listed in the column, row, and layer arrays is occupied.


# The dimensions of the 3 dimensional array
uint16 width
uint16 height
uint16 depth


# Lists of index values that are unoccupied in the multidimensional array
uint16[] column
uint16[] row
uint16[] layer
