# define any module required here
# write all the data structures used here on the start of file
# also give the names of variables to be initutive

class Cell:
    # parent_i and parent_j are row and column index of its parent
    # f = g + h, where h is heuristic function 
    def __init__(self, parent_i, parent_j, f, g, h):
        self.parent_i = parent_i
        self.parent_j = parent_j
        self.f = f
        self.g = g
        self.h = h

# check if the cell is vaild or not
def is_valid(row, col):
    return True

# check if the cell is blocked or not
def is_blocked(grid, row, col):
    return True

# check if cell is target or not
def is_target(row, col, dest):
    return True

# calculates 'h' heuristics 
def heuristic_func(row, col, dest, key):
    if key == 0:
        # Eucildean distance
        pass
    elif key == 1:
        # Manhatton distance
        pass
    else:
        # remove key and only take Euclidean distance 
        pass

# store path from source to target
# cell_details is an object of Cell
def store_path(cell_details, dest):
    return True