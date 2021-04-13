import math

# define any module required here
# write all the data structures used here on the start of file
# also give the names of variables to be initutive

ROW = 4
COL = 4

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
    return row >= 0 and row < ROW and col >=0 and col < COL

# check if the cell is blocked or not and return true if unblocked
def is_unblocked(grid, row, col):
    return (grid[row][col] == 1)
    

# check if cell is target or not
def is_target(row, col, dest):
    return row == dest[0] and col == dest[1]

# calculates 'h' heuristics 
def heuristic_func(row, col, dest, key):
    if key == 0:
        # Eucildean distance
        return math.sqrt((row-dest[0])**2 + (col-dest[1])**2)     

    else:
        # Manhatton distance
        return abs(row-dest[0]) + abs(col-dest[1])
    

# store path from source to target
# cell_details is an object of Cell

def store_path(cell_details, dest):
    print("\nThe Path is ")
    row = dest[0]
    col = dest[1]
    
    Path = []
    
    while not(cell_details[row][col].parent_i == row and cell_details[row][col].parent_j == col):
        Path.append([row, col])
        temp_row = cell_details[row][col].parent_i
        temp_col = cell_details[row][col].parent_j
        row = temp_row
        col = temp_col
        
    Path.append([row, col])
    
    while not len(Path):
        p = Path.top()
        Path.pop()
        print("->  ", p)
        
    return
