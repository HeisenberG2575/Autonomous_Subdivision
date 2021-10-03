from utils import is_valid, is_unblocked, is_target, Cell, COL, ROW, store_path, heuristic_func
import sys
import numpy as np

FLOAT_MAX = sys.float_info.max
closed_list = []
cell_details = []



"""
    Generating all the 8 successor of this cell 
  
            N.W   N   N.E 
              \   |   / 
               \  |  / 
            W----Cell----E 
                 / | \ 
               /   |  \ 
            S.W    S   S.E         
  
    Cell-->Popped Cell (i, j) 
    N -->  North       (i-1, j) 
    S -->  South       (i+1, j) 
    E -->  East        (i, j+1) 
    W -->  West           (i, j-1) 
    N.E--> North-East  (i-1, j+1) 
    N.W--> North-West  (i-1, j-1) 
    S.E--> South-East  (i+1, j+1) 
    S.W--> South-West  (i+1, j-1)
"""

found_dest = False

def generate_successor(grid, dest, i_succ, j_succ, i, j, constant, open_list):
    # process only if it is a valid cell
    # print("what is i,j?", i, j)
    R, C = grid.shape#this assumes numpy array
    global found_dest, closed_list, cell_details
    if is_valid(i_succ, j_succ, R, C):
        # if we've reached the destination
        if is_target(i_succ, j_succ, dest):
            cell_details[i_succ][j_succ].parent_i = i
            cell_details[i_succ][j_succ].parent_j = j
            print("Destination found.")
            store_path(cell_details, dest)
            found_dest = True
            # print("did we find?", found_dest)
            return found_dest

        # if the successor is on the closed_list or is blocked
        # then ignore else do the following
        elif (closed_list[i_succ][j_succ] == False and is_unblocked(grid, i_succ, j_succ)):
            g_new = cell_details[i][j].g + constant
            h_new = heuristic_func(i_succ, j_succ, dest, 0)
            f_new = g_new + h_new

            # if it isn't on the open_list, add it
            # make current square the parent of this square
            # record f, g, h costs of this cell
            # ------OR---------
            # if it is on the open_list then check if this
            # path to the square is better using 'f' as cost measure
            if (cell_details[i_succ][j_succ].f == FLOAT_MAX or cell_details[i_succ][j_succ].f > f_new):
                open_list.append((f_new, (i_succ, j_succ)))
                
                cell_details[i_succ][j_succ].f = f_new
                cell_details[i_succ][j_succ].g = g_new
                cell_details[i_succ][j_succ].h = h_new
                cell_details[i_succ][j_succ].parent_i = i
                cell_details[i_succ][j_succ].parent_j = j


# function to calculate the minimum distance between source and destination
# Parameters:
# grid: 2D matrix with entries as 0 and 1 where 0 is a blocked path and 1 is an open path
# src: tuple representing the coordinates of the source
# dest: tuple representing the coordinates of the destination


def a_star_algorithm(grid, src, dest):
	global closed_list, cell_details
	R, C = grid.shape#this assumes numpy array
	# closed_list: 2D matrix that stores initialize it to
	# false which means no cell has been included yet
	closed_list = [[False for i in range(C)] for j in range(R)]

	# cell_details: 2D matrix that contains the details of cell
	cell_details = [[Cell(FLOAT_MAX, FLOAT_MAX, FLOAT_MAX, -1, -1)
			         for i in range(C)] for j in range(R)]

	# open_list: it has information as- <f, <i, j>>
	# where f = g + h, and i, j are the row and column
	# open_list = []
	if not is_valid(src[0], src[1], R, C):
		print("Source is invalid.")
		return

	if not is_valid(dest[0], dest[1], R, C):
		print("Destination is invalid.")
		return

	if not (is_unblocked(grid, src[0], src[1]) and is_unblocked(grid, dest[0], dest[1])):
		print("Source or destination is blocked.")
		return

	if is_target(src[0], src[1], dest):
		print("Already at destination.")
		return

	# initializing the starting node
	i = src[0]
	j = src[1]

	cell_details[i][j].f = 0.0
	cell_details[i][j].g = 0.0
	cell_details[i][j].h = 0.0
	cell_details[i][j].parent_i = i
	cell_details[i][j].parent_j = j

	# open_list: it has information as- <f, <i, j>>
	# where f = g + h, and i, j are the row and column
	open_list = []
	# add source to open_list
	open_list.append((0.0, (i, j)))

	# found_destination: initially set to false
	global found_dest
	found_dest = False

	iterator = 0

	while len(open_list):
		# print("iter: ", iterator)
		# print("open_list before while", open_list)
		# remove element from the open list
		top = open_list[0]
		open_list.pop(0)

		# add this to the closed list
		i = top[1][0]
		j = top[1][1]
		closed_list[i][j] = True

		# ------1st successor------North-------
		generate_successor(grid, dest, i-1, j, i, j, 1.0, open_list)
		if found_dest == True:
		    return

		# ------2nd successor-----South-------
		generate_successor(grid, dest, i+1, j, i, j, 1.0, open_list)
		# print(found_dest)
		if found_dest == True:
		    return

		# ------3rd successor-----East-------
		generate_successor(grid, dest, i, j+1, i, j, 1.0, open_list)
		# print(found_dest)
		if found_dest == True:
		    return

		# ------4th successor-----West-------
		generate_successor(grid, dest, i, j-1, i, j, 1.0, open_list)
		# print(found_dest)
		if found_dest == True:
		    return

		# ------5th successor-----North-East-------
		generate_successor(grid, dest, i-1, j+1, i, j, 1.414, open_list)
		# print(found_dest)
		if found_dest == True:
		    return

		# ------6th successor-----North-West-------
		generate_successor(grid, dest, i-1, j-1, i, j, 1.414, open_list)
		# print(found_dest)
		if found_dest == True:
		    return

		# ------7th successor-----South-East-------
		generate_successor(grid, dest, i+1, j+1, i, j, 1.414, open_list)
		# print(found_dest)
		if found_dest == True:
		    return

		# ------8th successor-----South-West-------
		generate_successor(grid, dest, i+1, j-1, i, j, 1.414, open_list)
		# print(found_dest)
		if found_dest == True:
		    return

		iterator += 1
		# print("open_list before while", open_list)


	if found_dest == False:
		print("Destination not found.")



# here a 0 means a blocked cell and a 1 means an unblocked cell

# grid = [[1, 1, 0, 1],
#         [1, 0, 1, 1],
#         [1, 1, 0, 0],
#         [1, 0, 0, 1]]

# src = [0, 0]
# dest = [2, 1]
# path = (0, 0) -> (1,0) -> (2, 0) -> (2,1)

# a_star_algorithm(grid, src, dest)
