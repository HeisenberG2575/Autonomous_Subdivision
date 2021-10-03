from utils import COL, ROW
from a_star_algorithm import a_star_algorithm
import numpy as np
import sys

for i in range (0,10):
    # random grid
    grid = np.random.randint(0,2,(ROW, COL))
    
    # randomize these as well
    src = [0,0]
    dest = [2,1]
    # print(grid)

    with open("out_"+str(i)+".txt",'w') as f:
        sys.stdout = f # Change the standard output to the file we created.
        print("Grid is \n")
        print(grid)
        print("from: " + str(src) + " to: " + str(dest) + "\n")
        a_star_algorithm(grid, src, dest)