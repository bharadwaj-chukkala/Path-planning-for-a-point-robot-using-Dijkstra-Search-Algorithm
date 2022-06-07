# Path-Planning-for-a-point-robot-using-Dijkstra-Search-Algorithm
ENPM661 Project 2

A point robot traverses in an obstacle map to find the goal from a starting point without colliding with the obstacles



### User Dependencies:
 - Python 3.9
 - IDE to run the program (I used VSCode)
 - Libraries : numpy, argparse, matplotlib.pyplot, time

### Instructions to run:
 - Open an IDE
 - Navigate to the folder where the .py file exists
 - Upon running the code, the user will be prompted 
    - To enter the start coordinates
    - To enter the goal coordinates
    - Note: please have one space between the start and goal
 - To visualize the exploration and path, please uncomment the plt.pause() on line 300.
 - If you want the path to be displayed without live visualization keep the line commented.
 - The visuzaliztion using matplotlib is a bit slower so, to view the result, just run it.
 - It will take some time to generate the plot. Thank you for being Patient.

### Code:
 - The code describes how to implement Dijkstra algorithm on a point robot
 - Some notable functions are: 
     |
     |--
     | C_obs_space() |Constructs obstacle space
     | Djikstra() | Implements Djikstra Algorithm
     | Action_set() | Implements motion of point robot to traverse

### Special Instructions on giving Arbitrary start and goal nodes:
 - (0,0),(1,1),(400,250),(399,249),(399,250),(400,0),(400,249),(0,250),(0,400),(250,0) are out of bounds
 - I have defined an extra layer of bound to show the edges of the map.

### Output:
- A plot of all explored nodes and a path depicting the shortest distance between start and goal node.

### Contact Author

Name : __Bharadwaj Chukkala__ <br>
Email : bchukkal@terpmail.umd.edu <br>

[![forthebadge](https://forthebadge.com/images/badges/made-with-python.svg)](https://forthebadge.com)


