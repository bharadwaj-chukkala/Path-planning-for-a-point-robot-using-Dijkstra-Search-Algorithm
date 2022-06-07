#IMPORTING NECESSARY LIBRARIES

import numpy as np
import matplotlib.pyplot as plt
import time
import heapq

########## DEFINING A NODE CLASS TO STORE NODES AS OBJECTS ###############

class Node:

	def __init__(self, x, y, cost, parent_id):

		self.x = x
		self.y = y
		self.cost = cost
		self.parent_id = parent_id
	
	def __lt__(self,other):
		return self.cost < other.cost

########### DEFINING ACTIONS TO BE PERFORMED ##############
########### CALCULATING COST TO COME FOR ALL ACTIONS ########

def move_East(x,y,cost):
	x = x + 1
	cost = 1 + cost
	return x,y,cost

def move_West(x,y,cost):
	x = x - 1
	cost = 1 + cost
	return x,y,cost

def move_North(x,y,cost):
	y = y + 1
	cost = 1 + cost
	return x,y,cost

def move_South(x,y,cost):
	y = y - 1
	cost = 1 + cost
	return x,y,cost

def move_NorthEast(x,y,cost):
	x = x + 1
	y = y + 1
	cost = 1 + cost
	return x,y,cost

def move_NorthWest(x,y,cost):  #Traversing Diagonally will cost more because of the distance covered
	x = x - 1
	y = y + 1
	cost = np.sqrt(2) + cost
	return x,y,cost

def move_SouthEast(x,y,cost):
	x = x + 1
	y = y - 1
	cost = np.sqrt(2) + cost
	return x,y,cost

def move_SouthWest(x,y,cost):
	x = x -1
	y = y - 1
	cost = np.sqrt(2) + cost
	return x,y,cost

############ DEFINING A FUNCTION TO PERFORM ACTIONS THAT ARE DEFINED #########

def Action_set(move,x,y,cost):

	if move == 'West':
		return move_West(x,y,cost)
	elif move == 'East':
		return move_East(x,y,cost)
	elif move == 'North':
		return move_North(x,y,cost)
	elif move == 'South':
		return move_South(x,y,cost)
	elif move == 'NorthEast':
		return move_NorthEast(x,y,cost)
	elif move == 'NorthWest':
		return move_NorthWest(x,y,cost)
	elif move == 'SouthEast':
		return move_SouthEast(x,y,cost)
	elif move == 'SouthWest':
		return move_SouthWest(x,y,cost)
	else:
		return None

############ CONFIGURATION SPACE CONSTRUCTION WITH OBSTACLES ############

def C_obs_space(width,height):

    obs_space = np.full((height,width),0)
    
    for y in range(height) :
        for x in range(width):
            
        ####### CLEARANCE FOR THE OBSTACLES #######
            
            #Polygon Obstacle (Clearance)
            t1 = (y-5) - ((0.316) *(x+5)) - 173.608
            t2 = (y+5) + (1.23 * (x+5)) - 229.34 
            t3 = (y-5) + (3.2 * (x-5)) - 436 
            t4 = (y+5) - 0.857*(x-5) - 111.42 
            t5 = y + (0.1136*x) - 189.09
            
            #Circle Obstacle (Clearance)
            C = ((y -185)**2) + ((x-300)**2) - (45)**2  
            
            #Hexagon Obstacle (Clearance)
            h1 = (y-5) - 0.577*(x+5) - 24.97
            h2 = (y-5) + 0.577*(x-5) - 255.82
            h3 = (x-6.5) - 235 
            h6 = (x+6.5) - 165 
            h5 = (y+5) + 0.577*(x+5) - 175 
            h4 = (y+5) - 0.577*(x-5) + 55.82 
            
            #Conditions defining all points bounded by these lines are in the obstacle clearance area
            if(h1<0 and h2<0 and h3<0 and h4>0 and h5>0 and h6>0) or C<=0  or (t1<0 and t5>0 and t4>0)or (t2>0 and t5<0 and t3<0):
                obs_space[y,x] = 1
               
             
        ########  OBSTACLES   ########
            
            #Hexagon Obstacle    
            a1 = y - 0.577*x - 24.97 
            a2 = y + 0.577*x - 255.82
            a3 = x - 235 
            a6 = x - 165 
            a5 = y + 0.577*x - 175 
            a4 = y - 0.577*x + 55.82 
            
            #Circle Obstacle
            D = ((y -185)**2) + ((x-300)**2) - (40)**2 
            
            #Polygon Obstacle
            l1 = y - ((0.316) *x) - 173.608  
            l2 = y + (1.23 * x) - 229.34 
            l3 = y + (3.2 * x) - 436 
            l4 = y - 0.857*x - 111.42 
            l5 = y + (0.1136*x) - 189.09
            
            #Conditions defining all points bounded by these lines are in the obstacle area
            if(a1<0 and a2<0 and a3<0 and a4>0 and a5>0 and a6>0) or D<0 or (l1<0 and l5>0 and l4>0)or (l2>0 and l5<0 and l3<0):
                obs_space[y,x] = 2
                
                
####### DEFINING THE BOUNDARIES FOR CONFIGURATION SPACE ########

    for i in range(400):
        obs_space[0][i] = 1
        
    for i in range(400):
        obs_space[249][i] = 1
        
    for i in range(250):
        obs_space[i][1] = 1
        
    for i in range(250):
        obs_space[i][399] = 1
       
    return obs_space

########## TO SEE IF THE MOVE IS VALID OR NOT #########

def ValidMove(x, y, obs_space):

	e = obs_space.shape

	if( x > e[1] or x < 0 or y > e[0] or y < 0 ):
		return False
	
	else:
		try:
			if(obs_space[y][x] == 1  or obs_space[y][x]==2):
				return False
		except:
			pass
	return True

########## DEFINING A FUNCTION TO CHECK IF THE PRESENT NODE IS GOAL NODE ##########

def Check_goal(present, goal):

	if (present.x == goal.x) and (present.y == goal.y):
		return True
	else:
		return False

######### GENERATE UNIQUE KEY ##########

def key(node):
    key = 1022*node.x + 111*node.y 
    return key


########## DIJKSTRA ALGORITHM ###########

def dijkstra(start, goal,obs_space):

    if Check_goal(start, goal):
        return None,1
    goal_node = goal
    start_node = start
    
    moves = ['West','East','North','South','NorthEast','NorthWest','SouthEast','SouthWest']
    unexplored_nodes = {}  #List of all open nodes
    
    start_key = key(start_node) #Generating a unique key for identifying the node
    unexplored_nodes[(start_key)] = start_node
    
    explored_nodes = {} #List of all closed nodes
    priority_list = []  #List to store all dictionary entries with cost as the sorting variable
    heapq.heappush(priority_list, [start_node.cost, start_node]) #This Data structure will prioritize the node to be explored which has less cost.
    
    all_nodes = [] #stores all nodes that have been traversed, for visualization purposes.
    

    while (len(priority_list) != 0):
        
        #popping the first element in the priority list to create child nodes for exploration
        present_node = (heapq.heappop(priority_list))[1]
        #appending all child nodes so that the explored region of the map can be plotted.
        all_nodes.append([present_node.x, present_node.y])
        #creating a dict key for identfication of node individually
        present_id = key(present_node)
        #The program will exist if the present node is the goal node
        if Check_goal(present_node, goal_node):
            goal_node.parent_id = present_node.parent_id
            goal_node.cost = present_node.cost
            print("Goal Node found")
            return all_nodes,1

        if present_id in explored_nodes:  
            continue
        else:
            explored_nodes[present_id] = present_node
	#deleting the node from the open nodes list because it has been explored and further its child nodes will be generated	
        del unexplored_nodes[present_id]
    #For all actions in action set, a new child node has to be formed if it is not already explored
        for move in moves:
            x,y,cost = Action_set(move,present_node.x,present_node.y,present_node.cost)
   #Creating a node class object for all coordinates being explored
            new_node = Node(x,y,cost,present_node)  
   
            new_node_id = key(new_node) 
   
            if not ValidMove(new_node.x, new_node.y, obs_space):
                continue
            elif new_node_id in explored_nodes:
                continue
   
            if new_node_id in unexplored_nodes:
                if new_node.cost < unexplored_nodes[new_node_id].cost: 
                    unexplored_nodes[new_node_id].cost = new_node.cost
                    unexplored_nodes[new_node_id].parent_id = new_node.parent_id
            else:
                unexplored_nodes[new_node_id] = new_node
   			
            heapq.heappush(priority_list, [ new_node.cost, new_node])
   
    return  all_nodes,0

########### BACKTRACK AND GENERATE SHORTEST PATH ############

def Backtrack(goal_node):  
    x_path = []
    y_path = []
    x_path.append(goal_node.x)
    y_path.append(goal_node.y)

    parent_node = goal_node.parent_id
    while parent_node != -1:
        x_path.append(parent_node.x)
        y_path.append(parent_node.y)
        parent_node = parent_node.parent_id
        
    x_path.reverse()
    y_path.reverse()
    
    return x_path,y_path

#########  PLOT OBSTACLES SPACE, EXPLORED NODES, SHORTEST PATH  #######

def plot(start_node,goal_node,x_path,y_path,all_nodes,obs_space):

    ### Start node and Goal node ###
    plt.plot(start_node.x, start_node.y, "Dw")
    plt.plot(goal_node.x, goal_node.y, "Dg")

    ### Configuration Space for Obstacles ####
    plt.imshow(obs_space, "GnBu")
    ax = plt.gca()
    ax.invert_yaxis() #y-axis inversion
    
    ### All visited nodes ###
    for i in range(len(all_nodes)):
        plt.plot(all_nodes[i][0], all_nodes[i][1], "2g")
        #plt.pause(0.000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000001)
    
    ### Shortest path found ###
    plt.plot(x_path,y_path, ":r")
    plt.show()
    plt.pause(3)
    plt.close('all')

######### CALLING ALL MY FUNCTIONS TO IMPLEMENT dijkstra ALGORITHM ON A POINT ROBOT ###########

if __name__ == '__main__':

    width = 400
    height = 250
    obs_space = C_obs_space(width, height)
    
    #### Taking start node coordinates as input from user #####
    start_coordinates = input("Enter coordinates for Start Node: ")
    s_x, s_y = start_coordinates.split()
    s_x = int(s_x)
    s_y = int(s_y)
    
    ### Checking if the user input is valid #####
    if not ValidMove(s_x, s_y, obs_space):
        print("Start node is out of bounds")
        exit(-1)
		    
	##### Taking Goal node coordinates as input from user ##### 
    goal_coordinates = input("Enter coordinates for Goal Node: ")
    g_x, g_y = goal_coordinates.split()
    g_x = int(g_x)
    g_y = int(g_y)
    
    ### Checking if the user input is valid #####
    if not ValidMove(g_x, g_y, obs_space):
        print("Goal node is out of bounds")
        exit(-1)
    
    ### Timer to calculate computational  time ###
    timer_start = time.time()
    
	##### Creating start_node and goal_node objects 
    start_node = Node(s_x, s_y, 0.0, -1)
    goal_node = Node(g_x, g_y, 0.0, -1)
    all_nodes,flag = dijkstra(start_node, goal_node,obs_space)
    
    ##### Plot shortest path only when goal node is reached #####
    if (flag)==1:
        x_path,y_path = Backtrack(goal_node)
    else:
        print("No path was found")
		
    plot(start_node,goal_node,x_path,y_path,all_nodes,obs_space)
    timer_stop = time.time()
    
    C_time = timer_stop - timer_start
    print("The Total Runtime is:  ", C_time) 
	



	









