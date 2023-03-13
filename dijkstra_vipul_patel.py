#!/usr/bin/env python
# coding: utf-8

# In[5]:


# Implementation of the Dijkstra Algorithm for a Point Robot

# Importing the necessary libraries
import numpy as np
import sys
import cv2
import math
from ctypes import c_int64
import time
import heapq as hq

# Defining the clearance (clearance = 5)
clr = 5

# Defining the size of canvas
w = 600                          # width of canvas
h = 250                          # height of canvas

# Defining diffect colors to illustrate obstacles
red = [0 , 0 , 255]
green = [0 , 255 , 0]
blue = [255 , 0 , 0]
white = [255 , 255 , 255]

# Writing function to display the obstacles and clearance on the canvas
def Obstacles(obs):
    for x in range(w):
        for y in range(h):
            
            # Defining clearance on the borders
            if x<=clr or x>=600-clr or y>=250-clr or y<=clr:
                obs[249-y][x] = blue
            
            # Triangle
            # Defining the boundary clearance 
            if x>=460-clr and (y+2*x-1156)<=0 and (y-2*x+906.18)>=0 and y>=20 and y<=230:
                obs[249-y][x] = blue
            # Defining the Triangle
            if x>= 460 and (y-2*x+895)>=0 and (y+2*x-1145)<=0:
                obs[249-y][x] = red
            
            # Hexagon
            if x>=(300-75*math.cos(np.deg2rad(30))-clr) and x<=(300+75*math.cos(np.deg2rad(30))+ clr) and (y-0.58*x-32.568)<=0 and (y+0.58*x-378.978)<=0 and (y-0.58*x+128.98)>=0 and (y+0.58*x-217.43)>=0:
                obs[249-y][x] = blue
            if x>=(300-75*math.cos(np.deg2rad(30))) and x<=(300+75*math.cos(np.deg2rad(30))) and (y-0.58*x-26.79)<=0 and (y+0.58*x-373.21)<=0 and (y-0.58*x+123.21)>=0 and (y+0.58*x-223.21)>=0:
                obs[249-y][x] = red
                
            # Top Rectangle
            if (100-clr)<=x and x<=(150+clr) and (150-clr)<=y and y<=h:
                obs[249-y][x] = blue   
            if x>=100 and x<=150 and y>=150 and y<=h: 
                obs[249-y][x] = red
                
            # Bottom Rectangle
            if (100-clr)<=x and x<=(150+clr) and y>=0 and y<=(100+clr):
                obs[249-y][x] = blue
            if (100)<=x and x<=(150) and 0<=y and y<=(100): 
                obs[249-y][x] = red
    return obs


# Defing the movement of the robot
rev = 249

# Down Movement
def down(current_node, canvas):
    # Creating a new list to store current nodes
    new_node = list(current_node)
    # substracting 1 from y co-ordinate
    new_node[1] -= 1 
    if new_node[1]>=0 and (canvas[rev-new_node[1]][new_node[0]][0])==1:
        return new_node
    else: 
        return None

# Up Movement
def up(current_node, canvas):
    new_node = list(current_node)
    new_node[1] += 1 
    if new_node[1]<=h and (canvas[rev-new_node[1]][new_node[0]][0])==1:
        return new_node
    else: 
        return None
    
# Left Movement
def left(current_node, canvas):
    new_node = list(current_node)
    new_node[0] -= 1 
    if new_node[0] >= 0 and (canvas[rev-new_node[1]][new_node[0]][0])==1:
        return new_node
    else: 
        return None

# Right Movement
def right(current_node, canvas):
    new_node= list(current_node)
    new_node[0] += 1  
    if new_node[0] < w and (canvas[rev-new_node[1]][new_node[0]][0])==1:
        return new_node
    else: 
        return None

# Right-down Movement
def rt_down(current_node, canvas):
    new_node = list(current_node)
    new_node[1] -= 1 
    new_node[0] += 1
    if new_node[1] >= 0 and new_node[0] < w and (canvas[rev-new_node[1]][new_node[0]][0])==1:
        return new_node
    else: 
        return None
    
# Left-down Movement
def lf_down(current_node, canvas):
    new_node = list(current_node)
    new_node[1] -= 1 
    new_node[0] -= 1
    if new_node[1]>=0 and new_node[0]>=0 and (canvas[(rev-new_node[1]), new_node[0]][0])==1:
        return new_node
    else: 
        return None

# Right-up Movement
def rt_up(current_node, canvas):
    new_node = list(current_node)
    new_node[1] += 1 
    new_node[0] += 1
    if new_node[1]<h and new_node[0]<w and (canvas[rev-new_node[1]][new_node[0]][0])==1:
        return new_node
    else: return None

# Left-up Movement
def lf_up(current_node, canvas):
    new_node = list(current_node)
    new_node[1] += 1 
    new_node[0] -= 1
    if new_node[1] < h and new_node[0]>=0 and (canvas[rev-new_node[1]][new_node[0]][0])==1:
        return new_node
    else: 
        return None

# Defining function to explore the nodes 
def explore(new_node, current_node, temp, c_idx, explored_list, cost):
    chk = False
    new_node = tuple(new_node)
    # Check if the node is already explored
    if new_node not in explored_list:
        new_node = [current_node[0], current_node[1], new_node, current_node[2]]
        for i in temp:   
            if i[2] == new_node[2]:
                chk = True
                new_node[0] = new_node[0] + cost
                if i[0] > new_node[0]:
                    i[0] = new_node[0]
                    i[3] = new_node[3]
        if chk == False:
            new_node[0] = new_node[0] + cost 
            c_idx.value += 1
            new_node[1] = c_idx.value
            hq.heappush(temp, new_node)

# Dijkstra Algorithm
def algo(current_node, temp, c_idx, explored_list, canvas):
    new_node = up(current_node[2], canvas)
    if new_node != None:
        # Cost of motion
        cost = 1
        explore(new_node, current_node, temp, c_idx, explored_list, cost)
    
    new_node2 = down(current_node[2], canvas)
    if new_node2 != None:
        cost = 1
        explore(new_node2, current_node, temp, c_idx, explored_list, cost)
    
    new_node3 = left(current_node[2], canvas)
    if new_node3 != None:
        cost = 1
        explore(new_node3, current_node, temp, c_idx, explored_list, cost)

    new_node4 = right(current_node[2], canvas)
    if new_node4!= None:
        cost = 1
        explore(new_node4, current_node, temp, c_idx, explored_list, cost)

    new_node5 = rt_up(current_node[2], canvas)
    if new_node5 != None:
        cost = 1.4
        explore(new_node5, current_node, temp, c_idx, explored_list, cost)

    new_node6 = lf_up(current_node[2], canvas)
    if new_node6 != None:
        cost = 1.4
        explore(new_node6, current_node, temp, c_idx, explored_list, cost)
    
    new_node7 = rt_down(current_node[2], canvas)
    if new_node7 != None:
        cost = 1.4
        explore(new_node7, current_node, temp, c_idx, explored_list, cost)
    
    new_node8 = lf_down(current_node[2], canvas)
    if new_node8 != None:
        cost = 1.4
        explore(new_node8, current_node, temp, c_idx, explored_list, cost)

def backtrack(explored_list, start, goal, canvas):
    # Save the results as a video file
    result = cv2.VideoWriter('Dijkstra_Animation Video.mp4', cv2.VideoWriter_fourcc(*'MJPG'),1000,(600,250))
    # Create a list to store the back track path
    track = []
    # Add goal node to the list
    track.append(goal)
    # Display the exploration of nodes on screen
    for current_node in explored_list:
        canvas[249 - current_node[1], current_node[0]] = green
        cv2.waitKey(1)
        cv2.imshow("Output", canvas)
        # Write the changes in the video
        result.write(canvas)
    # Backtracking from goal node to start node
    current_node = tuple(goal)
    while(current_node != start):
        current_node = explored_list[current_node]
        track.append(current_node)
    track.reverse()
    for p in range (len(track)):
        canvas[249 - track[p][1], track[p][0]] = red
        # Writing the back tracked path in the video
        result.write(canvas)
    #cv2.imshow("canvas",(start_node , goal_node))
    # Display the back tracked path on screen
    cv2.imshow("Output", canvas)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def main():
    check = False
    canvas_e = np.ones((h, w, 3), dtype=np.uint8)
    
    # Create a canvas to display output on screen with obstacles
    canvas = Obstacles(canvas_e)
    
    # Taking input for the Start Node
    x1 = int(input('Enter x co-ordinate of start Node(0-600)\n'))
    y1 = int(input('Enter y co-ordinate of start Node(0-250)\n'))
    if (x1<0) or x1>600 or y1<0 or y1>250:
        print("Enter Valid input!!!!")
    
    # Taking input for Goal Node
    x2 = int(input('Enter x co-ordinate of Goal Node(0-600)\n'))
    y2 = int(input('Enter y co-ordinate of Goal Node(0-250)\n'))
    if (x1<0) or x1>600 or y1<0 or y1>250:
        print("Enter Valid input!!!!")
    
    # storing the start and goal nodes
    start_node =(x1, y1)
    goal_node = (x2, y2)
    c_idx = c_int64(0)
    n_s = [0.0, 0, start_node , start_node]
    
    #creating a list to stores the nodes
    temp = []
    
    # Creating a list to store the explored nodes
    explored_list = {}
    hq.heappush(temp, n_s)
    hq.heapify(temp)
    
    # Start timer to record the time taken for the process
    starting_time = time.time()

    while (len((temp)) > 0):
        current_node = hq.heappop(temp)
        
        # Adding to the explored list
        explored_list[current_node[2]] = current_node[3]
        # Checking if the search reached the goal node
        if current_node[2] == tuple(goal_node):
            print("Goal Reached")
            # Print the time taken to compute the path
            print("Time taken to reach the goal is ", round(time.time() - starting_time,2)," seconds.")
            # Performing back tracking to find optimal path
            backtrack(explored_list, start_node, goal_node, canvas)
            check = True
            break
        # Call the function to run the algorithm
        algo(current_node, temp , c_idx, explored_list, canvas)

    if check == False:
        # Display the message if the nodes entered are in the obstacle space
        print("The goal can not be reached as the entered input was in the obstacle space!!!  Please try again!")


if __name__ == '__main__':
    main()


# In[ ]:





# In[ ]:




