import numpy as np
import random
import Queue
from a_star import A_Star

class Robot(object):
    def __init__(self, maze_dim):
        '''
        Use the initialization function to set up attributes that your robot
        will use to learn and navigate the maze. Some initial attributes are
        provided based on common information, including the size of the maze
        the robot is placed in.
        '''
        
        self.location = [0, 0]
        self.heading = 'up'
        self.maze_dim = maze_dim

        self.goals = [[self.maze_dim/2, self.maze_dim/2],\
            [self.maze_dim/2, self.maze_dim/2-1],\
            [self.maze_dim/2-1, self.maze_dim/2], \
            [self.maze_dim/2-1, self.maze_dim/2-1]] #dimensions of the goal box
        
        #exploration 
        #info about walls in each loc. (left, up, right, down)
        self.walls = np.full((self.maze_dim, self.maze_dim, 4), -1) 
        for i in range(self.maze_dim): 
            self.walls[0, i, 0] = 0 #all left walls for column 0
            self.walls[i, 0, 3] = 0  #all bottom walls for row 0 
            self.walls[i, self.maze_dim-1, 1] = 0  #all top walls for row maze_dim-1
            self.walls[self.maze_dim-1, i, 2] = 0  #all right walls for col maze_dim-1
        
        self.mapping = True
        self.step = 0
        self.explored = np.zeros((self.maze_dim, self.maze_dim))
        
        self.goal_entrance  = self.goals[0]
        self.goal_found = False
        
        self.a_star = A_Star(self.maze_dim, self.location)
        self.path_to_goal = []
        self.step_count = 0

    def next_move(self, sensors):    
        #sensors: e.g. [0,3,1] open space for left, front, right
        print ('LOC', self.location)
        print ('step: ', self.step)
        rotation = 0
        movement = 0
        
        #useful variables
        action = {'0': 'left', '1': 'up', '2': 'right', '3': 'down'}
        action_index = {'left': 0, 'up': 1 , 'right': 2, 'down': 3}
        loc = self.location
        heading_turn90 = {'left': 'up', 'up': 'right', 'right': 'down', \
                            'down':'left'} 
        self.step +=1
        # useful functions        
        def find_distance(x1, y1, x2, y2):
            #how many steps it would take to go from (x1,y1) to (x2,y2)
            x_dist = np.abs(x2 - x1)
            y_dist = np.abs(y2 - y1)
            return x_dist + y_dist

        def find_indices_for_walls(heading): 
            #normal list has [left 0 , top 1, right 2 , down 3] 
            #e.g. up would return (left,top,right)-->[0,1,2]
            #
            sensors = {'u': [0, 1, 2], 'r': [1, 2, 3],\
                   'd': [2, 3, 0], 'l': [3, 0, 1],\
                   'up': [0, 1, 2], 'right': [1, 2,3],\
                   'down': [2, 3, 0], 'left': [3, 0, 1]}
            return sensors[heading]
        def choose_action(heading, direction, steps):
            #return rotation, number of steps, 
            #and new heading after moving 
            if heading == 'left':
                if direction == 'left':
                    return 0, steps, 'left'
                elif direction == 'right':
                    return 0, -steps, 'left'
                elif direction == 'down':
                    return -90, steps,'down'
                else:
                    return 90, steps, 'up'
            elif heading == 'right': 
                if direction == 'left':
                    return 0, -steps, 'right'
                elif direction == 'right':
                    return 0, steps, 'right'
                elif direction == 'down':
                    return 90, steps,'down'
                else:
                    return -90, steps, 'up'
            elif heading == 'down':
                if direction == 'left':
                    return 90, steps, 'left'
                elif direction == 'right':
                    return -90, steps, 'right'
                elif direction == 'down':
                    return 0, steps,'down'
                else:
                    return 0, -steps, 'down'
            else:
                if direction == 'left':
                    return -90, steps, 'left'
                elif direction == 'right':
                    return 90, steps, 'right'
                elif direction == 'down':
                    return 0, -steps,'up'
                else:
                    return 0, steps, 'up'
        def get_next_location(loc, direction, steps):
            #next location based on location, direction, number of steps taken
            loc_x = loc[0]
            loc_y = loc[1]
            if direction == 'left':
                return loc_x-steps, loc_y
            if direction == 'right':
                return loc_x+steps, loc_y
            if direction == 'up':
                return  loc_x, loc_y+steps
            if direction == 'down':
                return  loc_x, loc_y-steps
        def find_direction(x1,y1,x2, y2):
            #next heading direction from current and next location
            if x2- x1 == 0 and y2-y1 >= 1:
                return 'up'
            elif x2- x1 == 0 and y2-y1 <= -1:
                return 'down'
            elif x2- x1 >= 1 and y2-y1 == 0:
                return 'right'
            elif x2- x1 <= -1 and y2-y1 == 0:
                return 'left'
        def next_loc_from_wall(loc, wall_info):
            #get all next possible locations from the wall information
            next_locations = []
            for i in range(len(wall_info)):

                if wall_info[i] > 0: #only if there is no wall
                    j = 1
                    while j < wall_info[i].astype(np.int32)+1:
                        if j > 3:
                            break
                        if i == 0 and loc[0]-j >= 0 : #left
                            location = [loc[0]-j, loc[1] ]
                            next_locations.append(location)
                        if i == 1 and loc[1] + j < self.maze_dim: #up
                            location = [loc[0],loc[1] + j]
                            next_locations.append(location)
                        if i == 2 and loc[0] + j < self.maze_dim: #right
                            location = [loc[0]+ j, loc[1]]
                            next_locations.append(location)
                        if i ==3 and loc[1] -j >= 0:   #down 
                            location = [loc[0], loc[1] -j]
                            next_locations.append(location)
                        j +=1
            return next_locations
        def update_wall_info(loc, wall_info):
            #updates the neighboring wall information
            for i in range(len(wall_info)):
                #e.g. if there is a wall on the left, 
                #the right neighbor would have a wall on the right
                if wall_info[i] == 0:
                    if i == 0 and loc[0]-1 >= 0 and self.walls[loc[0]-1, loc[1], 2] == -1: 
                        self.walls[loc[0]-1, loc[1], 2] = 0
                        
                    if i == 1 and loc[1] +1 < self.maze_dim and self.walls[loc[0], loc[1]+1, 3] == -1:
                        self.walls[loc[0], loc[1]+1, 3] = 0
                        
                    if i == 2 and loc[0] +1 < self.maze_dim and self.walls[loc[0]+1, loc[1], 0] == -1:
                        self.walls[loc[0]+1, loc[1], 0] = 0
                        
                    if i ==3 and loc[1]-1 >= 0 and self.walls[loc[0], loc[1]-1, 1] == -1:
                        self.walls[loc[0], loc[1]-1, 1] = 0
                    
                #e.g. if there is no wall on the bottom, 
                #the bottom neighbor would have wall on its top
                if wall_info[i] > 0:
                    w_l = wall_info[i].astype(np.int32)
                    j = 1
                    while j <= w_l :
                        
                        if i == 0 and loc[0]-j >= 0 and self.walls[loc[0]-j, loc[1], 0] == -1: 
                            self.walls[loc[0]-j, loc[1], 0] = w_l-j
                            if self.walls[loc[0]-j, loc[1], 2] == -1 and wall_info[2] != -1: 
                                self.walls[loc[0]-j, loc[1], 2] = wall_info[2] + j
                        if i == 1 and loc[1] +j < self.maze_dim and self.walls[loc[0], loc[1]+j, 1] ==-1:
                            self.walls[loc[0], loc[1]+j, 1] = w_l-j
                            if self.walls[loc[0], loc[1]+j, 3] == -1 and wall_info[3] != -1:
                                self.walls[loc[0], loc[1]+j, 3] = wall_info[3] + j
                        if i == 2 and loc[0] +j < self.maze_dim and self.walls[loc[0]+j, loc[1], 2] == -1:
                            self.walls[loc[0]+j, loc[1], 2] = w_l-j
                            if self.walls[loc[0]+j, loc[1], 0]  == -1 and wall_info[0] != -1: 
                                self.walls[loc[0]+j, loc[1], 0] = wall_info[0] + j
                        if i == 3 and loc[1] -j >= 0 and self.walls[loc[0]-j, loc[1], 3] == -1:
                            self.walls[loc[0], loc[1]-j, 3] = w_l-j  
                            if self.walls[loc[0], loc[1]-j, 1] == -1 and wall_info[1] != -1: 
                                self.walls[loc[0], loc[1]-j, 1] = wall_info[1] + j
                        j += 1
            
        #end of functions
        #===================================================================
        
        #first round(exploration and mapping) 
        if self.mapping == True:
            
            # end of first round
            if (self.step >= 970 or 0 not in self.explored) and 0 not in [self.explored[x, y] for x,y in self.goals]: #check if all explored
                self.mapping = False
                print [self.explored[x, y] for x,y in self.goals] #check if all explored
                print self.explored #check the allocation of locations visited
                print self.walls #check the wall information 

                #get path to the goal by calling A-Star
                self.path_to_goal = self.a_star.path_to_goal(self.walls, self.goal_entrance)
                self.location = [0,0]
                self.heading = 'up'
                rotation = 'Reset'
                movement = 'Reset'

            else:  #still places left to explore
                if self.explored[loc[0], loc[1]] > 0: #location is already explored at least once
                    w = self.walls[loc[0], loc[1], :] #wall info for the coordinate
                    #check for unexplored direction
                    indx = find_indices_for_walls(self.heading)
                    if len(w[w==-1]) == 1: 
                        find_neg1 = w.tolist().index(-1)
                        if find_neg1 in indx: #now know all four wall information
                            self.walls[loc[0], loc[1], find_neg1] = sensors[indx.index(find_neg1) ]
                            w_updated = self.walls[loc[0], loc[1], :] 
                            update_wall_info(self.location, w_updated) #update neighbor information
                    w_updated = self.walls[loc[0], loc[1], :] 
                    
                    next_locations = next_loc_from_wall(self.location, w_updated)
                    not_explored = [i for i in next_locations if self.explored[i[0], i[1]] == 0]
                    no_wall_info = np.where(np.array(w_updated) == -1)[0]
                    
                    #Actions
                    if len(not_explored) > 0:
                        # Try to go to goal state if not explored
                        has_goal_loc = [i for i in not_explored if i in self.goals]
                        if len(has_goal_loc) > 0 and [loc[0], loc[1]] not in self.goals:
                            new_loc = has_goal_loc[0]
                            self.goal_entrance = new_loc
                            choice = find_direction(loc[0], loc[1], new_loc[0], new_loc[1])
                            num_steps = find_distance(loc[0], loc[1], new_loc[0], new_loc[1])
                            self.goal_found = True
                        else: #choose randomly among non-explored locations
                            choice = np.random.choice(len(not_explored), 1)[0]
                            new_loc = not_explored[choice] 
                            choice = find_direction(loc[0], loc[1], new_loc[0], new_loc[1])
                            num_steps= find_distance(loc[0], loc[1], new_loc[0], new_loc[1])
                    else: #all next locations are explored
                        #choose location which has been least visited
                        explored_num = [self.explored[x,y] for x,y in next_locations]
                        lowest_explored = np.argmin(explored_num)
                        if len(no_wall_info) == 1 or len(next_locations) == 0: # if there is a direction without sensor info
                            chioce = heading_turn90[self.heading]
                            new_loc = self.location
                            num_steps=0
                            
                        else:
                            #go to least explored place
                            new_loc = next_locations[lowest_explored]
                            choice = find_direction(loc[0], loc[1], new_loc[0], new_loc[1])
                            num_steps= find_distance(loc[0], loc[1], new_loc[0], new_loc[1])
                    rotation, movement, self.heading = choose_action(self.heading, choice, num_steps)
                
                    self.location = new_loc
                    self.explored[loc[0], loc[1]] += 1
             
                else: #location has not been explored
                    indx = find_indices_for_walls(self.heading)
                    self.walls[loc[0], loc[1], indx[0]]  = sensors[0]
                    self.walls[loc[0], loc[1], indx[1]]  = sensors[1]
                    self.walls[loc[0], loc[1], indx[2]]  = sensors[2]
                    w = self.walls[loc[0], loc[1], :]
                    update_wall_info(self.location, w)
                    #if there is missing wall information
                    if -1 in w: 
                        rotation = 90
                        self.heading = heading_turn90[self.heading]
                        movement = 0
                    else:
                        next_locations = next_loc_from_wall(self.location, w)
                        not_explored = [i for i in next_locations if self.explored[i[0], i[1]] == 0]
                        explored_num = [self.explored[x,y] for x,y in next_locations]
                        lowest_explored = np.argmin(explored_num)
                        
                        #choose randomly among non-explored location
                        if len(not_explored) > 0: 
                            choice = np.random.choice(len(not_explored), 1)[0]
                            new_loc = not_explored[choice] 
                            choice = find_direction(loc[0], loc[1], new_loc[0], new_loc[1])
                            num_steps= find_distance(loc[0], loc[1], new_loc[0], new_loc[1])
                        #choose least explored place
                        else: 
                            new_loc = next_locations[lowest_explored]
                            choice = find_direction(loc[0], loc[1], new_loc[0], new_loc[1])
                            num_steps= find_distance(loc[0], loc[1], new_loc[0], new_loc[1])
                        rotation, movement, self.heading = choose_action(self.heading, choice, num_steps)
                        self.location = new_loc
                    self.explored[loc[0], loc[1]] += 1

        #second round (find fastest path) 
        else: 
            new_loc = self.path_to_goal[self.step_count]
            direction = find_direction(loc[0], loc[1], new_loc[0], new_loc[1])
            num_steps = find_distance(loc[0], loc[1], new_loc[0], new_loc[1])
            if num_steps <= 3:
                rotation, movement, self.heading = choose_action(self.heading, direction, num_steps)    
                self.location = new_loc
                self.step_count += 1 
            else:
                num_steps = 3
                rotation, movement, self.heading = choose_action(self.heading, direction, num_steps)    
                self.location = get_next_location(self.location, direction, num_steps)
        return rotation, movement


