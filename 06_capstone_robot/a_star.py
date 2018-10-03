import numpy as np
import Queue

class A_Star(object):
    def __init__(self, maze_dim, start):    
        self.queue = Queue.PriorityQueue()
        self.queue.put((0, start))
        self.maze_dim = maze_dim
        self.came_from = {}#previous location
        
        self.cost_so_far = {}
        #fastest path: (A star)  f = g + h(x,y)
        
        self.came_from[(start[0], start[1])] = None
        self.cost_so_far[(start[0], start[1])] = 0 #g-values
        self.start = start
        
    def path_to_goal(self, wall, goal_entrance):
        # useful functions        
        def find_h_value(x, y):
            #how many steps it would take to go from (x1,y1) to (x2,y2)
            x_dist = np.abs(x - goal_entrance[0])
            y_dist = np.abs(y - goal_entrance[1])
            return x_dist + y_dist

        def next_loc_from_wall(loc, w):
            next_locations = []
            for i in range(len(w)):
                if w[i] > 0: #only if there is no wall
                    j = 1
                    while j < w[i].astype(np.int32)+1:
                        if j > 3:
                            break
                        if i == 0 and loc[0]-j >= 0 : 
                            location = [loc[0]-j, loc[1] ]
                            next_locations.append(location)
                        if i == 1 and loc[1] + j < self.maze_dim: 
                            location = [loc[0],loc[1] + j]
                            next_locations.append(location)
                        if i == 2 and loc[0] + j < self.maze_dim:
                            location = [loc[0]+ j, loc[1]]
                            next_locations.append(location)
                        if i ==3 and loc[1] -j >= 0:
                            location = [loc[0], loc[1] -j]
                            next_locations.append(location)
                        j += 1 
            return next_locations
        #==============end of functions ======================
        while not self.queue.empty():
            current = self.queue.get()[1]
            if current[0] == goal_entrance[0] and current[1] == goal_entrance[1]:
                break
            w = wall[current[0], current[1], :]
            next_locations = next_loc_from_wall(current, w)
            for i in next_locations:
                g_value = self.cost_so_far[(current[0], current[1])] + 1
                new_cost = g_value + find_h_value(i[0], i[1])
                #if i has not been explored yet or cost is lower 
                if (i[0], i[1]) not in self.cost_so_far or new_cost < self.cost_so_far[(i[0], i[1])]:
                    self.cost_so_far[(i[0], i[1])] = new_cost
                    priority = new_cost 
                    self.queue.put((priority, (i[0], i[1])))
                    self.came_from[(i[0], i[1])] = current
        
        #back track to find the path from goal to start
        current = goal_entrance
        path = []
        path.append(current)
        while current != self.start:
            current = self.came_from[(current[0], current[1])]
            path.append(current)
        #path.append(self.start)
        path.reverse()
        print ('path ', path)
        return path
           

            