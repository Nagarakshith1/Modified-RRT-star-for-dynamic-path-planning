import numpy as np
from shapely.geometry import Point, MultiPoint
from math import log
import matplotlib.pyplot as plt
import matplotlib.patches as patches


class RRT():
    def __init__(self,obstacles_list):
        '''
        Initialization of variables of object of class RRT().

        Parameters
        ----------
        obstacles_list : list(Polygon)
            List of obstacles in the environment

        Returns
        -------
        None.

        '''
        self.nodes = -1*np.ones((10050,5)) # x, y, cost of segment b/w node and parent, parent, in_tree(1/0), id_number
        self.nodes_num = 1 # number of vertices
        self.start = np.array([0,0])
        self.goal = np.array([8,8])
        self.gamma = 30
        self.dimension = 2
        self.epsilon = 2
        self.p_current = 0
        self.obstacles = obstacles_list
        
    def sample(self):
        '''
        Function to sample a random point in a given region

        Returns
        -------
        x : numpy array (2x1)
            The random point generated which do not collide with any of the obstacles in the map.
            
        '''
        while True:
            x = 10*np.random.uniform(-1,1,size=(2))
            x_point = Point(x)
            # check if the value already prsent inside the tree
            collision = False
            for i,obs in enumerate(self.obstacles):
                check = obs.intersects(x_point)
                if check is True:
                    collision = True
                    break
            if collision is False:
                return x
            
    def get_cost(self,index):
        '''
        To get cost of reaching the desired node in a tree from the start

        Parameters
        ----------
        index : int
            index of the node in the tree

        Returns
        -------
        cost : float
            Cost measured in terms of the distance from the node to the start node.

        '''
        cost = 0
        if index is not None:
            while True:
                cost = cost + self.nodes[index][2]
                index = int(self.nodes[index][3])
                if(self.nodes[index][3]==self.p_current):
                    return cost
            return None
        else:
            return None

    def steer(self, x1, nearest):
        '''
        Function to return the optimal control trajectory of traveling from x1 to x2
        and the cost between these two points

        Parameters
        ----------
        x1 : numpy
            The previously randomly sampled point
        nearest : numpy
            The nearest node already present in the tree.

        Returns
        -------
        x1/x_new : numpy vector
            The new random vector which is feasible by the steering of the robot.
        distance : float
            The distance between the new random point and the selected nearest point.

        '''
        x_new = np.zeros((2))
        # distance is the cost of the path connecting the two points
        distance = np.linalg.norm(x1-nearest,axis=0)
        if distance>1:
            x_new[0] = nearest[0] + (x1[0]-nearest[0])/distance
            x_new[1] = nearest[1] + (x1[1]-nearest[1])/distance
            return x_new,1
        else:
            return x1,distance
        
    def get_dist(self,x1,x2):
        distance = np.linalg.norm(x2-x1)
        return distance
    
    def is_in_collision(self,point1,point2):
        '''
        Function to check if a trajectory between two node leads to a collision

        Parameters
        ----------
        point1 : numpy
            Point 1.
        point2 : numpy
            Point 2.

        Returns
        -------
        collision : Boolean
            Collsion check: True if the trajectory collided with any obstacle, else False.

        '''
        t = np.linspace(0,1,20)
        x = point1[0] + (point2[0]-point1[0]) * t
        y = point1[1] + (point2[1]-point1[1]) * t
        x = x.reshape((-1,1))
        y = y.reshape((-1,1))
        points = np.concatenate((x,y),axis=1)
        points = MultiPoint(points)
        collision = False
        for i,obs in enumerate(self.obstacles):
                check = obs.intersects(points)
                if check is True:
                    collision = True
                    break
        return collision  #true if there is an intersection with either obstacle
    
    def nearest(self, x_rand):
        '''
        Function to find the nearest node to a point pass to the function

        Parameters
        ----------
        x_rand : numpy
            Position of randomly sampled point.

        Returns
        -------
        p : int
            The tree index of the node which is closest to the point in question.

        '''
        x_nearest_dist = np.linalg.norm(self.nodes[0:self.nodes_num,0:2]-x_rand,axis=1) #NOTE: I haven't added the memeber to the tree yet
        
        x_nearest_ids = np.argsort(x_nearest_dist)
        p = x_nearest_ids[0]
        if p==self.nodes_num:
            return None
        else:
            return p
    
    def get_near_list(self, x_rand):
        '''
        Function to get list of indices preseny inside a volume (circle) of tuned radius

        Parameters
        ----------
        x_rand : numpy
            Randomly sampled point.

        Returns
        -------
        near_indices : list
            A list of position of nodes in the tree within a volume defined by radius defined by 'vol' variable.

        '''
        x_nearest_dist = np.linalg.norm(self.nodes[0:self.nodes_num,0:2]-x_rand,axis=1)
        x_nearest_ids = np.argsort(x_nearest_dist)
        # nodes_temp = np.concatenate((self.nodes[0:self.nodes_num+1,:],x_nearest_ids),axis=1)
        vol = self.gamma*(log(self.nodes_num)/self.nodes_num)**(1/self.dimension)
        vol = min(vol, self.epsilon)
        near_indices = x_nearest_ids[x_nearest_dist[x_nearest_ids]<=vol]
        return near_indices
    
    def nearest_from_list(self, x_rand, near_indices, reconnect):
        '''
        To find the node from neaar_indices which leads to lowest global cost for the node under consideration

        Parameters
        ----------
        x_rand : numpy
            Randomly sampled point.
        near_indices : list(int)
            List of indices with the vicinity of therandomly sampled point.
        reconnect : Boolean
            DESCRIPTION.

        Returns
        -------
        final_index : int
            The globally lowest cost parent for the random point.
        cost : float
            The cost vector for all the near_indices.

        '''
        cost = np.zeros_like(near_indices)
        for k,index in enumerate(near_indices):
            condition = self.is_in_collision(x_rand,self.nodes[index][0:2])
            
            if condition is False:
                cost[k] = self.get_cost(index)
                cost[k] = cost[k] + np.linalg.norm(x_rand-self.nodes[index][0:2])
            else:
                cost[k] = 1000000
        final_index = near_indices[np.argmin(cost)]
        if reconnect is False:
            cost_current = self.get_cost(self.nodes_num-1)
            if cost_current > np.min(cost):
                return final_index,cost
            else:
                return None,None
        else:
            return final_index,cost

    def connect(self, x, dist_index, cost_steer, new = False):
        '''
        add state x to the tree
        cost of state x = cost of the parent + cost of steer(parent, x)
        '''
        if new is True:
            self.nodes[self.nodes_num][0:2] = x
            self.nodes[self.nodes_num][2] = cost_steer # cost of that node
            self.nodes[self.nodes_num][3] = dist_index  # parent index
            self.nodes[self.nodes_num][4] = 1 # in tree condition
            
            self.nodes_num +=1 # adding up the total number of nodes
        else:
            self.nodes[self.nodes_num-1][0:2] = x
            self.nodes[self.nodes_num-1][2] = cost_steer # cost of that node
            self.nodes[self.nodes_num-1][3] = dist_index  # parent index
        
    def rewire(self, x_rand, near_indices):
        '''
        Function to rewire all nodes in the tree within the O(gamma (log n/n)ˆ{1/d}} ball
        near the state x with x as parent (if cost is lowered for the node), and thus update the costs of all rewired neighbors

        Parameters
        ----------
        x_rand : numpy vector
            The randomly sampled point.
        near_indices : list(int)
            The list of nodes in th vicinty of the randomly sampled point

        Returns
        -------
        None.

        '''
        # to obtain cost of the randomly generated point
        cost_new = self.get_cost(self.nodes_num-1)
        for i in near_indices:
            if self.nodes[i,3] == self.p_current:
                continue
            condition = self.is_in_collision(x_rand,self.nodes[i][0:2])
            if condition is False:
                dist = np.linalg.norm(x_rand-self.nodes[i][0:2])
                cost_near = cost_new + dist
                if cost_near<self.get_cost(i):
                    self.nodes[i][3] = self.nodes_num-1
                    self.nodes[i][2] = dist
                    
    def check_goal(self):
        '''
        To check if goal is present in the tree expanded and return the index
        of the node nearest to the goal. The path between this point and 
        the start is lowest global cost possible to reach the goal in the tree

        Returns
        -------
        bool
            Whether nodes in the vicinity of the nodes are found such that robot just reaches the goal.
        min_cost_idx : int, None
            The index of the node nearest to the goal such the path between this point and the start is lowest
            global cost possible to reach the goal in the tree.

        '''

        dist_from_goal = np.linalg.norm(self.nodes[0:self.nodes_num,0:2]-self.goal,axis=1)
        idx = np.argwhere(dist_from_goal<0.6).flatten()
        if len(idx) != 0:
            cost = [self.get_cost(i) for i in idx.flatten()]
            min_cost_idx = idx[np.argmin(np.array(cost))]
            return True, min_cost_idx
        else:
            return False, None
        
    def find_path(self, min_cost_idx):
        '''
        Function to find the path given an node index in the tree

        Parameters
        ----------
        min_cost_idx : int
            The node index.

        Returns
        -------
        path : list(int)
            A list of tree nodes the robot needs to traverse from the start to
            reach the goal.

        '''
        path = []
        path.append(min_cost_idx)

        min_cost_idx = int(min_cost_idx)
        while min_cost_idx!=self.p_current:
            p = int(self.nodes[min_cost_idx][3])
            path.append(p)
            min_cost_idx = p
        return path
    
    def plot(self, path=0, path_present=False):
        '''
        Function to plot the map with the robot

        Parameters
        ----------
        path : list(int), optional
            The list of path nodes (wrt index of the nodes in the tree). The default is 0.
        path_present : boolean, optional
            Indicator whether path is to be printed or not. The default is False.

        Returns
        -------
        None.

        '''
        # plt the grid space
        # plot the obstacles
        # plot the edges
        fig, ax = plt.subplots(figsize=(6,6))
        ax.axis('square')
        ax.set(xlim=(-10, 10), ylim=(-10, 10))
        rect1 = patches.Rectangle((-0.5, -7), 1, 14, fill=True, facecolor='gray')
        rect2 = patches.Rectangle((-7, -0.5), 14, 1, fill=True, facecolor='gray')
        ax.add_patch(rect1)
        ax.add_patch(rect2)
    
        for i in range(self.nodes_num):
            if self.nodes[i][4]==1:
                parent = int(self.nodes[i][3])
                x = [self.nodes[i][0],self.nodes[parent][0]]
                y = [self.nodes[i][1],self.nodes[parent][1]]
                plt.plot(x,y,color='cyan',linewidth=1,zorder=-1)
        if path_present is True:
            points_x = []
            points_y = []
            for i in path:
                points_x.append(self.nodes[i][0])
                points_y.append(self.nodes[i][1])
            plt.plot(points_x,points_y,color='red',linewidth=2)
            plt.plot(points_x[0],points_y[0],color='blue',zorder=0)
        plt.scatter(self.start[0],self.start[1],s=10**2,color='blue',zorder=1)
        plt.scatter(8,8,s=10**2,color='green',zorder=1)
        plt.show()