import numpy as np
from shapely.geometry import Point, MultiPoint
from queue import Queue


class Replan():
    
    def __init__(self):
        pass
        
    def bfs_removal(self,node_id,rrt,blocked_nodes = False,ignore_id=-1):
        '''
        

        Parameters
        ----------
        node_id : int
            DESCRIPTION.
        rrt : object of RRT()
            Object which stores information of the tree expanded by RRT*.
        blocked_nodes : boolean, optional
            A boolean to which determies whether nodes are obstructed by dynamic
            obstacles. The default is False.
        ignore_id : integer, optional
            The child node (which lies on the path) ignored fpr deletion when 
            using the BFS method for pruning the tree. The default is -1.

        Returns
        -------
        None.

        '''
        q = Queue()
        q.put(node_id)
        while(not q.empty()):
            parent_id = q.get()
            if blocked_nodes:
                rrt.nodes[parent_id,4] = 0
            else:
                rrt.nodes[parent_id, 4] = 2
            child_ids = np.argwhere(np.logical_and(rrt.nodes[:,3]==parent_id, np.logical_or(rrt.nodes[:,4]==1, rrt.nodes[:,4]==0))).flatten()
            # child_ids = np.argwhere(rrt.nodes[:,3]==parent_id).flatten()
            child_ids = child_ids[child_ids!=ignore_id]
            if (child_ids.shape[0]):
                if blocked_nodes:
                    rrt.nodes[child_ids,4] = 0
                else:
                    rrt.nodes[child_ids, 4] = 2
                for i in range(child_ids.shape[0]):
                    q.put(child_ids[i])

    def prune_till_pcurr(self,pcurr_id,rrt):
        '''
        Prune the tree till the current node where the robot is present

        Parameters
        ----------
        pcurr_id : int
            Index of the current position of the robot according to the index in the tree.
        rrt : object of RRT()
            Object which stores information of the tree expanded by RRT*..

        Returns
        -------
        None.

        '''
        # find all the parent_ids till start
        list_id = [pcurr_id]

        # Get the current of the
        parent_id = int(rrt.nodes[pcurr_id,3])

        old_start_id = int(np.argwhere(rrt.nodes[:,3] == rrt.nodes[:,5])[0])

        while(parent_id != old_start_id):
            list_id.append(parent_id)
            parent_id = int(rrt.nodes[parent_id, 3])
        list_id.append(old_start_id)

        list_id = list_id[::-1]
        for i in range(len(list_id)-1):
            self.bfs_removal(list_id[i],rrt,False,list_id[i+1])

    def modify_tree_and_path(self,rrt,sigma_current=[],sigma_separate=[], blocked_tree_indices = []):
        '''
        Function to modify the path segments and the tree according to the nodes
        to be pruned. 

        Parameters
        ----------
        rrt : object of RRT()
            Object which stores information of the tree expanded by RRT*.
        sigma_current : list(int), optional
            The segment of path before the obstacle. The default is [].
        sigma_separate : list(int), optional
            The segment of the path after the obstacle. The default is [].
        blocked_tree_indices : list(int), optional
            The list of nodes in the tree blocked out by the
            new obstacle. The default is [].

        Returns
        -------
        None.

        '''
        invalid_ids = np.argwhere(rrt.nodes[:,4]==2).flatten()
        for index in blocked_tree_indices:
            self.bfs_removal(index,rrt,True,sigma_separate[0])
        rrt.nodes = np.delete(rrt.nodes,invalid_ids,0)
        rrt.nodes_num-= invalid_ids.shape[0]
        if len(rrt.nodes)==0:
            return
        for i,u_id in enumerate(sigma_current):
            array_id = int(np.argwhere(rrt.nodes[:,5]==u_id)[0])
            sigma_current[i] = array_id
        for i,u_id in enumerate(sigma_separate):
            array_id = int(np.argwhere(rrt.nodes[:,5]==u_id)[0])
            sigma_separate[i] = array_id
    
        #update the parent ids of all children
        for i in range(0,rrt.nodes_num):
            if sigma_separate  !=[]:
                if i == sigma_separate[0]:
                    rrt.nodes[i,3] = -1
                    continue
            parent_uid = rrt.nodes[i,3]
            if(sum(1*rrt.nodes[:,5]==parent_uid)==0):
                rrt.nodes[i, 3] = i
                continue
            parent_aid = int(np.argwhere(rrt.nodes[:,5]==parent_uid)[0])
            rrt.nodes[i,3] = parent_aid
        rrt.nodes[:,5] = np.arange(rrt.nodes.shape[0])


    def detectObstacle(self, rrt, path, newObstacle):
        '''
        Function to detect whether obstacle is actually bloocking the path or not

        Parameters
        ----------
        rrt : object of RRT()
            Object which stores information of the tree expanded by RRT*.
        path : list(int)
            List of indices of the nodes in the tree which form the path traversed
            by the robot.
        newObstacle : Polygon
            The dynamic obstacle in question.

        Returns
        -------
        bool
            A boolean which determines whetehr obstacle intersects the path (TRUE)
            or not (FALSE).

        '''
        points = []
        points = rrt.nodes[path,0:2]
        points_shapely = MultiPoint(points)
        if points_shapely.intersects(newObstacle):
            return True
        else:
            return False
    
    def removeOccupiedNodes(self, rrt, path, p_current_index, new_obstacle,path_blocked = False):
        '''
        Function to determine all the nodes in the tree occuped by the dynamic/new obstacle

        Parameters
        ----------
        rrt : object of RRT()
            Object which stores information of the tree expanded by RRT*.
        path : list(int)
            List of indices of the nodes in the tree which form the path traversed
            by the robot.
        p_current_index : int
            The current node in the tree where the robot is present.
        newObstacle : Polygon
            The dynamic obstacle in question.
        path_blocked : Boolean, optional
            A check condition which determines whether only blocked nodes are returned
            or the split path segments are also returned as well. The default is False.

        Returns
        -------
        sigma_current : list(int)
            The part of path before the obstacle.
        sigma_separate : list(int)
            The part of path after the obstacle.
        obstructed_ids : list(int)
            The part of path obstructed by the obstacle.
        indices: list(int)
            List of indices of the tree blocked by the new obstacle.

        '''
        if path_blocked is True:
            obstructed_path_nodes = [] # node tree indices
            obstructed_ids = [] # path indices
            for i in range(len(path)-1): # no need to check for the goal position
                point = Point(tuple(rrt.nodes[path[i],0:2]))
                if point.intersects(new_obstacle):
                    obstructed_path_nodes.append(path[i])
                    obstructed_ids.append(i)

            # the separated out list of paths between the new obstacle
            sigma_separate = path[obstructed_ids[-1]+1:]
            sigma_current = path[p_current_index:obstructed_ids[0]]

            ## call function to remove all children of each of these nodes

            # subsection to collect all nodes of the tree lying inside/on 
            # the obstacle (make obstacle slightly larger than like 1.01 instead of 1 to consider cases whenpoint lies on circle)
            indices = []

            for i in range(rrt.nodes_num):
                point = Point(rrt.nodes[i,0:2])
                indices.append(new_obstacle.intersects(point))
            indices = np.argwhere(np.asarray(indices))
            indices = indices.reshape(-1) # all indices in tree which lie inside new obstacle

            return sigma_current, sigma_separate, obstructed_ids, indices
        else:
            indices = []

            for i in range(rrt.nodes_num):
                point = Point(rrt.nodes[i, 0:2])
                indices.append(new_obstacle.intersects(point))
            indices = np.argwhere(np.asarray(indices))
            indices = indices.reshape(-1)  # all indices in tree which lie inside new obstacle

            return indices
    
    def find_path(self, rrt, node, break_id = -1):
        '''
        Function to find path from the tree after replanning

        Parameters
        ----------
        rrt : object of RRT()
            Object which stores information of the tree expanded by RRT*.
        node : int
            The node from whcih the path to the start is to be calculated.
        break_id : int, optional
            A condition when infinity cost is returned as path cannot be found. The default is -1.

        Returns
        -------
        path : list(int)
            The list of path values represented by the indices of the nodes in the tree.
        cost : float
            The value of total cost measured in euclidean distance.

        '''
        path = [node]
        cost = rrt.nodes[node,2]
        node = int(node)
        while node != rrt.p_current:
            p = int(rrt.nodes[node][3])
            if(p == break_id or p == -1 or rrt.nodes[p,4]==0):
                return [],np.infty
            cost = cost + rrt.nodes[p,2]
            path.append(p)
            node = p
        path.reverse()
        return path,cost
    
    
    def reconnect(self,rrt, sigma_current, sigma_separate):
        '''
        Reconnect function tries to connect the nodes on sigma_separate with 
        the nearest node of sigma_current to form the globally lowest cost path
        possible on the new tree structure
        
        Parameters
        ----------
        rrt : object of RRT()
            Object which stores information of the tree expanded by RRT*.
        sigma_current : list(int)
            The part of path before the obstacle.
        sigma_separate : list(int)
            The part of path after the obstacle.

        Returns
        -------
        bool
            Boolean to determine whether the reconnect process is successful or not.
        best_path : list(int) (OR) []
            The best new path of the robot. An empty list is returned of reconnect fails.

        '''
        radius = 0.75
        for i,separate_node in enumerate(sigma_separate):
            parent_uid = rrt.nodes[i,3]
            distance = np.linalg.norm(rrt.nodes[:,0:2]-rrt.nodes[separate_node,0:2],axis=1)
            potential_parents = np.argwhere((np.logical_and(distance<=radius, rrt.nodes[:,4]==1))).reshape(-1)
            potential_parents = self.modify_near_indices(rrt,potential_parents,sigma_separate[0])
            if len(potential_parents)!=0:
                min_cost = 100000
                best_path = []
                for j,potential_parent in enumerate(potential_parents):
                    path,cost = self.find_path(rrt,potential_parent,sigma_separate[0])
                    cost = cost + np.linalg.norm(rrt.nodes[potential_parent,0:2]-rrt.nodes[separate_node,0:2])
                    if cost<min_cost:
                        min_cost = cost
                        best_path = path
                        rrt.nodes[separate_node,3] = potential_parent
                        rrt.nodes[separate_node,2] = np.linalg.norm(rrt.nodes[potential_parent,0:2]-rrt.nodes[separate_node,0:2])
                if(best_path==[]):
                    return False, []
                best_path = best_path + sigma_separate[i:]
                return True,best_path
        return False,[]
        
    def nearest_regrow(self, rrt, x, sigma_separate_0):
        '''
        Function which works in tandem with regrow() to determine the nearest node to
        a random point while considering the fact that some nodes are temporarily
        blocked by dynamic obstcales

        Parameters
        ----------
        rrt : object of RRT()
            Object which stores information of the tree expanded by RRT*.
        x : numpy
            The position of point.
        sigma_separate_0 : int
            The frst node of sigma_separate.

        Returns
        -------
        p : int
            Th nearest node in terms of distance to x_rand.

        '''
        # assuming we have already deleted all the nodes bloacked by the obstacle
        
        x_nearest_dist = np.linalg.norm(rrt.nodes[0:rrt.nodes_num,0:2]-x,axis=1) #NOTE: I haven't added the member to the tree yet
        
        x_nearest_ids = np.argsort(x_nearest_dist)
        i = 0
        while True:
            p = x_nearest_ids[i]
            if rrt.nodes[p,4]==0:
                i = i+1
                continue
            output = self.modify_near_indices(rrt,np.array(p).flatten(),sigma_separate_0)
            if len(output)!=0:
                return p
            i = i+1
            if i == rrt.nodes_num:
                return None
    
    def regrow(self, rrt, new_obstacle, sigma_separate_0):
        '''
        Function to perform the regrowth of the tree to generate new possible trajectories

        Parameters
        ----------
        rrt : object of RRT()
            Object which stores information of the tree expanded by RRT*.
        new_obstacle : Polygpon
            The obstacle blocking the path.
        sigma_separate_0 : int
            The first node on sigma_separate list.

        Returns
        -------
        None.

        '''
        while True:
            x = rrt.sample()
            nearest_index = self.nearest_regrow(rrt, x, sigma_separate_0)
            x_new, cost_steer = rrt.steer(x, rrt.nodes[nearest_index][0:2])
            check = rrt.is_in_collision(x_new,rrt.nodes[nearest_index][0:2])
            if check is True:
                continue
            else:
                break
        near_indices = rrt.get_near_list(x_new)
        rrt.connect(x_new, nearest_index, cost_steer,new=True)
        if near_indices.shape[0] != 0:
            near_indices = self.modify_near_indices(rrt, near_indices, sigma_separate_0)
            near_indices = near_indices[np.argwhere(rrt.nodes[near_indices,4]==1).flatten()]
    
        if near_indices.shape[0] != 0:
            best_index, cost = rrt.nearest_from_list(x_new,near_indices,reconnect=False)
            if best_index is not(None):
                cost_steer = rrt.get_dist(x_new,rrt.nodes[best_index][0:2])
                rrt.connect(x_new, best_index,cost_steer,new=False)
            rrt.rewire(x_new,near_indices)
    
    def modify_near_indices(self, rrt, near_indices, sigma_separate_0):
        '''
        Function to update the near_indices output so that nodes from
        sigma_current are not considered (called only in the regrow() function)

        Parameters
        ----------
        rrt : object of RRT()
            Object which stores information of the tree expanded by RRT*.
        near_indices : list(int)
            The list of nodes in the vicinity of the sampled point.
        sigma_separate_0 : int
            The first node on the sigma_separate list.

        Returns
        -------
        near_indices : list(int)
            Updated list of nearby nodes so that they do not contain sigma_current 
            nodes or their relatives
            
        '''
        near_indices_copy = near_indices
        for i in near_indices_copy:
            parent_id = int(rrt.nodes[i,3])
            if parent_id == -1:
                near_indices = near_indices[near_indices!=i]
                continue
            while parent_id!= rrt.p_current:
                if parent_id == sigma_separate_0:
                    near_indices = near_indices[near_indices!=i]
                    break
                parent_id = int(rrt.nodes[parent_id,3])
        return near_indices
    
    def check_goal(self,rrt):
        '''
        Function to check for goal after replanning

        Parameters
        ----------
        rrt : object of RRT()
            Object which stores information of the tree expanded by RRT*.

        Returns
        -------
        bool
            Boolean to determine whether a path is found or not.
        TYPE
            The index in thetree which is close to the obstacle and leads to
            the lowest global cost from the robot current position to the goal.

        '''
        dist_from_goal = np.linalg.norm(rrt.nodes[0:rrt.nodes_num,0:2]-rrt.goal,axis=1)
        idx = np.argwhere(np.logical_and(dist_from_goal<0.6,rrt.nodes[0:rrt.nodes_num,4]==1)).flatten()

        if idx is not(None):
            cost = [rrt.get_cost(i) for i in idx.flatten()]
            min_cost_idx = idx[np.argmin(np.array(cost))]
            return True, min_cost_idx
        else:
            return False, None