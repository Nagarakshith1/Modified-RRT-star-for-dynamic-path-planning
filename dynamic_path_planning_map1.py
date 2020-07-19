import timeit
import numpy as np
from shapely.geometry import Point, MultiPoint
from shapely.geometry.polygon import Polygon
from math import log
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import pickle   
import shapely
from queue import Queue
from obstacles import Obstacles
from RRT import RRT
from replan import Replan


def plot(nodes,robot_pose,obstacle_pose,plot_no=0,path=0,path_present=False,two_paths=False,sigma_separate=[]):
    '''
    FUnction to plot the nodes, trajectoris, the path and the robot motions

    Parameters
    ----------
    nodes : array
        The tree expanded.
    robot_pose : Point
        The current position of the robot.
    obstacle_pose : Point
        The current position of the obstacle in the environment.
    plot_no : int, optional
        Number of plot for indexing the plot for the purpose of saving. The default is 0.
    path : list(int)
        The list of path values represented by th eindices of the nodes in the tree.
    path_present : boolean, optional
        To determine whether to print path on plot or not. The default is False.
    two_paths : boolean, optional
        To determine whetehr there are two paths to printed or not. The default is False.
    sigma_separate : int(list), optional
        Takes in a list of nodes to plot on the pathon the map. The default is [].

    Returns
    -------
    None.

    '''
    # plt the grid space
    # plot the obstacles
    # plot the edges

    fig, ax = plt.subplots(figsize=(5, 5))
    ax.axis('square')
    ax.set(xlim=(-10, 10), ylim=(-10, 10))
    rect1 = patches.Rectangle((4, -4), 1, 13, fill=True, facecolor='gray')
    rect2 = patches.Rectangle((-6, -5), 6, 1, fill=True, facecolor='gray')
    ax.add_patch(rect1)
    ax.add_patch(rect2)

    valid_nodes = np.argwhere(nodes[:,4]==1).flatten()

    for j in range(valid_nodes.shape[0]):
        i = valid_nodes[j]
        if nodes[i][4] == 1:
            parent = int(nodes[i][3])
            if (parent ==-1 or nodes[parent,4]!=1):
                continue
            x = [nodes[i][0], nodes[parent][0]]
            y = [nodes[i][1], nodes[parent][1]]
            plt.plot(x, y, color='cyan', linewidth=1,zorder=-1)
    if path_present is True:
        points_x = []
        points_y = []
        for i in path:
            points_x.append(nodes[i][0])
            points_y.append(nodes[i][1])
        plt.plot(points_x, points_y, color='red', linewidth=2)
        plt.plot(points_x[0], points_y[0], color='blue', zorder=0)

        if two_paths:
            points_x = []
            points_y = []
            for i in sigma_separate:
                points_x.append(nodes[i][0])
                points_y.append(nodes[i][1])
            plt.plot(points_x, points_y, color='red', linewidth=2)
            plt.plot(points_x[0], points_y[0], color='blue', zorder=0)

    plt.scatter(robot_pose.x, robot_pose.y, s=10 ** 2, color='blue', zorder=1)
    plt.scatter(8, 8, s=10 ** 2, color='green', zorder=1)
    if obstacle_pose is not(None):
        plt.scatter(obstacle_pose.x,obstacle_pose.y, s=10**2, color = 'red', zorder=1)
    # plt.show()
    plt.savefig('Plots_map1/Fig'+str(plot_no))
    
##################################################################################################################
'''
Main Loop
Please run the function one at a time. To run another function, clear all the variables andre run with desired function
'''
 
def runDynamicObstacles():
    found, min_cost_idx = rrt.check_goal()
    if found is True:
        path = rrt.find_path(min_cost_idx)
    else:
        print('path not found')
    path.reverse()
    # starting point of the obstacle which is added after the RRT* is done
    obstacle_pose = Point(-5,5)
    prev_obstacle_pose = obstacle_pose
    new_obstacle = Polygon(obstacle_pose.buffer(1+0.01))
    i=-1
    path_len = len(path)
    plot_no = 0
    step = 0
    blocked_tree_indices = replan.removeOccupiedNodes(rrt, path, i, new_obstacle)
    rrt.nodes[blocked_tree_indices, 4] = 0
    while (i!=path_len-1):
        recheck_goal = False
        i+=1
        print(plot_no)
        node = path[i]
        rrt.p_current = node
        robot_pose = Point(rrt.nodes[int(path[i]), 0], rrt.nodes[int(path[i]), 1])

        if step == 3:

            rrt.nodes[rrt.nodes[:,4]==0,4]=1
            obstacle_pose = Point(3.5,-5.0)
            prev_obstacle_pose = obstacle_pose
            new_obstacle = Polygon(obstacle_pose.buffer(1 + 0.01))
            map_obstacles.addNewObstacle(new_obstacle)
            rrt.obstacles = map_obstacles.getObstacles()
            replan.prune_till_pcurr(node, rrt)
            path = path[i:]
            i = 0
            path_len = len(path)
            replan.modify_tree_and_path(rrt,path)
            rrt.p_current = path[0]
            recheck_goal = True
    
        if step == 8:

            rrt.nodes[rrt.nodes[:,4]==0,4]=1
            obstacle_pose = Point(7.5,-7.5)
            new_obstacle = Polygon(obstacle_pose.buffer(1 + 0.01))
            map_obstacles.updateObstacle(new_obstacle,prev_obstacle_pose)
            prev_obstacle_pose = obstacle_pose
            rrt.obstacles = map_obstacles.getObstacles()
            replan.prune_till_pcurr(node, rrt)
            path = path[i:]
            i = 0
            path_len = len(path)
            replan.modify_tree_and_path(rrt,path)
            rrt.p_current = path[0]
            recheck_goal = True
            blocked_tree_indices = replan.removeOccupiedNodes(rrt, path, i, new_obstacle)
            rrt.nodes[blocked_tree_indices, 4] = 0
    
        if step == 10:

            rrt.nodes[rrt.nodes[:,4]==0,4]=1
            obstacle_pose = Point(7.5,2.5)
            new_obstacle = Polygon(obstacle_pose.buffer(1 + 0.01))
            map_obstacles.updateObstacle(new_obstacle,prev_obstacle_pose)
            prev_obstacle_pose = obstacle_pose
            rrt.obstacles = map_obstacles.getObstacles()
            replan.prune_till_pcurr(node, rrt)
            path = path[i:]
            i = 0
            path_len = len(path)
            replan.modify_tree_and_path(rrt,path)
            rrt.p_current = path[0]
            recheck_goal = True
    
        if step == 12:

            rrt.nodes[rrt.nodes[:,4]==0,4]=1
            obstacle_pose = Point(7.5,-7.5)
            new_obstacle = Polygon(obstacle_pose.buffer(1 + 0.01))
            map_obstacles.updateObstacle(new_obstacle,prev_obstacle_pose)
            prev_obstacle_pose = obstacle_pose
            rrt.obstacles = map_obstacles.getObstacles()
            replan.prune_till_pcurr(node, rrt)
            path = path[i:]
            i = 0
            path_len = len(path)
            replan.modify_tree_and_path(rrt,path)
            rrt.p_current = path[0]
            recheck_goal = True
            blocked_tree_indices = replan.removeOccupiedNodes(rrt, path, i, new_obstacle)
            rrt.nodes[blocked_tree_indices, 4] = 0
    
        flag_obstacle = replan.detectObstacle(rrt,path,new_obstacle)
    
        if flag_obstacle is True:
            recheck_goal = False
            print('Obstacle encountered')
            sigma_current, sigma_separate, obstructed_path_ids, blocked_tree_indices = replan.removeOccupiedNodes(rrt, path, i, new_obstacle,True)
            plot(rrt.nodes, robot_pose, obstacle_pose, plot_no, sigma_current, True,True,sigma_separate)
            plot_no += 1
    
            replan.modify_tree_and_path(rrt,sigma_current,sigma_separate, blocked_tree_indices)
            plot(rrt.nodes, robot_pose, obstacle_pose, plot_no, sigma_current, True,True,sigma_separate)
            plot_no += 1
    
            rrt.p_current = sigma_current[0]
            print('Reconnecting.....')
            flag_reconnect, new_path = replan.reconnect(rrt,sigma_current, sigma_separate)
            if flag_reconnect:
                path = new_path
                i= 0
                path_len = len(path)
                plot(rrt.nodes, robot_pose, obstacle_pose, plot_no, path, True)
                plot_no += 1
            else:
                print('Reconnect failed... trying regrow')
                temp = -1*np.ones((3000,6))
                rrt.nodes = np.vstack((rrt.nodes,temp))
                rrt.nodes[:,5] = np.arange(rrt.nodes.shape[0])
                
                for i in range(1500):
                    replan.regrow(rrt,new_obstacle,sigma_separate[0])
                flag_reconnect, new_path = replan.reconnect(rrt, sigma_current, sigma_separate)
                if flag_reconnect is True:
                    path = new_path
                    i = 0
                    path_len = len(path)
                    plot(rrt.nodes, robot_pose, obstacle_pose, plot_no, path, True)
                    plot_no+=1
                else:
                    print('Regrow failed')
        else:
            plot(rrt.nodes,robot_pose,obstacle_pose,plot_no,path[i:],True)
            plot_no += 1
    
        if recheck_goal is True:
            if (plot_no !=0):
                flag,min_cost_idx = replan.check_goal(rrt)
                path,cost = replan.find_path(rrt,min_cost_idx)
                i = 0
                path_len = len(path)
        step = step+1

def runReconnect():
    found, min_cost_idx = rrt.check_goal()
    if found is True:
        path = rrt.find_path(min_cost_idx)
    else:
        print('path not found')
    path.reverse()
    # starting point of the obstacle which is added after the RRT* is done
    obstacle_pose = Point(-5,5)
    prev_obstacle_pose = obstacle_pose
    new_obstacle = Polygon(obstacle_pose.buffer(1+0.01))
    i=-1
    path_len = len(path)
    plot_no = 0
    step = 0
    blocked_tree_indices = replan.removeOccupiedNodes(rrt, path, i, new_obstacle)
    rrt.nodes[blocked_tree_indices, 4] = 0
    while (i!=path_len-1):
        recheck_goal = False
        i+=1
        print(plot_no)
        node = path[i]
        rrt.p_current = node
        robot_pose = Point(rrt.nodes[int(path[i]), 0], rrt.nodes[int(path[i]), 1])

        if step == 3:
            rrt.nodes[rrt.nodes[:,4]==0,4]=1
            obstacle_pose = Point(6.5,2.5)
            prev_obstacle_pose = obstacle_pose
            new_obstacle = Polygon(obstacle_pose.buffer(1 + 0.01))
            map_obstacles.addNewObstacle(new_obstacle)
            rrt.obstacles = map_obstacles.getObstacles()
            replan.prune_till_pcurr(node, rrt)
            path = path[i:]
            i = 0
            path_len = len(path)
            replan.modify_tree_and_path(rrt,path)
            rrt.p_current = path[0]
            recheck_goal = True
    
        flag_obstacle = replan.detectObstacle(rrt,path,new_obstacle)
    
        if flag_obstacle is True:
            print('Obstacle encountered')
            sigma_current, sigma_separate, obstructed_path_ids, blocked_tree_indices = replan.removeOccupiedNodes(rrt, path, i, new_obstacle,True)
            plot(rrt.nodes, robot_pose, obstacle_pose, plot_no, sigma_current, True,True,sigma_separate)
            plot_no += 1
    
            replan.modify_tree_and_path(rrt,sigma_current,sigma_separate, blocked_tree_indices)
            plot(rrt.nodes, robot_pose, obstacle_pose, plot_no, sigma_current, True,True,sigma_separate)
            plot_no += 1
    
            rrt.p_current = sigma_current[0]
            print('Reconnecting.....')
            flag_reconnect, new_path = replan.reconnect(rrt,sigma_current, sigma_separate)
            if flag_reconnect:
                path = new_path
                i= 0
                path_len = len(path)
                plot(rrt.nodes, robot_pose, obstacle_pose, plot_no, path, True)
                plot_no += 1
            else:
                print('Reconnect failed trying regrow')
                temp = -1*np.ones((3000,6))
                rrt.nodes = np.vstack((rrt.nodes,temp))
                rrt.nodes[:,5] = np.arange(rrt.nodes.shape[0])
                for i in range(1500):
                    replan.regrow(rrt,new_obstacle,sigma_separate[0])
                flag_reconnect, new_path = replan.reconnect(rrt, sigma_current, sigma_separate)
                if flag_reconnect is True:
                    path = new_path
                    i = 0
                    path_len = len(path)
                    plot(rrt.nodes, robot_pose, obstacle_pose, plot_no, path, True)
                    plot_no+=1
                else:
                    print('Regrow failed')
        else:
            
            plot(rrt.nodes,robot_pose,obstacle_pose,plot_no,path[i:],True)
            plot_no+=1
                
        if recheck_goal is True:
            if (plot_no !=0):
                flag,min_cost_idx = replan.check_goal(rrt)
                path,cost = replan.find_path(rrt,min_cost_idx)
                i = 0
                path_len = len(path)
        step = step+1       
        
def runRegrow():
    found, min_cost_idx = rrt.check_goal()
    if found is True:
        path = rrt.find_path(min_cost_idx)
    else:
        print('path not found')
    path.reverse()
    obstacle_pose = Point(-5,5)
    prev_obstacle_pose = obstacle_pose
    new_obstacle = Polygon(obstacle_pose.buffer(1+0.01))
    i=-1
    path_len = len(path)
    plot_no = 0
    step = 0
    blocked_tree_indices = replan.removeOccupiedNodes(rrt, path, i, new_obstacle)
    rrt.nodes[blocked_tree_indices, 4] = 0
    while (i!=path_len-1):
        recheck_goal = False
        i+=1
        print(plot_no)
        node = path[i]
        rrt.p_current = node
        robot_pose = Point(rrt.nodes[int(path[i]), 0], rrt.nodes[int(path[i]), 1])

        if step == 3:

            rrt.nodes[rrt.nodes[:,4]==0,4]=1
            obstacle_pose = Point(3.5,-5.0)
            prev_obstacle_pose = obstacle_pose
            new_obstacle = Polygon(obstacle_pose.buffer(1 + 0.01))
            map_obstacles.addNewObstacle(new_obstacle)
            rrt.obstacles = map_obstacles.getObstacles()
            replan.prune_till_pcurr(node, rrt)
            path = path[i:]
            i = 0
            path_len = len(path)
            replan.modify_tree_and_path(rrt,path)
            rrt.p_current = path[0]
            recheck_goal = True
    
        flag_obstacle = replan.detectObstacle(rrt,path,new_obstacle)
    
        if flag_obstacle is True:
            print('Obstacle encountered')
            sigma_current, sigma_separate, obstructed_path_ids, blocked_tree_indices = replan.removeOccupiedNodes(rrt, path, i, new_obstacle,True)
            plot(rrt.nodes, robot_pose, obstacle_pose, plot_no, sigma_current, True,True,sigma_separate)
            plot_no += 1
    
            replan.modify_tree_and_path(rrt,sigma_current,sigma_separate, blocked_tree_indices)
            plot(rrt.nodes, robot_pose, obstacle_pose, plot_no, sigma_current, True,True,sigma_separate)
            plot_no += 1
    
            rrt.p_current = sigma_current[0]
            print('Reconnecting.....')
            flag_reconnect, new_path = replan.reconnect(rrt,sigma_current, sigma_separate)
            if flag_reconnect:
                path = new_path
                i= 0
                path_len = len(path)
                plot(rrt.nodes, robot_pose, obstacle_pose, plot_no, path, True)
                plot_no += 1
            else:
                j = 0
                print('Reconnect failed trying regrow')
                temp = -1*np.ones((3000,6))
                rrt.nodes = np.vstack((rrt.nodes,temp))
                rrt.nodes[:,5] = np.arange(rrt.nodes.shape[0])
                
                for i in range(1500):
                    replan.regrow(rrt,new_obstacle,sigma_separate[0])
                flag_reconnect, new_path = replan.reconnect(rrt, sigma_current, sigma_separate)
                if flag_reconnect is True:
                    path = new_path
                    i = 0
                    path_len = len(path)
                    plot(rrt.nodes, robot_pose, obstacle_pose, plot_no, path, True)
                    plot_no+=1
                else:
                    print('Regrow failed')
        else:
            plot_no+=1
            if (plot_no > 3):
                plot(rrt.nodes,robot_pose,obstacle_pose,plot_no,path[i:],True)
                
        if recheck_goal is True:
            if (plot_no !=0):
                flag,min_cost_idx = replan.check_goal(rrt)
                path,cost = replan.find_path(rrt,min_cost_idx)
                print('Latest Path Cost:')
                print(cost)
                i = 0
                path_len = len(path)
        step = step+1
        
# Code for testing (please ignore)
def RNN():
    
    found, min_cost_idx = rrt.check_goal()
    if found is True:
        path = rrt.find_path(min_cost_idx)
    else:
        print('path not found')
    path.reverse()
    # starting point of the obstacle which is added after the RRT* is done
    obstacle_pose = Point(-5,5)
    prev_obstacle_pose = obstacle_pose
    new_obstacle = Polygon(obstacle_pose.buffer(1+0.01))
    i=-1
    path_len = len(path)
    plot_no = 0
    step = 0
    while (i!=path_len-1):
        recheck_goal = False
        i+=1
        node = path[i]
        rrt.p_current = node
        robot_pose = Point(rrt.nodes[int(path[i]), 0], rrt.nodes[int(path[i]), 1])

        if step == 3:

            rrt.nodes[rrt.nodes[:,4]==0,4]=1
            obstacle_pose = Point(7.5,2.5)
            prev_obstacle_pose = obstacle_pose
            new_obstacle = Polygon(obstacle_pose.buffer(1 + 0.01))
            map_obstacles.addNewObstacle(new_obstacle)
            rrt.obstacles = map_obstacles.getObstacles()
            replan.prune_till_pcurr(node, rrt)
            path = path[i:]
            i = 0
            path_len = len(path)
            replan.modify_tree_and_path(rrt,path)
            rrt.p_current = path[0]
            recheck_goal = True
          
        flag_obstacle = replan.detectObstacle(rrt,path,new_obstacle)
    
        if flag_obstacle is True:
            print('Obstacle encountered')
            rrt2 = RRT(rrt.obstacles)
            rrt2.p_current = 0
            rrt2.start = rrt.nodes[rrt.p_current,0:2]
            rrt2.nodes[0,0:2] = rrt2.start
            rrt2.nodes[0][2] = 0
            rrt2.nodes[0][3] = 0
            j = 0
            replan.prune_till_pcurr(rrt.p_current, rrt)
            replan.modify_tree_and_path(rrt,path)
            for i in range(6000):
                x = rrt2.sample()
                nearest_index = rrt2.nearest(x)
                if nearest_index is None:
                    continue
                x_new, cost_steer = rrt2.steer(x, rrt2.nodes[nearest_index][0:2])
                check = rrt2.is_in_collision(x_new,rrt2.nodes[nearest_index][0:2])
                if check is False:
                    near_indices = rrt2.get_near_list(x_new)
                    rrt2.connect(x_new, nearest_index, cost_steer,new=True)
                    if near_indices.size != 0:
                        best_index, cost = rrt2.nearest_from_list(x_new,near_indices,reconnect=False)
                        if best_index is not(None):
                            cost_steer = rrt2.get_dist(x_new,rrt2.nodes[best_index][0:2])
                            rrt2.connect(x_new, best_index,cost_steer,new=False)
                        rrt2.rewire(x_new,near_indices)
                    
                found, min_cost_idx = rrt2.check_goal()
            if found is True:
                path = rrt2.find_path(min_cost_idx)
                plot(rrt2.nodes,robot_pose,None,0,path,True)
                break
            break

        step = step+1
        
# main functioon to intial RRT* algorithm
map_obstacles = Obstacles()
obstacles = []
obstacles.append(Polygon([(-6.6,-3.4),(0.6,-3.4),(0.6,-5.6),(-6.6,-5.6)]))
obstacles.append(Polygon([(3.4,-4.6),(5.6,-4.6),(5.6,9.6),(3.4,9.6)]))
obstacles.append(Polygon([(-10,9.4),(-10,10),(10,10),(10,9.4)]))
obstacles.append(Polygon([(-10,10),(-10,-10),(-9.4,-10),(-9.4,10)]))
obstacles.append(Polygon([(-10,-10),(10,-10),(10,-9.4),(-10,-9.4)]))
obstacles.append(Polygon([(9.4,-10),(10,-10),(10,10),(9.4,10)]))

map_obstacles.addNewObstacle(obstacles)
obstacles_list = map_obstacles.getObstacles()
cost_total = []

try:
    with open('rrt_map1.pkl', 'rb') as input:
        rrt = pickle.load(input)
except:
    print('Pickle file of RRT* data for map 1 not found---Running RRT*')
    rrt.start = np.array([-9,-9])
    rrt.nodes[0][0:2] = rrt.start
    rrt.nodes[0][2] = 0
    rrt.nodes[0][3] = 0
    for i in range(8000):
        print(i)
        x = rrt.sample()
        nearest_index = rrt.nearest(x)
        if nearest_index is None:
            continue
        x_new, cost_steer = rrt.steer(x, rrt.nodes[nearest_index][0:2])
        check = rrt.is_in_collision(x_new,rrt.nodes[nearest_index][0:2])
        if check is False:
            near_indices = rrt.get_near_list(x_new)
            rrt.connect(x_new, nearest_index, cost_steer,new=True)
            if near_indices.size != 0:
                best_index, cost = rrt.nearest_from_list(x_new,near_indices,reconnect=False)
                if best_index is not(None):
                    cost_steer = rrt.get_dist(x_new,rrt.nodes[best_index][0:2])
                    rrt.connect(x_new, best_index,cost_steer,new=False)
                rrt.rewire(x_new,near_indices)
            
    # final (after 10,000 iterations)
    found, min_cost_idx = rrt.check_goal()
    if found is True:
        path = rrt.find_path(min_cost_idx)
        rrt.plot(path,path_present=True)
    else:
        print('path not found')
    rrt.plot()
    rrt.plot(path,path_present=True)
    path.reverse()
    print(path)
    cost_total.append(rrt.get_cost(min_cost_idx))
    print(rrt.get_cost(min_cost_idx))
    
    with open('rrt_map1.pkl', 'wb') as output:
        pickle.dump(rrt, output, pickle.HIGHEST_PROTOCOL)

rrt.nodes = np.hstack((rrt.nodes,np.arange(rrt.nodes.shape[0]).reshape(rrt.nodes.shape[0],1)))
    
found, min_cost_idx = rrt.check_goal()
if found is True:
    path = rrt.find_path(min_cost_idx)
else:
    print('path not found')
path.reverse()
rrt.nodes = rrt.nodes[0:rrt.nodes_num,:]
plot(rrt.nodes,Point((0,0)),None,0,path,True)
replan = Replan()

 
# Function calls. Please run only one of them at a time. Wheb running one, comment out the other two calls
runDynamicObstacles()
# runReconnect()
# runRegrow()