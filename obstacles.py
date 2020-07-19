

class Obstacles():
    '''
    Type: Class
    To store details of the obstacles on the map
    '''
    def __init__(self):
        '''
        Initialize values for class variables
        self.obstacles_list: list of obstacles where each obstacle is reprsented by the type shapely.geometry.Polygon from the shapely class
        '''
        self.obstacles_list = []
        
    def getObstacles(self):
        return self.obstacles_list
    
    def addNewObstacle(self,new_obstacle):
        '''
        Add an obstacle to the map
        If list is passed to the function, then the list is extended, else the Polygon obstacle is appended
        new_obstacle: list of Polygons or Polygon
        '''
        if type(new_obstacle)==list:
            self.obstacles_list.extend(new_obstacle)
        else:
            self.obstacles_list.append(new_obstacle)
            
    def updateObstacle(self,new_obstacle,old_obstacle):
        '''
        
        Update the obstacle present already in the map.
        The old obstacle is removed and a new obstacle is added
        
        Parameters
        ----------
        new_obstacle : Polygon
            representing the new obstacle.
        old_obstacle : Polygon
            represemting the obstacle to be updated.
        
        Returns
        -------
        None.

        '''
        for i,obs in enumerate(self.obstacles_list):
            if old_obstacle==obs:
                del self.obstacles_list[i]
                self.obstacles_list.append(new_obstacle)
            else:
                print('false')
            
    def removeObstacle(self,removed_obstacle):
        '''
        
        Parameters
        ----------
        removed_obstacle : Polygon
            Remove an obstacle completely from the list of obstacles

        Returns
        -------
        None.

        '''
        for i,obs in enumerate(self.obstacles_list):
            if removed_obstacle==obs:
                del self.obstacles_list[i]