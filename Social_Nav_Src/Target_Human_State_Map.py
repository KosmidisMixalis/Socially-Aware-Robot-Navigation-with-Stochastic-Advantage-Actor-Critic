import numpy as np
from Reset_Env import get_model_pose
import math
import tf.transformations as tft


class TargetHumanStateMap:

    # Constants
    MAP_GRID_SIZE = None  # grid size 3x3
    MAP_CELL_SIZE = None  # cell dimension 1x1
     
  
    def __init__(self, grid_size, cell_size):

        self.MAP_GRID_SIZE = grid_size
        self.MAP_CELL_SIZE = cell_size

    
    # Function to calculate Euclidean distance with numpy
    def euclidean_distance(self, point1, point2):

        point1 = np.array(point1)
        point2 = np.array(point2)

        return np.linalg.norm(point1 - point2)


    def get_grid_center_position(self, index):
            
            center_x = self.MAP_CELL_SIZE / 2.0 + self.MAP_CELL_SIZE * index[0] - self.MAP_CELL_SIZE*(self.MAP_GRID_SIZE / 2.0)
            center_y = self.MAP_CELL_SIZE * (self.MAP_GRID_SIZE - index[1] - 1) + self.MAP_CELL_SIZE / 2.0
            
            if (center_x == 0):
                 
                return (center_y, center_x)
            
            return (center_y, -center_x)

    def transformation(self, robot_pose, cell_x, cell_y):

        x_r = robot_pose.position.x
        y_r = robot_pose.position.y
        q = robot_pose.orientation
        _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])

        x_world = x_r + (cell_x * math.cos(yaw) - cell_y * math.sin(yaw))
        y_world = y_r + (cell_x * math.sin(yaw) + cell_y * math.cos(yaw))
        return x_world, y_world


    def distance_to_goal_grid(self, point):

        if point != []:
            Human_x = point[0]
            Human_y = point[1]
            Human_z = point[2]  # irrelevant for distance calculation
            r_pose = get_model_pose("ridgeback")

            human_point = (Human_x, Human_y)

            index = 0
            grid = np.zeros(self.MAP_GRID_SIZE * self.MAP_GRID_SIZE)  
            for x in range(3):
                    for y in range(3):
                       
                        cell_x, cell_y = self.get_grid_center_position([x , y])
                        
                        grid_center_x, grid_center_y = self.transformation(r_pose, cell_x, cell_y) 
                        distance = self.euclidean_distance([grid_center_x, grid_center_y], human_point)
                        
                        grid[index] = distance
                        index += 1
            return grid
        else:
            grid = np.zeros(self.MAP_GRID_SIZE * self.MAP_GRID_SIZE)  

            return grid  

