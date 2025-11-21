import numpy as np

class SocialStateMap:

    # Constants
    MAP_GRID_SIZE = None  # grid size 3x3
    MAP_CELL_SIZE = None  # cell dimension 1x1
    PEDESTRIAN_COMFORT_SPACE = None
    ROBOT_RADIUS = None

 
    def __init__(self, grid_size, cell_size, pedestrian_social_space, r_radius):

        self.MAP_GRID_SIZE = grid_size
        self.MAP_CELL_SIZE = cell_size
        self.PEDESTRIAN_COMFORT_SPACE = pedestrian_social_space
        self.ROBOT_RADIUS = r_radius

    
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
    

    def social_distace_grid(self, humans_detect_poses):

        grid = np.zeros(self.MAP_GRID_SIZE * self.MAP_GRID_SIZE)    
 
        if humans_detect_poses != []:
            index = 0
            for x in range(3):
                    for y in range(3):
                        for cor in humans_detect_poses:
                            grid_center_x, grid_center_y = self.get_grid_center_position([x , y])

                            distance = self.euclidean_distance([grid_center_x, grid_center_y], [cor[0],cor[1]])
                         
                            if distance <= self.PEDESTRIAN_COMFORT_SPACE + self.ROBOT_RADIUS: 

                                grid[index] = 1

                        index += 1
                            
        return grid
    