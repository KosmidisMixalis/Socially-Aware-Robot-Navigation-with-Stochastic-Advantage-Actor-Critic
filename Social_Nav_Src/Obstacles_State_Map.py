import numpy as np

class ObstaclesStateMap:

    # Constants
    MAP_GRID_SIZE = None  # grid size 3x3
    MAP_CELL_SIZE = None  # cell dimension 1x1

    def __init__(self, grid_size, cell_size):

        self.MAP_GRID_SIZE = grid_size
        self.MAP_CELL_SIZE = cell_size

    def is_inside_cell(self, grid_center_x, grid_center_y , point, cell_size=1.0):

        x_point = point[0]
        y_point = point[1]

        half_size = cell_size / 2
        return (
            (grid_center_x - half_size <= x_point <= grid_center_x + half_size) and
            (grid_center_y - half_size <= y_point <= grid_center_y + half_size)
        )

    def get_grid_center_position(self, index):
            
            center_x = self.MAP_CELL_SIZE / 2.0 + self.MAP_CELL_SIZE * index[0] - self.MAP_CELL_SIZE*(self.MAP_GRID_SIZE / 2.0)
            center_y = self.MAP_CELL_SIZE * (self.MAP_GRID_SIZE - index[1] - 1) + self.MAP_CELL_SIZE / 2.0

            if (center_x == 0):
                 
                return (center_y, center_x)
            
            return (center_y, -center_x)
    

    def obstacle_distance_grid(self, lidar_data):
        
        grid = np.zeros(self.MAP_GRID_SIZE * self.MAP_GRID_SIZE)    

        if(lidar_data != []):
            index = 0
            
            for x in range(3):
                    for y in range(3):
                        grid_center_x, grid_center_y = self.get_grid_center_position([x , y])
                     
                        for point in lidar_data:
                            if self.is_inside_cell(grid_center_x, grid_center_y, point):
                                
                                grid[index] = 1
                        
                        index += 1
        return grid
    
