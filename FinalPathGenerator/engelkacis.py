from Util import Global as gl
import FinalPathGenerator.Path as Path
from matplotlib import pyplot as plt
from matplotlib.colors import ListedColormap
import FinalPathGenerator.StraightLine as sl
import csv
import FinalPathGenerator.FindCarIndex as FindCarIndex
from FinalPathGenerator.astarAlgorithm import Astar
import FinalPathGenerator.RigthLeftTurnsforObstacle as RigthLeftTurnsforObstacle
import FinalPathGenerator.NorthSouthTurnforObstacle as NorthSouthTurnforObstacle
from Util import Global as gl
from FinalPathGenerator.util import FindDirectionChange, FindWaypointIndex, DirectionList
import numpy as np


class escapeFromObstacle():
    def __init__(self, initial_x, initial_y, direction):
        # Load data from CSV
        try:
            self.data = gl.csv_to_list()
        except Exception as e:
            print(f"Warning: Failed to load CSV data: {e}")
            self.data = []  # Use empty list as fallback
            
        self.initial_x = initial_x
        self.initial_y = initial_y
        self.direction = direction
        
        # Safely get matrix from Global
        try:
            self.matrix = gl.matrix
            if self.matrix is None or len(self.matrix) == 0:
                print("Warning: Matrix in Global.py is empty, creating default matrix")
                self.matrix = np.zeros((20, 20), dtype=int)
                gl.matrix = self.matrix  # Update the global matrix
        except AttributeError:
            print("Warning: matrix not found in Global.py, creating default matrix")
            self.matrix = np.zeros((20, 20), dtype=int)
            gl.matrix = self.matrix  # Set the global matrix
            
    def createNewTrajectory(self):
        matrix = self.matrix
        initial_x = self.initial_x
        initial_y = self.initial_y
        direction = self.direction
        distance = 5
        
        # Check if matrix is defined, if not create a simple one
        if matrix is None or len(matrix) == 0:
            print("Warning: Matrix not properly initialized, creating a default matrix")
            matrix = np.zeros((20, 20), dtype=int)
            self.matrix = matrix
            gl.matrix = matrix  # Update global matrix
            
        # Ensure mission is defined
        if not hasattr(gl, 'mission') or gl.mission is None or len(gl.mission) == 0:
            print("Warning: Mission not defined, using default target")
            gl.mission = [[10, 10]]  # Default target
        car_initial_index_x,car_initial_index_y= FindCarIndex.findcarindex(self.initial_x, self.initial_y, self.data)
        start_point = (car_initial_index_x, car_initial_index_y)
        end_point = (gl.mission[0][0], gl.mission[0][1])
        shortest_path_cost, path = Astar.astar(matrix, start_point, end_point)
        direction_changes = FindDirectionChange.find_direction_changes(path)
        waypointIndex = FindWaypointIndex.findWaypointIndex(direction_changes, path)
        directionList = DirectionList.directionList(path,waypointIndex)
        if len(waypointIndex)>1:
            x1= path[waypointIndex[1]-1][0]
            y1= path[waypointIndex[1]-1][1]
        else:
            x1= path[-1][0]
            y1= path[-1][1]
        for row in self.data:
            x2 = float(row[0])
            y2 = float(row[1])
            if x1==x2 and y1== y2:
                up_point = float(row[3])
                down_point = float(row[2])
                right_point = float(row[4])
                left_point = float(row[5])

        directin_change_number= len(directionList)
        result_list = [(waypointIndex[i], waypointIndex[i + 1]) for i in range(len(waypointIndex) - 1)]
        result_list.append((waypointIndex[-1], len(path)))
        resultListLen= len(result_list)
        resultListControl=-1
        controlIndexNumber=-1
        directionChangeControl=0
        next_direction = directionList[directionChangeControl]
        corner_value=1
        if direction == "K":
            # initial_x, initial_y= 0,22
            x_main=[initial_x]
            y_main= [initial_y]
            if (initial_y+distance*2)<=up_point:
                x_main,y_main = sl.KuzeyGuneyHareket(initial_x, initial_x-3, initial_y, initial_y+distance, x_main, y_main)
                x_main,y_main = sl.KuzeyGuneyHareket(x_main[-1], x_main[-1]+3, y_main[-1],  y_main[-1]+distance, x_main, y_main)
            else:
                x_main,y_main = sl.KuzeyGuneyHareket(initial_x, initial_x-3, initial_y, initial_y+distance, x_main, y_main)
                x_main,y_main = sl.KuzeyGuneyHareket(x_main[-1], x_main[-1], y_main[-1], up_point, x_main, y_main)
                x_main,y_main,corner_value,directionChangeControl= RigthLeftTurnsforObstacle.rightleftturn(x_main,y_main,waypointIndex, corner_value, path, self.data, directionChangeControl, next_direction, directionList,direction)
        elif direction == "G":
            # initial_x, initial_y= -63,54
            x_main=[initial_x]
            y_main= [initial_y]
            if (initial_y-distance*2)>=down_point:
                x_main,y_main = sl.KuzeyGuneyHareket(initial_x, initial_x+3, initial_y, initial_y-distance, x_main, y_main)
                x_main,y_main = sl.KuzeyGuneyHareket(x_main[-1], x_main[-1]-3, y_main[-1], y_main[-1]-distance, x_main, y_main)
            else:
                x_main,y_main = sl.KuzeyGuneyHareket(initial_x, initial_x+3, initial_y, initial_y-distance, x_main, y_main)
                x_main,y_main = sl.KuzeyGuneyHareket(x_main[-1], x_main[-1], y_main[-1], down_point, x_main, y_main)
                x_main,y_main,corner_value,directionChangeControl= RigthLeftTurnsforObstacle.rightleftturn(x_main,y_main,waypointIndex, corner_value, path, self.data, directionChangeControl, next_direction, directionList,direction)
        elif direction == "B":
            # initial_x, initial_y= -6,61
            x_main=[initial_x]
            y_main= [initial_y]
            if (initial_x-distance*2)>=left_point:
                x_main,y_main = sl.DoguBatiHareket(initial_x, initial_x-distance, initial_y, initial_y-3, x_main, y_main)
                x_main,y_main = sl.DoguBatiHareket(x_main[-1], x_main[-1]-distance, y_main[-1], y_main[-1]+3, x_main, y_main)
            else:
                x_main,y_main = sl.DoguBatiHareket(initial_x, initial_x-distance, initial_y, initial_y-3, x_main, y_main)
                x_main,y_main = sl.DoguBatiHareket(x_main[-1], left_point, y_main[-1], y_main[-1], x_main, y_main)
                x_main,y_main,corner_value,directionChangeControl= NorthSouthTurnforObstacle.northsouthturn(x_main, y_main, waypointIndex, corner_value, path, self.data, directionChangeControl, next_direction, directionList,direction)


        elif direction == "D":
            # initial_x, initial_y= -55,58
            x_main=[initial_x]
            y_main= [initial_y]
            if (initial_x+distance*2)<=left_point:
                x_main,y_main = sl.DoguBatiHareket(initial_x, initial_x+distance, initial_y, initial_y+3, x_main, y_main)
                x_main,y_main = sl.DoguBatiHareket(x_main[-1], x_main[-1]+distance, y_main[-1], y_main[-1]-3, x_main, y_main)
            else:
                x_main,y_main = sl.DoguBatiHareket(initial_x, initial_x+distance, initial_y, initial_y+3, x_main, y_main)
                x_main,y_main = sl.DoguBatiHareket(x_main[-1], right_point, y_main[-1], y_main[-1], x_main, y_main)
                x_main,y_main,corner_value,directionChangeControl= NorthSouthTurnforObstacle.northsouthturn(x_main, y_main, waypointIndex, corner_value, path, self.data, directionChangeControl, next_direction, directionList,direction)

        
        return x_main,y_main
    








# escape_obj = escapeFromObstacle( 0, 45.36, "K")


# x_main,y_main= escape_obj.createNewTrajectory()

# # Plot the data
# plt.plot(x_main, y_main, marker='o', linestyle='-')

# # Set plot title and axis labels
# plt.title('Örnek Grafiği')
# plt.xlabel('X Ekseni')
# plt.ylabel('Y Ekseni')
# # Eksen sınırlarını manuel olarak belirleme
# plt.xlim(-70, 10)
# plt.ylim(-10, 70)
# # Display the grid
# plt.grid(True)

# # Show the plot
# plt.show()




#eğer kullanıcı bir engel girmesi durumunda bu foknsiyon çalışıp ter