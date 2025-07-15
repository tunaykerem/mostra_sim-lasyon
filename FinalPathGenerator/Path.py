import heapq
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.colors import ListedColormap
from FinalPathGenerator import Circle
import csv
#import PathPlanning
from FinalPathGenerator import StraightLine
from FinalPathGenerator import RightLeftTurn
from FinalPathGenerator import NorthSouthTurn
from FinalPathGenerator import BatidanKavsak
from FinalPathGenerator import KuzeydenKavsak
from FinalPathGenerator import FindCarIndex
from FinalPathGenerator import FirstDurak
from FinalPathGenerator.astarAlgorithm import Astar
from FinalPathGenerator.util import FindDirectionChange, FindWaypointIndex,DirectionList,Global

data = Global.csv_to_list()

def getTrajectory(x_main_current,y_main_current, inital_x,initial_y,target_index_x,target_index_y):
    # Matrixi Global.py den al
    matrix = Global.matrix

    #Initiliaze the variable
    car_initial_x = inital_x
    car_initial_y = initial_y
    car_initial_index_x,car_initial_index_y= FindCarIndex.findcarindex(car_initial_x, car_initial_y, data)
    start_point = (car_initial_index_x, car_initial_index_y)
    end_point = (target_index_x, target_index_y)

    #A* Searching algrithm ile path i bul
    shortest_path_cost, path = Astar.astar(matrix, start_point, end_point)
    # Yön değişimlerinin oldugu konumları matrix üzerinde bul  [(7, 8), (11, 8), (11, 6)]  bu 3 nokta yön değişimlerinin oldugu köşe noktalar
    direction_changes = FindDirectionChange.find_direction_changes(path)
    # Shortest Pathı bir listeye atmıştık. Aşağıdaki işlem ile köşe noktalarının o listetdeki hangi index oldugunu buluyoruz. waypoint index: [0, 2, 9] ise 0 başlanıç , 2 ilk köşe 9 2. köşe noktasıdır
    waypointIndex = FindWaypointIndex.findWaypointIndex(direction_changes, path)
    # Direction Changes te buldugum köşe noktaları nda hangi yöne döndüklerini buluyoruz .Direction List:  ['K', 'B']  bu değer , Araçta başlangıçta Kuzey sonra ilk köşe noktasından Batıya dönüyor
    directionList = DirectionList.directionList(path,waypointIndex)
    #Initialize corner value of each point on matrix
    up_point,down_point,right_point,left_point = 0,0,0,0
    x_main,y_main = [car_initial_x], [car_initial_y]
    start_control = True
    end_control = True
    #Toplam kaç defa yön değiştirdiğimizi buluyoruz
    directin_change_number= len(directionList)
    #Result list [(0, 11), (11, 18)] şeklinde bir dğer vermektedir. Rotayı oluştururken for döngüsünün range ini bu değerler ile kontrol ediyoruz
    result_list = [(waypointIndex[i], waypointIndex[i + 1]) for i in range(len(waypointIndex) - 1)]
    result_list.append((waypointIndex[-1], len(path)))
    resultListLen= len(result_list)
    resultListControl=-1
    controlIndexNumber=-1
    directionChangeControl=0
    next_direction = directionList[directionChangeControl]
    corner_value=1
    if shortest_path_cost != float('inf'):
        # for i in range( 0,len(path)):
        #     matrix[path[i][0], path[i][1]] = 2
        for ind in directionList:
            controlIndexNumber +=1
            resultListControl+=1
            if resultListControl < resultListLen:
                if ind == "K":
                    if end_control:
                        if start_control :
                            start_control = False
                            for i in range(result_list[resultListControl][0],result_list[resultListControl][1]):
                                if i== waypointIndex[-1]-1:
                                    end_control = False
                                x1 = path[i][0]
                                y1 = path[i][1]
                                # print(controlIndexNumber)
                                # print(len(result_list))
                                if len(result_list)==1:
                                    for row in data:
                                        x2 = float(row[0])
                                        y2 = float(row[1])
                                        if x1==x2 and y1== y2:
                                            up_point = float(row[3])
                                            down_point = float(row[2])
                                            right_point = float(row[4])
                                            left_point = float(row[5])
                                            x_main,y_main = StraightLine.KuzeyGuneyHareket(x_main[-1],right_point,y_main[-1],up_point,x_main,y_main)
                                else:
                                    if i != waypointIndex[controlIndexNumber+1] :
                                        for row in data:
                                            x2 = float(row[0])
                                            y2 = float(row[1])
                                            if x1==x2 and y1== y2:
                                                up_point = float(row[3])
                                                down_point = float(row[2])
                                                right_point = float(row[4])
                                                left_point = float(row[5])
                                                x_main,y_main = StraightLine.KuzeyGuneyHareket(x_main[-1],right_point,y_main[-1],up_point,x_main,y_main)

                                    if i == waypointIndex[controlIndexNumber + 1] - 1:

                                        x_main,y_main,corner_value,directionChangeControl= RightLeftTurn.rightleftturn(x_main,y_main,waypointIndex, corner_value, path, data, directionChangeControl, next_direction, directionList,ind)
                        else:
                            for i in range(result_list[resultListControl][0]+1,result_list[resultListControl][1]):
                                if i== waypointIndex[-1]-1:
                                    end_control = False
                                x1 = path[i][0]
                                y1 = path[i][1]
                                if i != waypointIndex[controlIndexNumber+1] :
                                    for row in data:
                                        x2 = float(row[0])
                                        y2 = float(row[1])
                                        if x1==x2 and y1== y2:
                                            up_point = float(row[3])
                                            down_point = float(row[2])
                                            right_point = float(row[4])
                                            left_point = float(row[5])
                                            x_main,y_main = StraightLine.KuzeyGuneyHareket(x_main[-1],right_point,y_main[-1],up_point,x_main,y_main)
                                if i == waypointIndex[controlIndexNumber + 1] - 1:
                                    x_main,y_main,corner_value,directionChangeControl= RightLeftTurn.rightleftturn(x_main,y_main,waypointIndex, corner_value, path, data, directionChangeControl, next_direction, directionList,ind)
                    else:
                        for i in range(result_list[resultListControl][0],result_list[resultListControl][1]):
                            x1 = path[i][0]
                            y1 = path[i][1]
                            if i != waypointIndex[controlIndexNumber] :
                                for row in data:
                                    x2 = float(row[0])
                                    y2 = float(row[1])
                                    if x1==x2 and y1== y2:
                                        up_point = float(row[3])
                                        down_point = float(row[2])
                                        right_point = float(row[4])
                                        left_point = float(row[5])
                                        x_main,y_main = StraightLine.KuzeyGuneyHareket(x_main[-1],right_point,y_main[-1],up_point,x_main,y_main)
                elif ind == "G":
                    if end_control:
                        if start_control :
                            start_control = False
                            for i in range(result_list[resultListControl][0],result_list[resultListControl][1]):
                                if i== waypointIndex[-1]-1:
                                    end_control = False
                                x1 = path[i][0]
                                y1 = path[i][1]
                                if len(result_list)==1:
                                    for row in data:
                                        x2 = float(row[0])
                                        y2 = float(row[1])
                                        if x1==x2 and y1== y2:
                                            up_point = float(row[3])
                                            down_point = float(row[2])
                                            right_point = float(row[4])
                                            left_point = float(row[5])
                                            x_main,y_main = StraightLine.KuzeyGuneyHareket(x_main[-1],left_point,y_main[-1],down_point,x_main,y_main)
                                else:
                                    if i != waypointIndex[controlIndexNumber+1] :
                                        for row in data:
                                            x2 = float(row[0])
                                            y2 = float(row[1])
                                            if x1==x2 and y1== y2:
                                                up_point = float(row[3])
                                                down_point = float(row[2])
                                                right_point = float(row[4])
                                                left_point = float(row[5])
                                                x_main,y_main = StraightLine.KuzeyGuneyHareket(x_main[-1],left_point,y_main[-1],down_point,x_main,y_main)

                                    if i == waypointIndex[controlIndexNumber + 1] - 1:

                                        x_main,y_main,corner_value,directionChangeControl= RightLeftTurn.rightleftturn(x_main,y_main,waypointIndex, corner_value, path, data, directionChangeControl, next_direction, directionList,ind)
                        else:
                            for i in range(result_list[resultListControl][0]+1,result_list[resultListControl][1]):
                                if i== waypointIndex[-1]-1:
                                    end_control = False
                                x1 = path[i][0]
                                y1 = path[i][1]
                                if i != waypointIndex[controlIndexNumber+1] :
                                    for row in data:
                                        x2 = float(row[0])
                                        y2 = float(row[1])
                                        if x1==x2 and y1== y2:
                                            up_point = float(row[3])
                                            down_point = float(row[2])
                                            right_point = float(row[4])
                                            left_point = float(row[5])
                                            x_main,y_main = StraightLine.KuzeyGuneyHareket(x_main[-1],left_point,y_main[-1],down_point,x_main,y_main)
                                            
                                if i == waypointIndex[controlIndexNumber + 1] - 1:

                                    x_main,y_main,corner_value,directionChangeControl= RightLeftTurn.rightleftturn(x_main,y_main,waypointIndex, corner_value, path, data, directionChangeControl, next_direction, directionList,ind)
                    else:
                        for i in range(result_list[resultListControl][0],result_list[resultListControl][1]):
                            x1 = path[i][0]
                            y1 = path[i][1]
                            if x1==5 and  y1==3:
                                x_main,y_main = KuzeydenKavsak.Kavsak(x_main, y_main)
                                break
                            if i != waypointIndex[controlIndexNumber] :
                                for row in data:
                                    x2 = float(row[0])
                                    y2 = float(row[1])
                                    if x1==x2 and y1== y2:
                                        up_point = float(row[3])
                                        down_point = float(row[2])
                                        right_point = float(row[4])
                                        left_point = float(row[5])
                                        x_main,y_main = StraightLine.KuzeyGuneyHareket(x_main[-1],left_point,y_main[-1],down_point,x_main,y_main)
                elif ind == "B":
                    if end_control:
                        if start_control:
                            start_control = False
                            for i in range(result_list[resultListControl][0],result_list[resultListControl][1]):
                                x1 = path[i][0]
                                y1 = path[i][1]
                                if i== waypointIndex[-1]-1:
                                    end_control = False
                                    end_control = False
                                if x1==0 and  y1==5 and Global.mission[0][0]==0 and Global.mission[0][1]==5:
                                    x_main,y_main = FirstDurak.firstDurak(x_main, y_main)
                                    break
                                if len(result_list)==1:
                                    for row in data:
                                        x2 = float(row[0])
                                        y2 = float(row[1])
                                        if x1==x2 and y1== y2:
                                            up_point = float(row[3])
                                            down_point = float(row[2])
                                            right_point = float(row[4])
                                            left_point = float(row[5])
                                            x_main,y_main = StraightLine.DoguBatiHareket(x_main[-1], left_point, y_main[-1], up_point, x_main, y_main)
                                else:
                                    if i != waypointIndex[controlIndexNumber+1] :
                                        for row in data:
                                            x2 = float(row[0])
                                            y2 = float(row[1])
                                            if x1==x2 and y1== y2:
                                                up_point = float(row[3])
                                                down_point = float(row[2])
                                                right_point = float(row[4])
                                                left_point = float(row[5])
                                                x_main,y_main = StraightLine.DoguBatiHareket(x_main[-1], left_point, y_main[-1], up_point, x_main, y_main)
                                    if i == waypointIndex[controlIndexNumber + 1] - 1:
                                        x_main,y_main,corner_value,directionChangeControl= NorthSouthTurn.northsouthturn(x_main, y_main, waypointIndex, corner_value, path, data, directionChangeControl, next_direction, directionList,ind)
                            
                        else:
                            for i in range(result_list[resultListControl][0]+1,result_list[resultListControl][1]):
                                x1 = path[i][0]
                                y1 = path[i][1]
                                if i== waypointIndex[-1]-1:
                                    end_control = False
                                if i != waypointIndex[controlIndexNumber+1] :
                                    for row in data:
                                        x2 = float(row[0])
                                        y2 = float(row[1])
                                        if x1==x2 and y1== y2:
                                            up_point = float(row[3])
                                            down_point = float(row[2])
                                            right_point = float(row[4])
                                            left_point = float(row[5])
                                            x_main,y_main = StraightLine.DoguBatiHareket(x_main[-1], left_point, y_main[-1], up_point, x_main, y_main)
                                if i == waypointIndex[controlIndexNumber + 1] - 1:
                                    x_main,y_main,corner_value,directionChangeControl= NorthSouthTurn.northsouthturn(x_main, y_main, waypointIndex, corner_value, path, data, directionChangeControl, next_direction, directionList,ind)
                    else:
                        for i in range(result_list[resultListControl][0],result_list[resultListControl][1]):
                            x1 = path[i][0]
                            y1 = path[i][1]

                            if i != waypointIndex[controlIndexNumber] :
                                for row in data:
                                    x2 = float(row[0])
                                    y2 = float(row[1])
                                    if x1==x2 and y1== y2:
                                        up_point = float(row[3])
                                        down_point = float(row[2])
                                        right_point = float(row[4])
                                        left_point = float(row[5])
                                        x_main,y_main = StraightLine.DoguBatiHareket(x_main[-1], left_point, y_main[-1], up_point, x_main, y_main)
                            if x1==0 and  y1==5 and Global.mission[0][0]==0 and Global.mission[0][1]==5:
                                x_main,y_main = FirstDurak.firstDurak(x_main, y_main)
                                break
                elif ind == "D":
                    if end_control:
                        if start_control:
                            start_control = False
                            for i in range(result_list[resultListControl][0],result_list[resultListControl][1]):
                                x1 = path[i][0]
                                y1 = path[i][1]
                                if i== waypointIndex[-1]-1:
                                    end_control = False
                                if len(result_list)==1:
                                    for row in data:
                                        x2 = float(row[0])
                                        y2 = float(row[1])
                                        if x1==x2 and y1== y2:
                                            up_point = float(row[3])
                                            down_point = float(row[2])
                                            right_point = float(row[4])
                                            left_point = float(row[5])
                                            x_main,y_main = StraightLine.DoguBatiHareket(x_main[-1], right_point, y_main[-1], down_point, x_main, y_main)
                                else:
                                    if i != waypointIndex[controlIndexNumber+1] :
                                        for row in data:
                                            x2 = float(row[0])
                                            y2 = float(row[1])
                                            if x1==x2 and y1== y2:
                                                up_point = float(row[3])
                                                down_point = float(row[2])
                                                right_point = float(row[4])
                                                left_point = float(row[5])
                                                x_main,y_main = StraightLine.DoguBatiHareket(x_main[-1], right_point, y_main[-1], down_point, x_main, y_main)
                                    if i == waypointIndex[controlIndexNumber + 1] - 1:
                                        x_main,y_main,corner_value,directionChangeControl= NorthSouthTurn.northsouthturn(x_main, y_main, waypointIndex, corner_value, path, data, directionChangeControl, next_direction, directionList,ind)
                            
                        else:
                            for i in range(result_list[resultListControl][0]+1,result_list[resultListControl][1]):
                                x1 = path[i][0]
                                y1 = path[i][1]

                                if i== waypointIndex[-1]-1:
                                    end_control = False
                                if i != waypointIndex[controlIndexNumber+1] :
                                    for row in data:
                                        x2 = float(row[0])
                                        y2 = float(row[1])
                                        if x1==x2 and y1== y2:
                                            up_point = float(row[3])
                                            down_point = float(row[2])
                                            right_point = float(row[4])
                                            left_point = float(row[5])
                                            x_main,y_main = StraightLine.DoguBatiHareket(x_main[-1], right_point, y_main[-1], down_point, x_main, y_main)
                                if i == waypointIndex[controlIndexNumber + 1] - 1:
                                    x_main,y_main,corner_value,directionChangeControl= NorthSouthTurn.northsouthturn(x_main, y_main, waypointIndex, corner_value, path, data, directionChangeControl, next_direction, directionList,ind)
                    else:
                        for i in range(result_list[resultListControl][0],result_list[resultListControl][1]):
                            x1 = path[i][0]
                            y1 = path[i][1]
                            if x1==7 and  y1==1:
                                x_main,y_main = BatidanKavsak.Kavsak(x_main, y_main)
                                break
                            if i != waypointIndex[controlIndexNumber] :
                                for row in data:
                                    x2 = float(row[0])
                                    y2 = float(row[1])
                                    if x1==x2 and y1== y2:
                                        up_point = float(row[3])
                                        down_point = float(row[2])
                                        right_point = float(row[4])
                                        left_point = float(row[5])
                                        x_main,y_main = StraightLine.DoguBatiHareket(x_main[-1], right_point, y_main[-1], down_point, x_main, y_main)

    
    return x_main_current+x_main,y_main_current+y_main
  





