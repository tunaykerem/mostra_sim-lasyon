import numpy as np
import Circle


def northsouthturn(x_main,y_main,waypointIndex,corner_value,path,data,directionChangeControl,next_direction,directionList,direction):
    x_raidus=0
    y_radius=0
    center_x=0
    center_y=0

    end_up_point =0
    end_down_point = 0
    end_right_point = 0
    end_left_point = 0

    start_up_point = 0
    start_down_point = 0
    start_right_point = 0
    start_left_point = 0
    try:
        value = waypointIndex[corner_value]
        #Köseden önceki son noktanın koordinatı
        x_end = path[value-1][0]
        y_end = path[value-1][1]
        #Köseden sonraki ilk noktanın koordinatı
        x_start = path[value+1][0]
        y_start = path[value+1][1]
        corner_value += 1
        for row in data:
            x_end_value = int(row[0])
            y_end_value = int(row[1])
            if x_end==x_end_value and y_end== y_end_value:
                # print(row)
                end_down_point = float(row[2])
                end_up_point = float(row[3])
                end_right_point = float(row[4])
                end_left_point = float(row[5])
        for row in data:
            x_start_value = int(row[0])
            y_start_value = int(row[1])
            if x_start==x_start_value and y_start== y_start_value:
                # print(row)
                start_down_point = float(row[2])
                start_up_point = float(row[3])
                start_right_point = float(row[4])
                start_left_point = float(row[5])
    except IndexError:
        print("Geçersiz dizin!")

    directionChangeControl +=1
    next_direction = directionList[directionChangeControl]
    if direction == "B":    
        if next_direction == "K":
            #radiusları yönlere göre ayarlamak lazım
            radius_x= abs(end_left_point-start_right_point)
            # print(x_raidus)
            radius_y = abs(end_down_point-start_down_point)
            center_x= end_left_point
            center_y= start_down_point
            starting_angle= 3*np.pi/2
            ending_angle = np.pi
            #def drawingCircle(radius_x,radius_y,center_x,center_y,starting_angle,ending_angle):
            x_main, y_main = Circle.drawingCircle(radius_x,radius_y,center_x,center_y,starting_angle,ending_angle,x_main,y_main)
        elif next_direction == "G":

            radius_x= abs(end_left_point-start_left_point)
            # print(x_raidus)
            radius_y = abs(end_down_point-start_up_point)

            center_x= end_left_point
            center_y= start_up_point
            starting_angle= np.pi/2
            ending_angle = np.pi
            #def drawingCircle(radius_x,radius_y,center_x,center_y,starting_angle,ending_angle):
            x_main, y_main = Circle.drawingCircle(radius_x,radius_y,center_x,center_y,starting_angle,ending_angle,x_main,y_main)  
    elif direction=="D":
        if next_direction == "K":
            #radiusları yönlere göre ayarlamak lazım
            radius_x= abs(end_right_point-start_right_point)
            # print(x_raidus)
            radius_y = abs(end_up_point-start_down_point)
            center_x= end_right_point
            center_y= start_down_point
            starting_angle= 3*np.pi/2
            ending_angle = 2*np.pi
            #def drawingCircle(radius_x,radius_y,center_x,center_y,starting_angle,ending_angle):
            x_main, y_main = Circle.drawingCircle(radius_x,radius_y,center_x,center_y,starting_angle,ending_angle,x_main,y_main)
        elif next_direction == "G":

            radius_x= abs(end_right_point-start_left_point)
            # print(x_raidus)
            radius_y = abs(end_up_point-start_up_point)

            center_x= end_right_point
            center_y= start_up_point
            starting_angle= np.pi/2
            ending_angle = 0
            #def drawingCircle(radius_x,radius_y,center_x,center_y,starting_angle,ending_angle):
            x_main, y_main = Circle.drawingCircle(radius_x,radius_y,center_x,center_y,starting_angle,ending_angle,x_main,y_main)  
    

    return x_main,y_main,corner_value,directionChangeControl