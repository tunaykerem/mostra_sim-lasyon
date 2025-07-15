from .util import Global as g
from Util import Global as Global

def dur(carIndex,direction):
    if direction == "K":
            for i in g.corner_point:
                if int(carIndex[0]) - 1 == i[0] and carIndex[1] == i[1]:
                    found = False
                    for row in g.csv_to_list():
                        if row[0] == str(i[0]+1) and row[1] == str(i[1]):
                            stop_point = [float(row[4]), float(row[3])]
                            if stop_point not in Global.stop_points:
                                Global.stop_points.insert(0, stop_point)
                                print(Global.stop_points)
                                found = True
                                break
                    if found:
                        break

                       
    elif direction=="G":
        for i in g.corner_point:
            if int(carIndex[0])+1 == i[0] and carIndex[1] == i[1]:
                found = False
                for row in g.csv_to_list():
                    if row[0] == str(i[0]) and row[1] == str(i[1]):
                        stop_point = [float(row[5]), float(row[2])]
                        if stop_point not in Global.stop_points:
                                Global.stop_points.insert(0, stop_point)
                                print(Global.stop_points)
                                found = True
                                break
                    if found:
                        break  
                                    
    elif direction=="D":
        for i in g.corner_point:
            if int(carIndex[1])+1==i[1] and carIndex[0] == i[0]:
                found = False
                for row in g.csv_to_list():
                    if row[0] == str(i[0]) and row[1] == str(i[1]):
                        stop_point = [float(row[4]), float(row[2])]
                        if stop_point not in Global.stop_points:
                            Global.stop_points.insert(0, stop_point)
                            print(Global.stop_points)
                            found = True
                            break
                    if found:
                        break 
                                    
    elif direction=="B":
        for i in g.corner_point:
            if int(carIndex[1])-1 == i[1] and carIndex[0] == i[0]:
                found=False
                for row in g.csv_to_list():
                    if row[0] == str(i[0]) and row[1] == str(i[1]): 
                        stop_point = [float(row[5]), float(row[3])]
                        if stop_point not in Global.stop_points:
                            Global.stop_points.insert(0, stop_point)
                            print(Global.stop_points)
                            found = True
                            break
                    if found:
                        break
     