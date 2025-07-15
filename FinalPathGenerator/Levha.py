from .util import Global as g
from FinalPathGenerator.FindCarIndex import findcarindex
from FinalPathGenerator.PathGenerator import get_firstTrajectory
from matplotlib import pyplot as plt
from Util import Global
import numpy as np

currentx=0
currenty=0
def levha(levhAdi, direction, current_x, current_y):
    # Convert numpy array to list for comparison if needed
    if isinstance(levhAdi, np.ndarray):
        levhAdi = levhAdi.tolist()
    
    # Calculate car index from current position
    data = g.csv_to_list()  # Get the CSV data
    carIndex = findcarindex(current_x, current_y, data)
    if carIndex is None:
        print(f"Could not find car index for position ({current_x}, {current_y})")
        return Global.x_main, Global.y_main
    
    # Check for girilmez yol sign (no entry) - handle both string and list formats
    if levhAdi == 'giris_olmayan_yol' or levhAdi == ['giris_olmayan_yol']:
        print(f"GIRILMEZ YOL DETECTED! Car index: {carIndex}, Direction: {direction}")
        
        # IMMEDIATE STOP - Critical safety action
        Global.breaking = 1  # Apply brakes immediately
        Global.v_desired = 0.0  # Set desired speed to zero
        Global.state = "dur"  # Set state to stop
        print("EMERGENCY STOP ACTIVATED!")

        if direction=="K":
            for i in g.corner_point:
                if carIndex[0]-1==i[0] and carIndex[1]==i[1]:
                    print("car index:",carIndex)
                    print("corner: ",i)
                    g.matrix[i[0]-1,i[1]]=0
                    print("Matrix updated:", g.matrix)
                    # Update global vehicle location and recalculate trajectory
                    g.vehicle_location=[current_x,current_y]
                    g.x_main,g.y_main=get_firstTrajectory()
                    Global.x_main = g.x_main
                    Global.y_main = g.y_main
                    print("New trajectory calculated!")

        elif direction=="G":
            for i in g.corner_point:
                if carIndex[0]+1==i[0] and carIndex[1]==i[1]:
                    g.matrix[i[0]+1,i[1]]=0
                    g.vehicle_location=[current_x,current_y]
                    g.x_main,g.y_main=get_firstTrajectory()
                    Global.x_main = g.x_main
                    Global.y_main = g.y_main
                    print("New trajectory calculated!")

        elif direction=="D":
            for i in g.corner_point:
                if carIndex[1]+1==i[1] and carIndex[0]==i[0]:
                    g.matrix[i[0],i[1]+1]=0
                    g.vehicle_location=[current_x,current_y]
                    g.x_main,g.y_main=get_firstTrajectory()
                    Global.x_main = g.x_main
                    Global.y_main = g.y_main
                    print("New trajectory calculated!")

        elif direction=="B":
            for i in g.corner_point:
                if carIndex[1]-1==i[1] and carIndex[0]==i[0]:
                    g.matrix[i[0],i[1]-1]=0
                    g.vehicle_location=[current_x,current_y]
                    g.x_main,g.y_main=get_firstTrajectory()
                    Global.x_main = g.x_main
                    Global.y_main = g.y_main
                    print("New trajectory calculated!")




    elif levhAdi == "solaDönlmez" or levhAdi == "sola_donulmez":
        if direction=="K":
            for i in g.corner_point:
                if carIndex[0]-1==i[0] and carIndex[1]==i[1]:
                    g.matrix[i[0],i[1]-1]=0

        elif direction=="G":
            for i in g.corner_point:
                if carIndex[0]+1==i[0] and carIndex[1]==i[1]:
                    g.matrix[i[0],i[1]+1]=0

        elif direction=="D":
            for i in g.corner_point:
                if carIndex[1]+1==i[1] and carIndex[0]==i[0]:
                    g.matrix[i[0]-1,i[1]]=0

        elif direction=="B":
            for i in g.corner_point:
                if carIndex[1]-1==i[1] and carIndex[0]==i[0]:
                    g.matrix[i[0]+1,i[1]]=0

        g.x_main,g.y_main=get_firstTrajectory()

    elif levhAdi == "sağaDönlmez" or levhAdi == "saga_donulmez":
        if direction=="K":
            for i in g.corner_point:
                if carIndex[0]-1==i[0] and carIndex[1]==i[1]:
                    g.matrix[i[0],i[1]+1]=0

        elif direction=="G":
            for i in g.corner_point:
                if carIndex[0]+1==i[0]:
                    g.matrix[i[0],i[1]-1]=0

        elif direction=="D":
            for i in g.corner_point:
                if carIndex[1]+1==i[1]:
                    g.matrix[i[0]+1,i[1]]=0

        elif direction=="B":
            for i in g.corner_point:
                if carIndex[1]-1==i[1]:
                    g.matrix[i[0]-1,i[1]]=0
        g.vehicle_location=[current_x,current_y]
        g.x_main,g.y_main=get_firstTrajectory()

    elif levhAdi == "dur":

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

    return g.x_main,g.y_main
# g.x_main,g.y_main=get_firstTrajectory()

# plt.plot(g.x_main, g.y_main, marker='o', linestyle='-')
# plt.title('Rotaa')
# plt.xlabel('X Ekseni')
# plt.ylabel('Y Ekseni')
# plt.xlim(-90, 10)
# plt.ylim(-10, 90)
# plt.grid(True)
# plt.show()

# currentx = float(input("X koordinatını giriniz: "))
# currenty = float(input("Y koordinatını giriniz: "))

# levha(['giris_olmayan_yol'],"K",currentx,currenty)

# plt.ylim(-10, 90), g.y_main, marker='o', linestyle='-')
# plt.grid(True)aa')
# plt.show()('X Ekseni')
# plt.ylabel('Y Ekseni')
# plt.xlim(-90, 10)
# plt.ylim(-10, 90)
# plt.grid(True)
# plt.show()
