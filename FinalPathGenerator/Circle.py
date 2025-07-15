import matplotlib.pyplot as plt
import numpy as np

#radius_x: x eksenindeki yarıçapı:
#radius_y: y eksenindeki yarıçapı:
#center_x: Merkezin x eksenindeki konumu
#center_y: Merkezin y eksenindeki konumu

def drawingCircle(radius_x,radius_y,center_x,center_y,starting_angle,ending_angle,x_main,y_main):
    
    x_waypoints = []
    y_waypoints = []
    theta = np.linspace(starting_angle,ending_angle, 100)
    x = center_x + radius_x * np.cos(theta)
    y = center_y + radius_y * np.sin(theta)
    for i in x:
        x_waypoints.append(round(i,2))
    for i in y:
        y_waypoints.append(round(i,2))
    x_main =x_main+ x_waypoints
    y_main = y_main+y_waypoints
    return x_main,y_main



# starting_angle= 0*np.pi
# ending_angle = np.pi/2
# x_main,y_main = [6],[0]
# x_waypoints,y_waypoints = drawingCircle(6, 6, 0, 0, starting_angle, ending_angle,x_main,y_main)
# x_son = x_waypoints[-1]
# y_son = y_waypoints[-1]
# # Elipsi çiz
# plt.plot(x_waypoints, y_waypoints, marker='o', linestyle='-')
# plt.title('Path for Intersections')
# plt.xlabel('X Axis')
# plt.ylabel('Y Axis')
# plt.grid(True)
# plt.show()
























# x_waypoints,y_waypoints = drawingCircle(6, 6, -6, 55, 0, np.pi/2)
# print(x_waypoints)
# x_son = x_waypoints[-1]
# y_son = y_waypoints[-1]
# # Elipsi çiz
# plt.plot(x_waypoints, y_waypoints, marker='o', linestyle='-')
# plt.title('Örnek Grafiği')
# plt.xlabel('X Ekseni')
# plt.ylabel('Y Ekseni')
# plt.grid(True)
# plt.show()



# print(x_waypoints[50],y_waypoints[50])
# plt.plot(x_waypoints, y_waypoints, marker='o', linestyle='-')
# plt.title('Örnek Grafiği')
# plt.xlabel('X Ekseni')
# plt.ylabel('Y Ekseni')
# plt.grid(True)
# plt.show()


# # Son noktanın üzerine değeri yazdır
# plt.annotate(f'({x_son:.2f}, {y_son:.2f})', (x_son, y_son), textcoords="offset points", xytext=(0,10), ha='center')
# plt.grid(True)
# plt.show()


# import math

# import matplotlib.pyplot as plt
# import numpy as np
# def getCircleCCW(a,b,x_start,x_end,center_x,center_y,radius,control):
#     # Elipsin açıları sadece ilk çeyrek için sınırlanır (0 ile pi/2 arası)
#     x_waypoints = []
#     y_waypoints = []
#     if (control == "DG"):
#         theta = np.linspace(np.pi / 2, 0, 100)
#         x = center_x + a * np.cos(theta)
#         y = center_y + b * np.sin(theta)
#         for i in x:
#             x_waypoints.append(round(i, 2))
#         for i in y:
#             y_waypoints.append(round(i, 2))
#     elif (control == "B"):
#         theta = np.linspace(np.pi / 2, np.pi, 100)
#         x = center_x + a * np.cos(theta)
#         y = center_y + b * np.sin(theta)
#         for i in x:
#             x_waypoints.append(round(i, 2))
#         for i in y:
#             y_waypoints.append(round(i, 2))
#     elif (control == "BK"):
#         theta = np.linspace(3 * np.pi / 2, 1 * np.pi, 100)
#         x = center_x + a * np.cos(theta)
#         y = center_y + b * np.sin(theta)

#         for i in x:
#             x_waypoints.append(round(i, 2))
#         for i in y:
#             y_waypoints.append(round(i, 2))
#     return x_waypoints,y_waypoints

# def VirajAlmaKuzeyGuneyCCW(a, b, x_main_end, x_main_end_minus, x_main_end_minus1, y_main_end, r,yon,x_main,y_main):
#     # KuzeydenBatiyaViraj
#     if (yon=="K"):
#         x_values, y_values = getCircleCCW(a, b, x_main_end, x_main_end_minus, x_main_end_minus1, y_main_end, 6,yon)
#         x_main += x_values
#         y_main += y_values
#     return x_main,y_main



# x_main, y_main = [0],[0]
# x_main, y_main = VirajAlmaKuzeyGuneyCCW(6,10,x_main[-1],x_main[-1]-6,x_main[-1]-6,y_main[-1],6,"K",x_main,y_main)
# print(x_main[-1],y_main[-1])
# plt.plot(x_main, y_main, marker='o', linestyle='-')
# plt.title('Örnek Grafiği')
# plt.xlabel('X Ekseni')
# plt.ylabel('Y Ekseni')
# plt.grid(True)
# plt.show()


# """
# Alınacak Parametreler;
#     1: 
# """
