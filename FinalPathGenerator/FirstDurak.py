import numpy as np
import matplotlib.pyplot as plt

def quintic_trajectory_planning(q0, qf,y_start,y_end, m):
    n = q0.shape[0]
    tf=10
    # Polynomial Parameters
    a0 = np.copy(q0)
    a3 = (20 * qf - 20 * q0) / (2*tf*tf*tf)
    a4 = (30 * q0 - 30 * qf) / (2*tf*tf*tf*tf)
    a5 = (12 * qf - 12 * q0) / (2*tf*tf*tf*tf*tf)

    timesteps = np.linspace(0, 10, num = round(m))
    q = np.zeros((n,round(m)))

    for i in range(len(timesteps)):
        t = timesteps[i]
        t_2 = t * t
        t_3 = t_2 * t
        t_4 = t_3 * t
        t_5 = t_4 * t

        q[:, i] = (a0) + (a3 * t_3) + (a4 * t_4) + (a5 * t_5)
    return q

def DoguBatiHareket(x_start,x_end,y_start,y_end,x_main,y_main):
    #Batıda şerit değiştireceğimiz için x eksenindeki waypoint sayısını alacaz
    waypoint_sayisi = round(abs((x_start-x_end))*10)

    q0 = np.array([y_start])
    qf = np.array([y_end])
    qq = quintic_trajectory_planning(q0, qf, x_start, x_end,waypoint_sayisi)
    q = qq.tolist()[0]

    x_waypoints = []

    for i in range(waypoint_sayisi):
        x = x_start + (i * (x_end - x_start) / (waypoint_sayisi - 1))
        x_waypoints.append(round(x, 2))
    x_main += x_waypoints
    y = [round(deger, 2) for deger in q]
    y_main += y
    return x_main,y_main
def firstDurak(x_main,y_main):

    x_main , y_main = DoguBatiHareket(x_main[-1], -46.5, y_main[-1], 75.5, x_main, y_main)
    x_main , y_main = DoguBatiHareket(x_main[-1], -58, y_main[-1], 72, x_main, y_main)
    return x_main,y_main