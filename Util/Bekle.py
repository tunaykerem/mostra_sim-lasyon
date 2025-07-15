from Util import Global as Global
from Util import Distance as Distance
import time as t

start_time = 0  
end_time = 0  

def bekle():    
    distance = Distance.finaldistance()
    final_speed = 0  
    
    acceleration = (Global.current_speed - final_speed) / distance
   
    
    elapsed_time =distance 
    if Global.current_speed - acceleration * elapsed_time<0.2:
        
        Global.v_desired=0.5
    else:
        Global.v_desired = Global.current_speed - acceleration * elapsed_time
    passing_time=0
    if Distance.finaldistance() <= 0.3:
        if Global.bekle_control:
            Global.time_start= t.time()
            Global.bekle_control=False
        Global.time_end=t.time()

        passing_time = Global.time_end-Global.time_start

        print(passing_time)

        if passing_time<=5:
            Global.v_desired = 0.0
            Global.breaking = 1
        else:
            Global.stop_points.pop(0)
            Global.state="sabit"
            Global.breaking = 0
            Global.bekle_control=True


        