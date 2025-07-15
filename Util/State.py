from Util import Global
from Util  import Distance 

def set_state():
    # print(Distance.finaldistance(),Global.stop_points[1][0],Global.stop_points[1][1])
    if Distance.finaldistance()<=10:
        if Global.direction=="K" and (Global.stop_points[0][1] - Global.current_y)>0:
            Global.state="bekle"
        elif Global.direction=="G" and (Global.stop_points[0][1] - Global.current_y)<0:
            Global.state="bekle"
        elif Global.direction=="B" and (Global.stop_points[0][0] - Global.current_x)<0:
            Global.state="bekle"
        elif Global.direction=="D" and (Global.stop_points[0][0] - Global.current_x)>0:
            Global.state="bekle"
    if Distance.finaldistance()<=0.3 and len(Global.stop_points)==1 :
        Global.state="dur"
        Global.breaking=1
    elif Distance.finaldistance()>10:
        Global.state="sabit"
    
    
