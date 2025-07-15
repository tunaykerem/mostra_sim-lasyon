
import Util.Global as Global

def get_direction():

    if ((Global.yaw <= -60) and (Global.yaw > -110)):
        Global.direction = "K"
    elif (Global.yaw <= -160 and (Global.yaw > -210)):
        Global.direction = "B"
    elif((Global.yaw <= -240) and (Global.yaw > -300)):
        Global.direction = "G"
    elif((Global.yaw <= -330) and (Global.yaw > -390)) or ((Global.yaw <= 30) and (Global.yaw > -30)):
        Global.direction = "D"