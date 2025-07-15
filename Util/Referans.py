from Util import Global
from Util import Bekle

def set_referance():
    if Global.state=="sabit":
        Global.v_desired=3
    elif Global.state=="bekle":
        Bekle.bekle()
    elif Global.state=="dur":
        Global.v_desired=0
        