import traceback
import numpy as np
from Controller.throttle_controller import ThrottlePIDController
import Util.Global as Global

class DrivingController:
    def __init__(self):
        self.pid_controller = ThrottlePIDController()

    def update(self,vehicle):   
        current_speed = np.real(np.sqrt(abs(vehicle.get_velocity().x * vehicle.get_velocity().x + vehicle.get_velocity().y * vehicle.get_velocity().y)))
        Global.current_speed=current_speed
        throttle = self.pid_controller.update(Global.v_desired, current_speed)
        return throttle
