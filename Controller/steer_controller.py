import sys
import os
import math
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from Util import Global

class SteerController:

    @staticmethod
    def steer_control(current_yaw, current_x, current_y, tx, ty):
        k = 0.1  # look forward gain
        Lfc =4.2  # look-ahead distance
        L = 2.9

        derece = abs(current_yaw)
        radyan = derece * (np.pi / 180)

        alpha_hat = math.atan2(ty - current_y, (tx - current_x))    
        alpha = alpha_hat - radyan
        Lf = k * 1.9 + Lfc
        steer_output = -math.atan2(2.0 * L * math.sin(alpha) / (Lf), 1.0)

        Global.alpha = alpha
        Global.alpha_hat = alpha_hat
        Global.math_sin_alpha = math.sin(alpha)
        Global.steer_output = steer_output
        return steer_output