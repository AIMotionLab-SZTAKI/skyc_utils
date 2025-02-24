import os
import time

from skyc_utils.skyc_maker import Trajectory, XYZYaw, Color, LightProgram, write_skyc
from skyc_utils.skyc_inspector import get_light_data

if __name__ == '__main__':
    traj = Trajectory()
    traj.set_start(XYZYaw(0, 0, 0, 0))
    traj.add_goto(XYZYaw(0, 0, 0.5, 0), dt=3)
    traj.add_goto(XYZYaw(0.7, 0, 0.8, 0), dt=3)
    traj.add_goto(XYZYaw(0.7, 0.7, 1.0, 0), dt=3)
    traj.add_goto(XYZYaw(0.7, -0.7, 1.2, 0), dt=3)
    traj.add_goto(XYZYaw(0, 0, 0.5, 0), dt=3)
    traj.add_goto(XYZYaw(0, 0, 0, 0),  dt=3)

    lights = LightProgram()
    lights.append_color(Color.WHITE, 3)
    lights.append_color(Color(100, 200, 255), 2)
    lights.append_color(Color.MAGENTA, 1.5)
    lights.append_color(Color.RED, 2.5)
    lights.append_color(Color.YELLOW, 4)
    lights.append_color(Color(120, 250, 50), 10)

    write_skyc([traj], [lights])