import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import make_interp_spline, BSpline

from skyc_utils.trajectory import (
    Trajectory, TrajectoryType,
    Pose, Velocity, Acceleration, Jerk
)
from skyc_utils.skyc import Skyc
from skyc_utils.light_program import LightProgram, Color

def main():
    traj = Trajectory(traj_type=TrajectoryType.COMPRESSED, degree=3)
    traj.append_goto(Pose(1.0, 0.0, 0.8, 0.0), dt=10)
    traj.append_goto(Pose(1.5, 0.5, 1.0, 0.5), dt=10, end_vel=Velocity(0.5, 0.5, 0.5, 0.0))
    traj.append_goto(Pose(1.0, 0.0, 0.0, 0.0), dt=10)
    lights = LightProgram()
    lights.set_color(3, Color.WHITE)
    lights.set_color(5, Color(100, 200, 255), )
    lights.set_color(6.5, Color.MAGENTA, )
    lights.set_color(9, Color.RED)
    lights.set_color(13, Color.YELLOW, )
    lights.set_color(15, Color(120, 250, 50))

    skyc = Skyc()
    skyc.add_drone(traj, lights)
    skyc.write()



if __name__ == "__main__":
    main()