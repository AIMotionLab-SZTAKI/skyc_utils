import numpy as np
from skyc_utils.skyc_maker import Trajectory, write_skyc, XYZYaw, evaluate_pickle
from skyc_utils.utils import select_file

if __name__ == '__main__':
    traj = Trajectory()
    traj.set_start(XYZYaw(0, 0, 0, 0))
    traj.add_goto(XYZYaw(0, 0, 0.5, 0), 3)  # takeoff
    traj.add_goto(XYZYaw(0.7, 0, 0.8, 0), 3)
    traj.add_goto(XYZYaw(0.7, 0.7, 1.0, 0), 3)
    traj.add_goto(XYZYaw(0.7, -0.7, 1.2, 0), 3)
    traj.add_goto(XYZYaw(-0.7, -0.7, 0.7, 0), 3)
    traj.add_goto(XYZYaw(-0.7, 0.7, 0.5, 0), 3)

    traj.add_goto(XYZYaw(0.7, 0, 0.8, 0), 3)
    traj.add_goto(XYZYaw(0.7, 0.7, 1.0, 0), 3)
    traj.add_goto(XYZYaw(0.7, -0.7, 1.2, 0), 3)
    traj.add_goto(XYZYaw(-0.7, -0.7, 0.7, 0), 3)
    traj.add_goto(XYZYaw(-0.7, 0.7, 0.5, 0), 3)

    traj.add_goto(XYZYaw(0, 0, 0.5, 0), 3)
    traj.add_goto(XYZYaw(0, 0, 0, 0), 3)

    write_skyc([traj])
