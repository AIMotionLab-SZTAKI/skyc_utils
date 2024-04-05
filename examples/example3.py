import numpy as np
from skyc_utils.skyc_maker import Trajectory, write_skyc, XYZYaw, evaluate_pickle, TrajectoryType
from skyc_utils.utils import select_file

if __name__ == '__main__':
    traj = Trajectory(traj_type=TrajectoryType.POLY4D, degree=7)
    #TODO: ACCEPT ITERABLES INSTEAD OF XYZYaw
    traj.set_start(XYZYaw(0, 0, 0, 0))
    traj.add_goto(XYZYaw(0, 0, 0.5, 0), continuity=3, dt=3)  # takeoff
    traj.add_goto(XYZYaw(0.7, 0, 0.8, 0), continuity=3, dt=3)
    traj.add_goto(XYZYaw(0.7, 0.7, 1.0, 0), continuity=3, dt=3)
    traj.add_goto(XYZYaw(0.7, -0.7, 1.2, 0), continuity=3, dt=3)
    traj.add_goto(XYZYaw(-0.7, -0.7, 0.7, 0), continuity=3, dt=3)
    traj.add_goto(XYZYaw(-0.7, 0.7, 0.5, 0), continuity=3, dt=3)

    traj.add_goto(XYZYaw(0.7, 0, 0.8, 0), continuity=3, dt=3)
    traj.add_goto(XYZYaw(0.7, 0.7, 1.0, 0), continuity=3, dt=3)
    traj.add_goto(XYZYaw(0.7, -0.7, 1.2, 0), continuity=3, dt=3)
    traj.add_goto(XYZYaw(-0.7, -0.7, 0.7, 0), continuity=3, dt=3)
    traj.add_goto(XYZYaw(-0.7, 0.7, 0.5, 0), continuity=3, dt=3)

    traj.add_goto(XYZYaw(0, 0, 0.5, 0), continuity=3, dt=3)
    traj.add_goto(XYZYaw(0, 0, 0, 0), continuity=3, dt=3)

    traj.add_parameter(-10, "stabilizer.controller", 2)

    write_skyc([traj])
