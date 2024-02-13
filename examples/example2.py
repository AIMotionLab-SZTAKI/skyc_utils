import numpy as np
from skyc_utils.skyc_maker import Trajectory, write_skyc, XYZYaw, evaluate_pickle
from skyc_utils.utils import select_file

if __name__ == '__main__':
    traj1 = Trajectory("POLY4D")
    traj1.set_start(XYZYaw(0, 0.5, 0, 0))
    traj1.add_goto(XYZYaw(0, 0.5, 0.5, 0), 3)  # takeoff
    traj1.add_goto(XYZYaw(0, 0.5, 0.5, 180), 5)  # half a turn in place
    traj1.add_goto(XYZYaw(0, 0.5, 0.5, 0), 5)  # half a turn again in place
    traj1.add_goto(XYZYaw(0, 0.5, 0, 0), 3)  # land
    traj1.parameters = [[-100, "stabilizer.controller", 2],
                        [-8, "stabilizer.controller", 1],
                        [-2, "stabilizer.controller", 2],
                        [5, "stabilizer.controller", 1]]

    traj2 = Trajectory("POLY4D")
    traj2.set_start(XYZYaw(0, -0.5, 0, 0))
    traj2.add_goto(XYZYaw(0, -0.5, 0.5, 0), 3)  # takeoff
    traj2.add_goto(XYZYaw(0, -0.5, 0.5, 180), 5)  # half a turn in place
    traj2.add_goto(XYZYaw(0, -0.5, 0.5, 0), 5)  # half a turn again in place
    traj2.add_goto(XYZYaw(0, -0.5, 0, 0), 3)  # land
    traj2.parameters = [[-50, "stabilizer.controller", 2],
                        [-10, "stabilizer.controller", 1],
                        [-3, "stabilizer.controller", 2],
                        [10, "stabilizer.controller", 1]]

    write_skyc([traj1, traj2])
