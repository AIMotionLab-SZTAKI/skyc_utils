import numpy as np
from skyc_utils.skyc_maker import Trajectory, write_skyc, XYZYaw, evaluate_pickle, TrajectoryType
from skyc_utils.utils import select_file


def fig8(t, x_start, y_start, x_max, y_max, z, yaw_max=0.0):
    period = 5
    rho = np.pi / period * t
    x = list(x_start + np.sin(rho) * x_max)
    y = list(y_start + np.sin(2*rho) * y_max)
    z = [z for _ in x]
    yaw = list(np.sin(1.5*rho) * yaw_max)
    return t, x, y, z, yaw


if __name__ == '__main__':
    traj = Trajectory(TrajectoryType.POLY4D, degree=5)
    traj.set_start(XYZYaw(0, 0, 0, 0))
    traj.add_goto(XYZYaw(1, 1, 1, 270), 5)
    traj.add_goto(XYZYaw(0, 2, 1, -45), 5)
    pickle_filename = "traj.pickle"
    t_x_y_z_yaw, parameters = evaluate_pickle(pickle_filename)

    traj.add_goto(XYZYaw(x=t_x_y_z_yaw[1][0],
                         y=t_x_y_z_yaw[2][0],
                         z=t_x_y_z_yaw[3][0],
                         yaw=t_x_y_z_yaw[4][0]), 5)
    traj.add_interpolated_traj(t_x_y_z_yaw, 50, method="mosek", fit_ends=True, force_0_derivs=True)
    traj.add_goto(XYZYaw(0, 0, 0, 0), 5)
    traj.parameters = [[-100, "stabilizer.controller", 2],
                       [-5, "stabilizer.controller", 1],
                       [5, "stabilizer.controller", 2]]
    write_skyc([traj])
