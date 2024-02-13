import numpy as np
from skyc_utils.skyc_maker import Trajectory, write_skyc, XYZYaw
from skyc_utils.utils import extend_ppoly
from skyc_utils.utils import select_file
import pickle
import scipy.interpolate as interpolate


if __name__ == '__main__':
    traj = Trajectory()
    pickle_filename = "traj_poly.pickle"
    with open(pickle_filename, "rb") as file:
        data = pickle.load(file)
        start = XYZYaw(*[float(ppoly(0)) for ppoly in data])
        traj.set_start(start)
        ppolys = XYZYaw(*[extend_ppoly(ppoly) for ppoly in data])
        traj.add_ppoly(ppolys)
        traj.add_parameter(-10, "pptraj.traj_mode_drone", 0)
        traj.add_parameter(-9, "Lqr2.max_delay_time_ms", 50000)
        traj.add_parameter(-8, "stabilizer.controller", 7)
        traj.add_parameter(-7, "Lqr2.rod_length_safety", 0.62)
        traj.add_parameter(-2, "usd.logging", 1)
        traj.add_parameter(13, "usd.logging", 0)

        # traj.parameters = [[-10, "pptraj.traj_mode_drone", 0],
        #                    [-9, "Lqr2.max_delay_time_ms", 50000],
        #                    [-8, "stabilizer.controller", 7],
        #                    [-7, "Lqr2.rod_length_safety", 0.62],
        #                    [-2, "usd.logging", 1],
        #                    [13, "usd.logging", 0]]
        write_skyc([traj])

