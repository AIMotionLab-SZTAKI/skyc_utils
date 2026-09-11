# three_drones_collide_at_points.py
import numpy as np
from scipy.interpolate import BSpline, make_splrep

from skyc_utils.light_program import LightProgram, Color
from skyc_utils.trajectory import (
    Trajectory, TrajectoryType,
    Pose, Velocity, Acceleration, Jerk
)
from skyc_utils.skyc import Skyc, plot_skyc_trajectories


def main():
    traj0 = Trajectory(TrajectoryType.POLY4D, degree=7, start=Pose(-1, 0, 0, 0))
    traj1 = Trajectory(TrajectoryType.POLY4D, degree=7, start=Pose(0, 0, 0, 0))
    traj2 = Trajectory(TrajectoryType.POLY4D, degree=7, start=Pose(1, 0, 0, 0))
    T = 3.0
    t = np.linspace(0, 9.0, 100)
    x = np.sin(t*2*np.pi/T)
    y = np.cos(t*2*np.pi/T) - 1
    z = 1.0-np.cos(t*2*np.pi/T/3)*0.5 + 0.5
    yaw = np.sin(t*2*np.pi/T/3)*np.pi
    traj0.append_bspline(
        make_splrep(t, x-1.0),
        make_splrep(t, y),
        make_splrep(t, z),
        make_splrep(t, yaw)
    )
    traj0.prepend_goto(start=Pose(-1, 0, 0, 0),
                       start_vel=Velocity(),
                       start_acc=Acceleration(),
                       start_jerk=Jerk(),
                       dt=2)
    traj0.append_goto(end=Pose(-1, 0, 0, 0),
                       dt=2)

    traj1.append_bspline(
        make_splrep(t, -x),
        make_splrep(t, y),
        make_splrep(t, z),
        make_splrep(t, yaw)
    )
    traj1.prepend_goto(start=Pose(0, 0, 0, 0),
                       start_vel=Velocity(),
                       start_acc=Acceleration(),
                       start_jerk=Jerk(),
                       dt=2)
    traj1.append_goto(end=Pose(0, 0, 0, 0),
                      end_vel=Velocity(),
                      dt=2)

    traj2.append_bspline(
        make_splrep(t, x + 1.0),
        make_splrep(t, y),
        make_splrep(t, z),
        make_splrep(t, yaw)
    )
    traj2.prepend_goto(start=Pose(1, 0, 0, 0),
                       start_vel=Velocity(),
                       start_acc=Acceleration(),
                       start_jerk=Jerk(),
                       dt=2)
    traj2.append_goto(end=Pose(1, 0, 0, 0),
                      end_vel=Velocity(),
                      end_acc=Acceleration(),
                      dt=2)

    lights0 = LightProgram()
    lights0.set_color(3, Color.WHITE)
    lights0.set_color(6, Color.MAGENTA)
    lights1 = LightProgram()
    lights1.set_color(0, Color.BLUE)

    # Pack into a skyc (no lights/parameters)
    skyc = Skyc()

    skyc.add_drone(traj0, lights0)
    skyc.add_drone(traj1, lights1)
    skyc.add_drone(traj2, LightProgram())
    skyc.write(name="three_drones_collisions")

    # Visual check (separate windows per drone; pose/vel/acc)
    plot_skyc_trajectories("three_drones_collisions.skyc")


if __name__ == "__main__":
    main()
