# two_drones.py
import numpy as np
from scipy.interpolate import BSpline

from skyc_utils.trajectory import (
    Trajectory, TrajectoryType,
    Pose, Velocity, Acceleration, Jerk  # imported for convenience if you tweak gotos later
)
from skyc_utils.skyc import Skyc, plot_skyc_trajectories


def make_clamped_uniform_knots(T: float, n_ctrl: int, k: int) -> np.ndarray:
    """
    Create a clamped, uniform knot vector on [0, T] for a degree-k B-spline
    with n_ctrl control points.
    """
    if n_ctrl <= k:
        raise ValueError("n_ctrl must be > k")
    # number of interior knots
    n_int = n_ctrl - k - 1
    if n_int > 0:
        interior = np.linspace(0.0, T, n_int + 2)[1:-1]  # exclude endpoints
    else:
        interior = np.array([], dtype=float)
    t = np.r_[np.zeros(k + 1), interior, np.full(k + 1, T)]
    return t


def spiral_controls(center_x: float, center_y: float,
                    r: float, z0: float, z1: float,
                    turns: float, n_ctrl: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Build control points for an ascending spiral around (center_x, center_y).
    Clamped cubic B-splines start at the first control point and end at the last,
    so the curve will start/end at these controls.
    """
    s = np.linspace(0.0, 1.0, n_ctrl)
    theta = 2.0 * np.pi * turns * s
    x_ctrl = center_x + r * np.cos(theta)
    y_ctrl = center_y + r * np.sin(theta)
    z_ctrl = z0 + (z1 - z0) * s
    return x_ctrl, y_ctrl, z_ctrl


def build_spiral_bspline(T: float,
                         center_x: float, center_y: float,
                         r: float, z0: float, z1: float,
                         turns: float,
                         n_ctrl: int = 12,
                         k: int = 3) -> tuple[BSpline, BSpline, BSpline]:
    """
    Make clamped cubic BSplines (x, y, z) over [0, T], sharing the same knots.
    """
    t = make_clamped_uniform_knots(T, n_ctrl, k)
    cx, cy, cz = spiral_controls(center_x, center_y, r, z0, z1, turns, n_ctrl)
    bsx = BSpline(t, cx, k)
    bsy = BSpline(t, cy, k)
    bsz = BSpline(t, cz, k)
    return bsx, bsy, bsz


def make_drone_traj(start: Pose,
                    takeoff_alt: float,
                    spiral_T: float,
                    spiral_center: tuple[float, float],
                    spiral_radius: float,
                    spiral_z0: float,
                    spiral_z1: float,
                    spiral_turns: float,
                    land_alt: float) -> Trajectory:
    """
    Create a trajectory:
      - goto takeoff_alt
      - BSpline ascending spiral
      - goto land_alt
    """
    traj = Trajectory(TrajectoryType.POLY4D, degree=7, start=start)

    # 1) Takeoff
    traj.add_goto(Pose(start.x, start.y, takeoff_alt, start.yaw), dt=2.0)

    # 2) Ascending spiral (ensure clamped splines so add_bspline works with your trimming)
    cx, cy = spiral_center
    bsx, bsy, bsz = build_spiral_bspline(
        T=spiral_T,
        center_x=cx, center_y=cy,
        r=spiral_radius,
        z0=spiral_z0,
        z1=spiral_z1,
        turns=spiral_turns,
        n_ctrl=12,  # tune if you want smoother / tighter control
        k=3
    )
    traj.add_bspline(bsx, bsy, bsz, yaw=None)  # yaw left at 0 via your implementation

    # Compute spiral end (clamped spline ends at last control)
    end_x = bsx.c[-1]
    end_y = bsy.c[-1]

    # 3) Land vertically where the spiral ends
    traj.add_goto(Pose(end_x, end_y, land_alt, start.yaw), dt=2.0)

    return traj


def main():
    # Build two non-colliding spirals (centers 2.0m apart, radii 0.4m → min sep ≥ 1.2m)
    # Drone 0
    start0 = Pose(0.0, 0.0, 0.0, 0.0)
    traj0 = make_drone_traj(
        start=start0,
        takeoff_alt=0.5,
        spiral_T=10.0,
        spiral_center=(0.0, 0.0),
        spiral_radius=0.4,
        spiral_z0=0.5,
        spiral_z1=1.6,
        spiral_turns=2.0,
        land_alt=0.0
    )

    # Drone 1 (offset by +2.0m in x)
    start1 = Pose(2.0, 0.0, 0.0, 0.0)
    traj1 = make_drone_traj(
        start=start1,
        takeoff_alt=0.5,
        spiral_T=10.0,
        spiral_center=(2.0, 0.0),
        spiral_radius=0.4,
        spiral_z0=0.5,
        spiral_z1=1.6,
        spiral_turns=2.0,
        land_alt=0.0
    )

    # Pack into a skyc (no lights, no parameters)
    skyc = Skyc()
    skyc.add_drone(traj0)
    skyc.add_drone(traj1)
    skyc.write(name="two_drones")  # produces two_drones.skyc in the CWD

    plot_skyc_trajectories("two_drones.skyc")


if __name__ == "__main__":
    main()