import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import make_interp_spline, BSpline

from skyc_utils.trajectory import (
    Trajectory, TrajectoryType,
    Pose, Velocity, Acceleration, Jerk
)
from skyc_utils.skyc import Skyc
from skyc_utils.light_program import LightProgram, Color

def eval_trajectory_dense(traj: Trajectory, n: int = 1000):
    T = traj.duration
    t = np.linspace(0.0, T, n) if T > 0 else np.array([0.0])

    px = []; py = []; pz = []; pyaw = []
    vx = []; vy = []; vz = []; vyaw = []
    ax = []; ay = []; az = []; ayaw = []
    jx = []; jy = []; jz = []; jyaw = []

    for ti in t:
        fs = traj.evaluate(float(ti))
        px.append(fs.pose.x);  py.append(fs.pose.y);  pz.append(fs.pose.z);  pyaw.append(fs.pose.yaw)
        vx.append(fs.vel.x);   vy.append(fs.vel.y);   vz.append(fs.vel.z);   vyaw.append(fs.vel.yaw)
        ax.append(fs.acc.x);   ay.append(fs.acc.y);   az.append(fs.acc.z);   ayaw.append(fs.acc.yaw)
        jx.append(fs.jerk.x);  jy.append(fs.jerk.y);  jz.append(fs.jerk.z);  jyaw.append(fs.jerk.yaw)

    to_np = lambda lst: np.array(lst, dtype=float)
    return (
        t,
        (to_np(px), to_np(py), to_np(pz), to_np(pyaw)),
        (to_np(vx), to_np(vy), to_np(vz), to_np(vyaw)),
        (to_np(ax), to_np(ay), to_np(az), to_np(ayaw)),
        (to_np(jx), to_np(jy), to_np(jz), to_np(jyaw)),
    )

def add_spiral_bspline_segment(traj: Trajectory, turns: float = 2.0, seg_time: float = 6.0):
    """
    Construct an ascending spiral as BSplines and append via traj.add_bspline(...).
    The spiral starts at the current end pose to avoid discontinuities.
    """
    # Current end pose/time
    _, end_fs = traj.end_conditions
    x0, y0, z0, yaw0 = end_fs.pose.x, end_fs.pose.y, end_fs.pose.z, end_fs.pose.yaw

    # Build param samples for the spiral (Archimedean-style in XY, linear in Z)
    n_ctrl = 60  # number of interpolation points to build the BSpline
    t = np.linspace(0, seg_time, n_ctrl)

    theta = np.linspace(0.0, 2.0 * np.pi * turns, n_ctrl)
    r_start, r_end = 0.05, 0.7
    r = np.linspace(r_start, r_end, n_ctrl)

    # Base spiral around (0,0); we will shift to (x0, y0)
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    z = np.linspace(0.0, 0.6, n_ctrl)  # ascend 0.6 m (adjust as you like)

    # Shift so that the spiral starts exactly at the current end pose
    x += (x0 - x[0])
    y += (y0 - y[0])
    z += (z0 - z[0])

    # Approximate tangent-based yaw from finite differences, unwrap for continuity
    dx = np.gradient(x, t)
    dy = np.gradient(y, t)
    yaw = np.unwrap(np.arctan2(dy, dx))

    # Optionally align initial yaw to the current yaw (smallest adjustment)
    yaw += (yaw0 - yaw[0])

    # Create cubic BSplines for each axis (domain is [t_start, t_end])
    x_bs: BSpline = make_interp_spline(t, x, k=3)
    y_bs: BSpline = make_interp_spline(t, y, k=3)
    z_bs: BSpline = make_interp_spline(t, z, k=3)
    yaw_bs: BSpline = make_interp_spline(t, yaw, k=3)

    # Append via the new method you'll implement
    traj.add_bspline(x_bs, y_bs, z_bs, yaw_bs)

def main():
    # Build a trajectory (degree 7 to allow up to jerk constraints)
    traj = Trajectory(TrajectoryType.POLY4D, degree=7, start=Pose(0.0, 0.0, 0.0, 0.0))

    # 1) Position-only goto
    traj.add_goto(Pose(1.0, 0.0, 0.8, 0.0), dt=2.0)

    # 2) Arrive with some velocity
    traj.add_goto(Pose(1.5, 0.5, 1.0, 0.5), dt=1.5, end_vel=Velocity(0.5, 0.5, 0.5, 0.0))

    # 3) Match pos + vel + acc (acc zeros)
    traj.add_goto(
        Pose(1.0, 1.0, 1.0, 1.0),
        dt=1.0,
        end_vel=Velocity(0.2, 0.0, 0.0, 0.0),
        end_acc=Acceleration(0.0, 0.0, 0.0, 0.0),
    )

    # 4) Match up to jerk (all zeros on arrival)
    traj.add_goto(
        Pose(0.2, 0.2, 0.8, 0.2),
        dt=2.0,
        end_vel=Velocity(0.0, 0.0, 0.0, 0.0),
        end_acc=Acceleration(0.0, 0.0, 0.0, 0.0),
        end_jerk=Jerk(0.0, 0.0, 0.0, 0.0),
    )

    # 5) Append a smooth ascending spiral via BSpline
    add_spiral_bspline_segment(traj, turns=2.5, seg_time=6.0)

    # Evaluate densely
    t, pose, vel, acc, jerk = eval_trajectory_dense(traj, n=2000)
    x, y, z, yaw = pose
    vx, vy, vz, vyaw = vel
    ax, ay, az, ayaw = acc
    jx, jy, jz, jyaw = jerk

    # Yaw in degrees for readability
    yaw_deg  = np.rad2deg(yaw)
    vyaw_deg = np.rad2deg(vyaw)
    ayaw_deg = np.rad2deg(ayaw)
    jyaw_deg = np.rad2deg(jyaw)

    # Plot
    fig, axs = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
    fig.suptitle("Trajectory debug: pose / vel / acc / jerk (with BSpline spiral)")

    # Pose
    axs[0].plot(t, x, label='x [m]')
    axs[0].plot(t, y, label='y [m]')
    axs[0].plot(t, z, label='z [m]')
    ax0b = axs[0].twinx()
    ax0b.plot(t, yaw_deg, linestyle='--', alpha=0.7, label='yaw [deg]')
    axs[0].set_ylabel("pose")
    axs[0].grid(True)
    lines1, labels1 = axs[0].get_legend_handles_labels()
    lines2, labels2 = ax0b.get_legend_handles_labels()
    axs[0].legend(lines1 + lines2, labels1 + labels2, loc='upper right')

    # Velocity
    axs[1].plot(t, vx, label='vx [m/s]')
    axs[1].plot(t, vy, label='vy [m/s]')
    axs[1].plot(t, vz, label='vz [m/s]')
    ax1b = axs[1].twinx()
    ax1b.plot(t, vyaw_deg, linestyle='--', alpha=0.7, label='yaẇ [deg/s]')
    axs[1].set_ylabel("vel")
    axs[1].grid(True)
    l1, lab1 = axs[1].get_legend_handles_labels()
    l2, lab2 = ax1b.get_legend_handles_labels()
    axs[1].legend(l1 + l2, lab1 + lab2, loc='upper right')

    # Acceleration
    axs[2].plot(t, ax, label='ax [m/s²]')
    axs[2].plot(t, ay, label='ay [m/s²]')
    axs[2].plot(t, az, label='az [m/s²]')
    ax2b = axs[2].twinx()
    ax2b.plot(t, ayaw_deg, linestyle='--', alpha=0.7, label='yaẅ [deg/s²]')
    axs[2].set_ylabel("acc")
    axs[2].grid(True)
    l1, lab1 = axs[2].get_legend_handles_labels()
    l2, lab2 = ax2b.get_legend_handles_labels()
    axs[2].legend(l1 + l2, lab1 + lab2, loc='upper right')

    # Jerk
    axs[3].plot(t, jx, label='jx [m/s³]')
    axs[3].plot(t, jy, label='jy [m/s³]')
    axs[3].plot(t, jz, label='jz [m/s³]')
    ax3b = axs[3].twinx()
    ax3b.plot(t, jyaw_deg, linestyle='--', alpha=0.7, label='yaw⃛ [deg/s³]')
    axs[3].set_ylabel("jerk")
    axs[3].set_xlabel("time [s]")
    axs[3].grid(True)
    l1, lab1 = axs[3].get_legend_handles_labels()
    l2, lab2 = ax3b.get_legend_handles_labels()
    axs[3].legend(l1 + l2, lab1 + lab2, loc='upper right')

    # Mark segment boundaries
    if traj.polynomial is not None:
        knots = traj.polynomial.x.x  # shared across axes
        for axp in axs:
            for k in knots:
                axp.axvline(k, color='k', alpha=0.15, linewidth=1)

    traj.export_json(True)

    traj.add_parameter(2.0, "paramName", 1)

    lights = LightProgram()
    lights.append_color(Color.WHITE, 3)
    lights.append_color(Color(100, 200, 255), 2)
    lights.append_color(Color.MAGENTA, 1.5)
    lights.append_color(Color.RED, 2.5)
    lights.append_color(Color.YELLOW, 4)
    lights.append_color(Color(120, 250, 50), 10)

    skyc = Skyc()
    skyc.add_drone(traj, lights)
    skyc.write()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
