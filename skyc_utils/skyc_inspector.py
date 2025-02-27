import os
import zipfile
import shutil
import json
from typing import List, Tuple, Optional, Any, Union
import matplotlib.figure
import matplotlib.pyplot as plt
import numpy as np
import scipy.interpolate as interpolate
import bisect
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import pickle
from skyc_utils.utils import select_file, cleanup
from scipy.interpolate import BPoly


def assert_no_car_collision(drones: List[List[List[float]]], car: List[List[float]], TIMESTEP):
    """Function that checks if the car's 'pole' has collided with any drones and raises an error if it has."""
    for idx, car_timestamp in enumerate(car[0]):
        for drone in drones:
            drone_timestamp = drone[0][idx]
            assert abs(car_timestamp - drone_timestamp) < TIMESTEP / 50
            distance = np.sqrt((drone[1][idx] - car[1][idx]) ** 2 + (drone[2][idx] - car[2][idx]) ** 2)
            if distance < 0.2:
                print(f"CRASH DETECTED BETWEEN DRONE AND CAR AT {drone_timestamp}")
            # assert distance > 0.2


def eval_if_car(eval_times, car_file):
    """Function that checks if a car log file is present, and if it is, processes its data into an evaluation
    variable, similar to that of the drones: [[t0, t1, ..], [x0, x1, ...], [y0, y1, ...], [z0, z1, ...]]"""

    def get_index_by_time(times: List[float], t: float):
        """Determine the index of the trajectory segment into which the given 't' falls."""
        index = 0
        for i in range(len(times)):
            if times[i] < t:
                index = i
            else:
                break
        return index
    if os.path.exists(car_file):
        car_trajs = []
        with open(car_file, 'r') as file:
            for line in file:
                parts = line.strip().split(": ")
                delay, tck = pickle.loads(eval(parts[1]))
                car_trajs.append((delay+float(parts[0]), tck))
        car_eval = [[], [], [], []]
        for eval_time in eval_times:
            car_eval[0].append(eval_time)
            traj_idx = get_index_by_time([car_traj[0] for car_traj in car_trajs], eval_time)
            # print(f"Traj idx: {traj_idx}")
            start_time, tck = car_trajs[traj_idx]
            tck_eval = interpolate.splev(min(tck[0][-1], eval_time - start_time), tck)
            car_eval[1].append(float(tck_eval[0]))
            car_eval[2].append(float(tck_eval[1]))
            car_eval[3].append(float(tck_eval[2]))
        return car_eval
    else:
        return None


def rotate_point_around_origin(angle_deg: float, point: np.ndarray) -> np.ndarray:
    """Function that takes a point and an angle, and returns that point rotated by the angle around the Z axis."""
    angle_rad = np.radians(angle_deg)
    rotmat = np.array([[np.cos(angle_rad), -np.sin(angle_rad), 0],
                       [np.sin(angle_rad), np.cos(angle_rad), 0],
                       [0, 0, 1]])
    return np.dot(rotmat, point)


def car_position_to_vertices(position: List[float], CAR_R, CAR_H) -> List[List[np.ndarray]]:
    """Function that approximates the car's taken 'pole' by a prism with a polygon base. It takes the position of the
    car and returns the vertices of the prism."""
    angles = 10  # the angles of the polygon, larger -> more like a circle
    corners = []
    faces = []
    for i in range(2 * angles):
        alpha = i * 2 * np.pi / angles
        x = position[0] + np.cos(alpha) * CAR_R
        y = position[1] + np.sin(alpha) * CAR_R
        z = 0 if i < angles else CAR_H
        corners.append([x, y, z])
    faces.append([i for i in range(angles)])
    faces.append([i for i in range(angles, 2 * angles)])
    for i in range(angles):
        faces.append([i, (i + 1) % angles, (i + 1) % angles + angles, i + angles])
    return [[np.array(corners[vertex]) for vertex in face] for face in faces]


def drone_pose_to_vertices(pose: List[float], L, H, P, DEFAULT_ANGLE) -> List[List[np.ndarray]]:
    """Function that takes a pose and returns the vertices that the drone's model will contain in that pose. The pose
    may or may not contain a heading. If it doesn't, then the default angle shall be used."""
    corners = [[-L / 2, -L / 2, -H / 2],
               [L / 2, -L / 2, -H / 2],
               [L / 2, L / 2, -H / 2],
               [-L / 2, L / 2, -H / 2],
               [L / 2, -L / 2, H / 2],
               [-L / 2, -L / 2, H / 2],
               [-L / 2, L / 2, H / 2],
               [L / 2, L / 2, H / 2],
               [L / 2 + P, 0,
                0]]  # the 3D coordinates of the drone model, which is a box with a triangular tip on its front
    faces = [[3, 2, 1, 0],
             [7, 6, 5, 4],
             [0, 5, 6, 3],
             [0, 1, 4, 5],
             [2, 3, 6, 7],
             [1, 8, 4],
             [1, 2, 8],
             [4, 8, 7],
             [2, 7, 8]]  # the corners defining the faces of the drone model, with normal vector pointing outward
    angle_deg = pose[3] if len(pose) == 4 else DEFAULT_ANGLE  # check if pose includes yaw
    # we first rotate *then* translate, else the translation gets rotated as well
    rotated_corners = np.array([rotate_point_around_origin(angle_deg, np.array(corner)) for corner in corners])
    rotated_translated_corners = rotated_corners + np.array(pose[:3])
    return [[rotated_translated_corners[vertex] for vertex in face] for face in faces]  # extract the vertices


def unpack_skyc_file(skyc_filename: str) -> str:
    '''Function that takes a skyc file and extracts its contents neatly into a folder, as if we used winrar. Returns
    the name of this folder.'''
    folder_name = os.path.splitext(skyc_filename)[0]  # first element of the list is the file name, second is ".skyc"
    if os.path.exists(folder_name):  # if there is a leftover folder from a previous run, delete it!
        shutil.rmtree(folder_name)
    os.makedirs(folder_name)  # make a new folder, named after the skyc file
    with zipfile.ZipFile(skyc_filename, 'r') as zip_ref:  # then extract everything into it
        zip_ref.extractall(folder_name)
    return folder_name


def get_traj_data(skyc_file: str) -> List[dict]:
    '''Function that extracts the contents of the trajectory.json files in the provided skyc file. Returns the
    dictionary containing this data.'''
    folder_name = unpack_skyc_file(skyc_file)  # unpack the skyc file (it's like a zip)
    drones_folder = os.path.join(folder_name, "drones")  # within it, there should be a 'drones' folder for trajectories
    traj_data = []
    for root, dirs, files in os.walk(drones_folder):
        # iterating over the files and folders in the drones folder, we are looking for trajectory files
        if 'trajectory.json' in files:
            with open(os.path.join(root, 'trajectory.json'), 'r') as json_file:
                data = json.load(json_file)
                points = data.get("points")
                assert points is not None
                data["has_yaw"] = True if len(points[0][1]) == 4 else False  # determine if there is a yaw trajectory
                traj_data.append(data)
                traj_type = data.get("type", "COMPRESSED").upper()
                # compressed trajectories can only be of degree 1, 3 and 7 as per the bitcraze documentation
                # if a trajectory is not compressed, it is poly4d, which can only have degrees up to 7
                ctrl_point_num = [0, 2, 6] if traj_type == "COMPRESSED" else [0, 1, 2, 3, 4, 5, 6]
                for point in points:
                    if len(point[2]) not in ctrl_point_num:
                        print(f"point2: {point[2]}")
                    # assert len(point[2]) in ctrl_point_num  # throw an error if the degree is not matching the type!
    cleanup(files=[], folders=[folder_name])
    return traj_data

def get_light_data(skyc_file: str) -> list[dict]:
    """
    TODO: docstring
    """
    folder_name = unpack_skyc_file(skyc_file)  # unpack the skyc file (it's like a zip)
    drones_folder = os.path.join(folder_name, "drones")
    light_data = []
    for root, dirs, files in os.walk(drones_folder):
        if 'lights.json' in files:
            with open(os.path.join(root, 'lights.json'), 'r') as json_file:
                data = json.load(json_file)
                light_data.append(data)
    cleanup(files=[], folders=[folder_name])
    return light_data

def get_data(skyc_file: str) -> list[tuple[dict, dict]]:
    """
    TODO: docstring
    """
    folder_name = unpack_skyc_file(skyc_file)  # unpack the skyc file (it's like a zip)
    drones_folder = os.path.join(folder_name, "drones")  # within it, there should be a 'drones' folder for trajectories
    ret = []
    for root, dirs, files in os.walk(drones_folder):
        if 'trajectory.json' in files:
            with open(os.path.join(root, 'trajectory.json'), 'r') as json_file:
                data = json.load(json_file)
                points = data.get("points")
                assert points is not None
                data["has_yaw"] = True if len(points[0][1]) == 4 else False  # determine if there is a yaw trajectory
                traj_data = data
        else:
            traj_data = None
        if 'lights.json' in files:
            with open(os.path.join(root, 'lights.json'), 'r') as json_file:
                data = json.load(json_file)
                light_data = data
        else:
            light_data = None
        if traj_data is not None or light_data is not None:
            ret.append((traj_data, light_data))
    cleanup(files=[], folders=[folder_name])
    return ret



def extend_takeoff_land(traj_data: List[dict]) -> Tuple[float, float]:
    '''Function that takes the trajectories and adds a segment to their end or start, so that they start and end at
     the same time. Returns this start and end time. This is important for the animation later.'''
    last_land_time = 0.0
    first_takeoff_time = 0.0
    # first we loop over the trajectories to determine first_takeoff_time and last_land_time
    for trajectory in traj_data:
        landingTime = trajectory.get("landingTime")
        takeoffTime = trajectory.get("takeoffTime")
        last_land_time = landingTime if landingTime > last_land_time else last_land_time
        first_takeoff_time = takeoffTime if takeoffTime < first_takeoff_time else first_takeoff_time
    # then we loop over the trajectories again in order to insert the extension that ensures that they are all the same
    # duration by adding an extension to their beginning or end as needed
    for trajectory in traj_data:
        if trajectory.get("landingTime") < last_land_time:
            last_point = trajectory["points"][-1]
            extension = [last_land_time, last_point[1], []]
            trajectory["points"].append(extension)
        if trajectory.get("takeoffTime") > first_takeoff_time:
            first_point = trajectory["points"][0]
            extension = [first_takeoff_time, first_point[1], []]
            trajectory["points"].insert(0, extension)
    return first_takeoff_time, last_land_time


def evaluate_segment(points: List[List[float]], start_time: float, end_time: float,
                     eval_time, has_yaw: bool, LIMITS) -> Tuple[float, ...]:
    # TODO: find a more efficient method
    '''Function that takes the control points of a bezier curve, creates an interpolate.BPoly object for each
    dimension of the curve, evaluates them at the given time and returns a tuple with the time, x, y, z and yaw.'''
    # The bernstein coefficients are simply the coordinates of the control points for each dimension.
    x_coeffs = [point[0] for point in points]
    y_coeffs = [point[1] for point in points]
    z_coeffs = [point[2] for point in points]
    x_BPoly = interpolate.BPoly(np.array(x_coeffs).reshape(len(x_coeffs), 1), np.array([start_time, end_time]))
    y_BPoly = interpolate.BPoly(np.array(y_coeffs).reshape(len(y_coeffs), 1), np.array([start_time, end_time]))
    z_BPoly = interpolate.BPoly(np.array(z_coeffs).reshape(len(z_coeffs), 1), np.array([start_time, end_time]))
    X = x_BPoly(eval_time)
    Y = y_BPoly(eval_time)
    Z = z_BPoly(eval_time)
    # Make sure that the trajectory doesn't take the drone outside the limits of the optitrack system!
    # assert LIMITS[0][0] < X < LIMITS[0][1] and LIMITS[1][0] < Y < LIMITS[1][1] and LIMITS[2][0] < Z < LIMITS[1][1]
    if not (LIMITS[0][0] < X < LIMITS[0][1] and LIMITS[1][0] < Y < LIMITS[1][1] and LIMITS[2][0] < Z < LIMITS[1][1]):
        print(f"OVER LIMIT! X: {X}, Y: {Y}, Z: {Z}")
    retval = [float(eval_time), float(X), float(Y), float(Z)]
    if has_yaw:
        yaw_coeffs = [point[3] for point in points]
        yaw_BPoly = interpolate.BPoly(np.array(yaw_coeffs).reshape(len(yaw_coeffs), 1),
                                      np.array([start_time, end_time]))
        retval.append(float(yaw_BPoly(eval_time)))
    return tuple(retval)


def get_derivative(txyz_yaw: List[List[float]]) -> List[List[float]]:
    '''Function that takes a list of coordinates with timestamps and calculates their derivatives. Returns a
    List[List[float]] of the same dimension as the input, where the first timestamps' derivatives are the same as the
    second for ease of plotting later.'''
    output = [txyz_yaw[0]]
    for xyz_yaw in txyz_yaw[1:]:
        derivative = list(np.diff(xyz_yaw) / np.diff(txyz_yaw[0]))
        output.append([derivative[0]] + derivative)
    return output


def assert_no_collisions(traj_eval: List[List[List[float]]], TIMESTEP) -> None:
    '''Function that asserts that for every drone, paired with every other drone, their timestamps are identical and
    their euclidean distance is smaller than 0.2, which is apprixmately the size of a drone.'''
    num_of_timestamps = len(traj_eval[0][0])
    num_of_drones = len(traj_eval)
    for i in range(num_of_drones):
        for j in range(i + 1, num_of_drones):
            for idx in range(num_of_timestamps):
                t1 = traj_eval[i][0][idx]
                t2 = traj_eval[j][0][idx]
                xyz1 = (traj_eval[i][1][idx], traj_eval[i][2][idx], traj_eval[i][3][idx])
                xyz2 = (traj_eval[j][1][idx], traj_eval[j][2][idx], traj_eval[j][3][idx])
                distance = np.sqrt((xyz1[0] - xyz2[0]) ** 2 + (xyz1[1] - xyz2[1]) ** 2 + (xyz1[2] - xyz2[2]) ** 2)
                # assert distance > 0.2
                if (distance < 0.2):
                    print(f"WARNING: DISTANCE < 0.2")
                assert abs(t1 - t2) < TIMESTEP / 50


class PausableAnimation:
    """Utility class for an animation that can be paused by pressing space."""

    def __init__(self, anim_data: List[List[List[float]]], drones: List[Poly3DCollection],
                 car: Optional[Poly3DCollection], car_anim_data: Optional[List[List[float]]],
                 fig: matplotlib.figure.Figure, frames: int, interval: int, repeat: bool, ax, L, H, P,
                 CAR_R=None, CAR_H=None, DEFAULT_ANGLE=0):
        self.anim_data = anim_data
        self.drones = drones
        self.car = car
        self.anim_data = anim_data
        self.car_anim_data = car_anim_data
        self.frames = frames
        self.interval = interval
        self.repeat = repeat
        self.paused = False
        self.text = ax.text2D(0, 0, "t=0.00s", transform=ax.transAxes, fontsize=10, color="black")
        self.animation = FuncAnimation(fig, self.update, frames=self.frames,
                                       interval=self.interval, repeat=self.repeat)
        fig.canvas.mpl_connect('key_press_event', self.toggle_pause)
        self.L = L
        self.H = H
        self.P = P
        self.CAR_R = CAR_R if self.car is not None else None
        self.CAR_H = CAR_H if self.car is not None else None
        self.DEFAULT_ANGLE = DEFAULT_ANGLE

    def update(self, frame_idx: int):
        """Function that updates the vertices of the drones according to the current frame's index."""
        self.text.set_text(f"t={self.anim_data[0][0][frame_idx]:.2f}s")
        for i, drone in enumerate(self.drones):
            pose = [float(lst[frame_idx]) for lst in self.anim_data[i][1:]]
            drone.set_verts(drone_pose_to_vertices(pose, self.L, self.H, self.P, self.DEFAULT_ANGLE))
        if self.car is not None:
            self.car.set_verts(car_position_to_vertices([lst[frame_idx] for lst in self.car_anim_data[1:]], self.CAR_R, self.CAR_H))

    def toggle_pause(self, event):
        if event.key == ' ':
            if self.paused:
                self.animation.resume()
            else:
                self.animation.pause()
            self.paused = not self.paused


def animate(limits: Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float]],
            fps: float, speed: float, timestep: float, traj_eval: List[List[List[float]]],
            car_eval: Optional[List[List[float]]], COLORS, L, H, P, DEFAULT_ANGLE, CAR_R, CAR_H) -> FuncAnimation:
    """Function that takes initializes the 3D plot for the animation, as well as the animation itself then returns the
    animation object."""
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.cla()
    ax.set_title("Press space to pause/resume animation.")
    ax.set_xlim(*limits[0])
    ax.set_ylim(*limits[1])
    ax.set_zlim(*limits[2])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    anim_interval = round(1000 / fps / speed)  # this is the delay between frames in msec
    # this line below seems scary, but all it does is takes every Nth element from the high resolution evaluation
    # results, where N is determined so that it fits the FPS requested
    anim_traj_eval = [[lst[::round(1 / timestep / fps)] for lst in drone] for drone in traj_eval]
    anim_length = len(anim_traj_eval[0][0])
    start_poses = [[float(lst[0]) for lst in drone[1:]] for drone in traj_eval]
    # we initialize the drones with their starting positions, and later update their vertices in the animation
    drones = [
        Poly3DCollection(drone_pose_to_vertices(start_pose, L, H, P, DEFAULT_ANGLE), facecolors=COLORS[idx % len(COLORS)], edgecolors='black')
        for
        idx, start_pose in enumerate(start_poses)]
    if car_eval is not None:
        car_start = [car_eval[1][0], car_eval[2][0]]
        car = Poly3DCollection(car_position_to_vertices(car_start, CAR_R, CAR_H), facecolors=COLORS[-1], edgecolors='black')
        car_anim_data = [lst[::round(1 / timestep / fps)] for lst in car_eval]
        ax.add_collection3d(car)
    else:
        car_anim_data = None
        car = None
    for drone in drones:
        ax.add_collection3d(drone)
    anim = PausableAnimation(anim_data=anim_traj_eval, drones=drones, car=car, car_anim_data=car_anim_data,
                             fig=fig, frames=anim_length, interval=anim_interval, repeat=True, ax=ax,
                             L=L, H=H, P=P, CAR_R=CAR_R, CAR_H=CAR_H, DEFAULT_ANGLE=DEFAULT_ANGLE)
    return anim.animation


def plot_data(traj_eval: List[List[List[float]]],
              first_deriv: List[List[List[float]]],
              second_deriv: List[List[List[float]]], traj_data) -> None:
    """Function that takes cares of the plotting. It has one figure, with a column of subplots for each drone. There
    will be 3 rows: pose/angle, their derivatives and their 2nd derivatives"""
    fig, subplots = plt.subplots(3, len(traj_eval))
    if len(traj_eval) == 1:  # double indexing wouldn't work when we only have 1 drone, hence this
        subplots = np.array(subplots).reshape((3, 1))
    for idx in range(len(traj_eval)):
        subplots[0, idx].plot(traj_eval[idx][0], traj_eval[idx][1], label='x')
        subplots[0, idx].plot(traj_eval[idx][0], traj_eval[idx][2], label='y')
        subplots[0, idx].plot(traj_eval[idx][0], traj_eval[idx][3], label='z')
        subplots[0, idx].set_title(f"Drone{idx} pose [m, degrees]", fontsize=10)
        subplots[0, idx].grid(True)
        subplots[0, idx].set_xlabel('t [s]')

        subplots[1, idx].plot(first_deriv[idx][0], first_deriv[idx][1], label='x')
        subplots[1, idx].plot(first_deriv[idx][0], first_deriv[idx][2], label='y')
        subplots[1, idx].plot(first_deriv[idx][0], first_deriv[idx][3], label='z')
        subplots[1, idx].set_title(f"Drone{idx}, 1st derivatives [m/s, deg/s]", fontsize=10)
        subplots[1, idx].grid(True)
        subplots[1, idx].set_xlabel('t [s]')

        subplots[2, idx].plot(second_deriv[idx][0], second_deriv[idx][1], label='x')
        subplots[2, idx].plot(second_deriv[idx][0], second_deriv[idx][2], label='y')
        subplots[2, idx].plot(second_deriv[idx][0], second_deriv[idx][3], label='z')
        subplots[2, idx].set_title(f"Drone{idx}, 2nd derivatives [m/s2, degrees/s2]", fontsize=10)
        subplots[2, idx].grid(True)
        subplots[2, idx].set_xlabel('t [s]')
        if all([traj.get("has_yaw", False) for traj in traj_data]):
            # when yaw also has to be plotted, it's sensible to plot it on a different y axis, since it's a different
            # unit of measurement from the rest
            twin_subplot = subplots[0, idx].twinx()
            twin_subplot.plot(traj_eval[idx][0], traj_eval[idx][4], label='yaw', color='r')
            # we need to combine the legends from the two y axes since they will be on the same subplot
            xyz_lines, xyz_labels = subplots[0, idx].get_legend_handles_labels()
            yaw_lines, yaw_labels = twin_subplot.get_legend_handles_labels()
            subplots[0, idx].legend(xyz_lines + yaw_lines, xyz_labels + yaw_labels, fontsize=10)

            twin_subplot = subplots[1, idx].twinx()
            twin_subplot.plot(first_deriv[idx][0], first_deriv[idx][4], label='yaw', color='r')
            # we need to combine the legends from the two y axes since they will be on the same subplot
            xyz_lines, xyz_labels = subplots[1, idx].get_legend_handles_labels()
            yaw_lines, yaw_labels = twin_subplot.get_legend_handles_labels()
            subplots[1, idx].legend(xyz_lines + yaw_lines, xyz_labels + yaw_labels, fontsize=10)

            twin_subplot = subplots[2, idx].twinx()
            twin_subplot.plot(second_deriv[idx][0], second_deriv[idx][4], label='yaw', color='r')
            # we need to combine the legends from the two y axes since they will be on the same subplot
            xyz_lines, xyz_labels = subplots[2, idx].get_legend_handles_labels()
            yaw_lines, yaw_labels = twin_subplot.get_legend_handles_labels()
            subplots[1, idx].legend(xyz_lines + yaw_lines, xyz_labels + yaw_labels, fontsize=10)
        else:
            subplots[0, idx].legend(fontsize=10)
            subplots[1, idx].legend(fontsize=10)
            subplots[2, idx].legend(fontsize=10)
    fig.subplots_adjust(hspace=0.4)

class BezierCurve:
    """
    Convenience class to encapsulate a single bezier curve (as opposed to a series of bezier curves like in a skyc file).
    """
    def __init__(self, points: list[float], start_time: float, end_time: float):
        self.points: list[float] = points  #: The control points of the curve (the coefficients).
        self.start_time: float = start_time  #: The start of the curve.
        self.end_time: float = end_time  #: The end of the curve.
        coeffs: list[tuple[float, ...]] = list(zip(*points))  #: This is equivalent to transposing the matrix formed by the points.
        self.BPolys: list[BPoly] = [BPoly(np.array(coeffs).reshape(len(coeffs), 1), np.array([start_time, end_time]))
                                    for coeffs in coeffs]  #: The interpolate.BPoly objects formed from the points.

    def x(self, time: float, nu: int = 0) -> np.ndarray:
        """
        Wrapper around the __call__ function of the appropriate BPoly object; it evaluates the x Bezier Polynomial.

        Args:
            time (float): The timestamp at which to evaluate.
            nu (int): The number of derivative to evaluate.

        Returns:
            np.ndarray: The "nu"th derivative at "time" time.
        """
        return self.BPolys[0](time, nu)

    def y(self, time: float, nu: int = 0) -> np.ndarray:
        """
        Wrapper around the __call__ function of the appropriate BPoly object; it evaluates the y Bezier Polynomial.

        Args:
            time (float): The timestamp at which to evaluate.
            nu (int): The number of derivative to evaluate.

        Returns:
            np.ndarray: The "nu"th derivative at "time" time.
        """
        return self.BPolys[1](time, nu)

    def z(self, time: float, nu: int = 0) -> np.ndarray:
        """
        Wrapper around the __call__ function of the appropriate BPoly object; it evaluates the z Bezier Polynomial.

        Args:
            time (float): The timestamp at which to evaluate.
            nu (int): The number of derivative to evaluate.

        Returns:
            np.ndarray: The "nu"th derivative at "time" time.
        """
        return self.BPolys[2](time, nu)

    def yaw(self, time: float, nu: int = 0) -> np.ndarray:
        """
        Wrapper around the __call__ function of the appropriate BPoly object; it evaluates the yaw Bezier Polynomial.

        Args:
            time (float): The timestamp at which to evaluate.
            nu (int): The number of derivative to evaluate.

        Returns:
            np.ndarray: The "nu"th derivative at "time" time.
        """
        return self.BPolys[3](time, nu)

class TrajEvaluator:
    def __init__(self, traj_data: dict):
        self.traj_data = traj_data  #: The dictionary that can be read from trajectory.json.
        self.bezier_curves: list[BezierCurve] = []  #: The list of individual segments.
        segments = self.traj_data.get("points")
        assert segments is not None
        for i in range(1, len(segments)):
            prev_segment = segments[i - 1]
            start_point = prev_segment[1]  # The current segment's start pose is the end pose of the previous one.
            start_time = prev_segment[0]  # The current segment starts when the previous ends.
            segment = segments[i]
            end_point = segment[1]
            end_time = segment[0]
            ctrl_points = segment[2]  # The "extra" control points, which aren't physically on the curve.
            # points will contain all points of the bezier curve, including the start and end, unlike in trajectory.json
            points = [start_point, *ctrl_points, end_point] if ctrl_points else [start_point, end_point]
            self.bezier_curves.append(BezierCurve(points, start_time, end_time))

    def select_curve(self, time: float) -> BezierCurve:
        """
        Calculates which Bezier segment the timestamp falls under.

        Args:
            time (float): The time at which we're investigating the trajectory.

        Returns:
            BezierCurve: The segment which will be traversed at the given timestamp.
        """
        if time < self.traj_data["takeoffTime"]:  # this should never happen, but in case it does we return the 0th
            return self.bezier_curves[0]
        # This loop breaks if we find a match, meaning that it relies on the segments being sorted in increasing
        # timestamp order.
        for bezier_curve in self.bezier_curves:
            if bezier_curve.start_time <= time <= bezier_curve.end_time:
                return bezier_curve
        return self.bezier_curves[-1]

    def evaluate(self, times: list[float]) -> list[list[float]]:
        '''Function that looks at which bezier curve each timestamp falls into, then evaluates the curve at that
        timestamp, and returns the result for each timestamp.
        The return will be a list of length 5, where each subelement is also a list.
        The first list is the timestamps.
        The second list is the x positions.
        The third is the y positions; etc.
        '''
        ret: list[list[float]] = []
        for time in times:
            curve = self.select_curve(time)
            ret.append([time, float(curve.x(time)), float(curve.y(time)), float(curve.z(time)), float(curve.yaw(time))])
        ret = [list(item) for item in zip(*ret)]
        return ret


def inspect(filename=None, car_file=None):
    if filename is None:
        SKYC_FILE: Union[str, None] = select_file("skyc")
    else:
        SKYC_FILE = filename
    if SKYC_FILE is not None:
        traj_data = get_traj_data(SKYC_FILE)  # this will be a list of the dictionaries in the trajectory.json files
    else:
        exit()
    if len(traj_data) < 10:
        LIMITS = ((-2, 2), (-2, 2), (-0.15, 1.6))  # physical constraints of the optitrack system
    else:
        LIMITS = ((-1, len(traj_data) * 0.5 + 1), (-1, len(traj_data) * 0.5 + 1), (-0.1, 2))  # TODO
    TIMESTEP = 0.005  # we keep this relatively constant for the sake of the animation coming later
    takeoff_time, land_time = extend_takeoff_land(traj_data)  # make every trajectory start and end at the same time

    evaluators = [TrajEvaluator(d) for d in traj_data]
    eval_times = list(np.linspace(takeoff_time, land_time, round((land_time - takeoff_time) / TIMESTEP)))
    traj_eval = [evaluator.evaluate(eval_times) for evaluator in evaluators]
    assert_no_collisions(traj_eval, TIMESTEP)
    # car_file = "/home/aimotion-i9/Projects/Palko_Demo_v3/car_logs/car_log.txt"
    car_eval = eval_if_car(eval_times, car_file) if car_file is not None else None
    if car_eval is not None:
        assert_no_car_collision(drones=traj_eval, car=car_eval, TIMESTEP=TIMESTEP)
    first_deriv = [get_derivative(item) for item in traj_eval]
    second_deriv = [get_derivative(item) for item in first_deriv]
    ANIM_FPS = 50
    ANIM_SPEED = 100  # this is the factor by which we speed the animation up in case it's slow due to calculations
    COLORS = ['r', 'b', 'g', 'y', 'c', 'm']
    DRONE_SCALE = 2
    DRONE_Z_SCALE = 0.3
    CAR_SCALE = 1
    CAR_H = 2 * CAR_SCALE
    CAR_R = 0.15 * CAR_SCALE
    L = 0.1 * DRONE_SCALE
    H = 0.05 * DRONE_SCALE * DRONE_Z_SCALE
    P = 0.04 * DRONE_SCALE
    DEFAULT_ANGLE = 0
    if len(traj_eval) <= 5:
        plot_data(traj_eval, first_deriv, second_deriv, traj_data)
    animation = animate(LIMITS, ANIM_FPS, ANIM_SPEED, TIMESTEP, traj_eval, car_eval, COLORS, L, H, P, DEFAULT_ANGLE, CAR_R, CAR_H)
    plt.show()


if __name__ == "__main__":
    inspect()
