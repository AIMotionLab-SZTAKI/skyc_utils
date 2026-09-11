from skyc_utils.trajectory import Trajectory
from skyc_utils.light_program import LightProgram
import os
import shutil
from typing import Union, Optional
import json
import sys
import zipfile
import tempfile
import numpy as np
import matplotlib.pyplot as plt

def is_num(var):
    """There has to be a built-in for this...."""
    return isinstance(var, float) or isinstance(var, int)

def cleanup(files: list[str], folders: list[str]):
    """
    function meant for deleting unnecessary files
    """
    for file in files:
        if os.path.exists(file):
            os.remove(file)
            print(f"Deleted {file}")
    for folder in folders:
        if os.path.exists(folder):
            shutil.rmtree(folder)
            print(f"Deleted {folder} folder")

class Skyc:
    def __init__(self):
        self.has_lights = False
        self.drones: list[Union[tuple[Trajectory], tuple[Trajectory, LightProgram]]] = []

    def add_drone(self, traj: Trajectory, light_program: Optional[LightProgram] = None) -> None:
        if len(self.drones) == 0:
            self.has_lights = light_program is not None
        if self.has_lights:
            assert(light_program is not None), "Light Program required for Skyc file with has_lights=True!"
            self.drones.append((traj, light_program))
        else:
            assert(light_program is None), "Light Program not allowed for Skyc file with has_lights=False!"
            self.drones.append((traj,))

    def write(self, name: str = sys.argv[0][:-3]) -> None:
        cleanup(files=["show.json",
                       "cues.json",
                       f"{name}.zip",
                       f"{name}.skyc",
                       "trajectory.json"
                       "lights.json"],
                folders=["drones"])
        # Create the 'drones' folder if it doesn't already exist
        os.makedirs('drones', exist_ok=True)
        drones = []
        for index, drone in enumerate(self.drones):
            traj = drone[0]
            traj.export_json(write_file=True)
            data = traj.bezier
            parameters = traj.parameters
            # The trajectory is saved to a json file with the data below
            drone_settings = {
                "trajectory": {"$ref": f"./drones/drone_{index}/trajectory.json#"},
                "home": data[0][1][0:3],
                "startYaw": data[0][1][-1],
                "landAt": data[-1][1][0:3],
                "name": f"drone_{index}",
            }
            if parameters is not None and len(parameters) > 0:
                for parameter in parameters:
                    assert is_num(parameter[0]) and isinstance(parameter[1], str) and is_num(parameter[2])
                drone_settings["parameters"] = parameters
            drones.append({
                "type": "generic",
                "settings": drone_settings
            })
            # Create the 'drone_x' folder if it doesn't already exist
            drone_folder = os.path.join('drones', f'drone_{index}')
            os.makedirs(drone_folder, exist_ok=True)
            shutil.move('trajectory.json', drone_folder)
            if self.has_lights:
                light_program = drone[1]
                light_program.export_json()
                drone_settings["lights"] =  {"$ref": f"./drones/drone_{index}/lights.json#"}
                shutil.move('lights.json', drone_folder)

        # This wall of text below is just overhead that is required to make a skyc file.
        ########################################CUES.JSON########################################
        items = [{"time": 0.0,
                  "name": "start"}]
        cues = {
            "version": 1,
            "items": items
        }
        json_object = json.dumps(cues, indent=2)
        with open("cues.json", "w") as f:
            f.write(json_object)
        #######################################SHOW.JSON###########################################
        validation = {
            "maxAltitude": 2.0,
            "maxVelocityXY": 2.0,
            "maxVelocityZ": 1.5,
            "minDistance": 0.8
        }
        cues = {
            "$ref": "./cues.json"
        }
        settings = {
            "cues": cues,
            "validation": validation
        }
        meta = {
            "id": f"{name}.py",
            "inputs": [f"{name}.py"]
        }
        show = {
            "version": 1,
            "settings": settings,
            "swarm": {"drones": drones},
            "environment": {"type": "indoor"},
            "meta": meta,
            "media": {}
        }
        json_object = json.dumps(show, indent=2)
        with open("show.json", "w") as f:
            f.write(json_object)

        # Create a new zip file
        with zipfile.ZipFile(f"{name}.zip", 'w', zipfile.ZIP_DEFLATED) as zipf:
            # Add the first file to the zip
            zipf.write("show.json")

            # Add the second file to the zip
            zipf.write("cues.json")

            # Recursively add files from the specified folder and its sub-folders
            for root, _, files in os.walk("drones"):
                for file in files:
                    file_path = os.path.join(root, file)
                    zipf.write(file_path)

        print('Compression complete. The files and folder have been zipped.')

        os.rename(f'{name}.zip', f'{name}.skyc')
        # Delete everything that's not 'trajectory.skyc'
        cleanup(files=["show.json",
                       "cues.json",
                       f"{name}.zip",
                       "trajectory.json",
                       "lights.json"],
                folders=["drones"])
        print(f"{name}.skyc ready!")

    @staticmethod
    def from_file(file: str) -> 'Skyc':
        def _read_json_from_zip(zf: zipfile.ZipFile, inner_path: str) -> tuple[str, dict]:
            # inner_path may be like "./drones/drone_0/trajectory.json#"
            path = inner_path.split('#', 1)[0].lstrip('./')
            try:
                raw = zf.read(path)
            except KeyError as e:
                raise FileNotFoundError(f"Missing file in archive: {path}") from e
            return path, json.loads(raw.decode('utf-8'))

        with zipfile.ZipFile(file, 'r') as zf:
            # --- show.json ---
            try:
                show_raw = zf.read('show.json')
            except KeyError as e:
                raise FileNotFoundError("show.json not found in the skyc archive") from e
            show = json.loads(show_raw.decode('utf-8'))

            drones_meta = show.get('swarm', {}).get('drones', [])
            if not isinstance(drones_meta, list) or not drones_meta:
                raise ValueError("show.json has no swarm/drones entries.")

            has_lights = any('lights' in d.get('settings', {}) for d in drones_meta)
            skyc = Skyc()

            # Rebuild each drone
            for d in drones_meta:
                settings = d.get('settings', {})
                traj_ref = settings.get('trajectory', {}).get('$ref')
                if not traj_ref:
                    raise ValueError("A drone in show.json is missing its trajectory $ref.")

                # Read trajectory.json blob -> temp file -> Trajectory.from_json(path)
                traj_path_in_zip, traj_json = _read_json_from_zip(zf, traj_ref)
                with tempfile.NamedTemporaryFile(suffix='.json', delete=False) as tmp:
                    tmp.write(json.dumps(traj_json).encode('utf-8'))
                    traj_tmp_path = tmp.name
                try:
                    traj = Trajectory.from_json(traj_tmp_path)
                finally:
                    os.remove(traj_tmp_path)

                # Copy parameters from show.json into the trajectory, if present
                params = settings.get('parameters', [])
                if params:
                    if not isinstance(params, list):
                        raise ValueError("'parameters' must be a list.")
                    for entry in params:
                        if (not isinstance(entry, (list, tuple))) or len(entry) != 3:
                            raise ValueError("Each parameter must be [time, name, value].")
                        t, pname, val = entry
                        if not (is_num(t) and isinstance(pname, str) and is_num(val)):
                            raise ValueError("Parameter entry types must be [number, str, number].")
                        traj.add_parameter(float(t), pname, float(val))

                # Optional lights
                if has_lights:
                    lights_ref = settings.get('lights', {}).get('$ref')
                    if not lights_ref:
                        raise ValueError("Skyc has lights=True but a drone is missing lights.json $ref.")
                    _, lights_json = _read_json_from_zip(zf, lights_ref)
                    with tempfile.NamedTemporaryFile(suffix='.json', delete=False) as tmp:
                        tmp.write(json.dumps(lights_json).encode('utf-8'))
                        lights_tmp_path = tmp.name
                    try:
                        light_prog = LightProgram.from_json(lights_tmp_path)
                    finally:
                        os.remove(lights_tmp_path)

                    skyc.add_drone(traj, light_prog)
                else:
                    skyc.add_drone(traj)

        return skyc

def _eval_traj_dense(traj, n: int = 1500):
    """Return t, (x,y,z,yaw), (vx,vy,vz,vyaw), (ax,ay,az,ayaw)."""
    T = float(traj.duration)
    t = np.linspace(0.0, T, n) if T > 0 else np.array([0.0], dtype=float)

    px = []; py = []; pz = []; pyaw = []
    vx = []; vy = []; vz = []; vyaw = []
    ax = []; ay = []; az = []; ayaw = []

    for ti in t:
        fs = traj.evaluate(float(ti))
        px.append(fs.pose.x);   py.append(fs.pose.y);   pz.append(fs.pose.z);   pyaw.append(fs.pose.yaw)
        vx.append(fs.vel.x);    vy.append(fs.vel.y);    vz.append(fs.vel.z);    vyaw.append(fs.vel.yaw)
        ax.append(fs.acc.x);    ay.append(fs.acc.y);    az.append(fs.acc.z);    ayaw.append(fs.acc.yaw)

    to_np = lambda lst: np.array(lst, dtype=float)
    return (
        t,
        (to_np(px), to_np(py), to_np(pz), to_np(pyaw)),
        (to_np(vx), to_np(vy), to_np(vz), to_np(vyaw)),
        (to_np(ax), to_np(ay), to_np(az), to_np(ayaw)),
    )

def plot_skyc_trajectories(skyc_path: str) -> None:
    """
    Load a .skyc file, reconstruct all drones' trajectories, and plot:
      - Pose (x,y,z,yaw)
      - Velocity (vx,vy,vz,vyaw)
      - Acceleration (ax,ay,az,ayaw)
    in separate windows per drone.
    """
    skyc = Skyc.from_file(skyc_path)
    if not skyc.drones:
        print("No drones found in the file.")
        return

    for i, drone in enumerate(skyc.drones):
        traj = drone[0]
        t, pose, vel, acc = _eval_traj_dense(traj, n=1800)
        x, y, z, yaw = pose
        vx, vy, vz, vyaw = vel
        ax, ay, az, ayaw = acc

        fig, axs = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
        fig.suptitle(f"Drone {i}: Trajectory")

        # Pose
        axs[0].plot(t, x,  label="x [m]")
        axs[0].plot(t, y,  label="y [m]")
        axs[0].plot(t, z,  label="z [m]")
        axs[0].plot(t, yaw, linestyle="--", alpha=0.8, label="yaw [rad]")
        axs[0].set_ylabel("Pose")
        axs[0].grid(True)
        axs[0].legend(loc="upper right")

        # Velocity
        axs[1].plot(t, vx, label="vx [m/s]")
        axs[1].plot(t, vy, label="vy [m/s]")
        axs[1].plot(t, vz, label="vz [m/s]")
        axs[1].plot(t, vyaw, linestyle="--", alpha=0.8, label="yaẇ [rad/s]")
        axs[1].set_ylabel("Velocity")
        axs[1].grid(True)
        axs[1].legend(loc="upper right")

        # Acceleration
        axs[2].plot(t, ax, label="ax [m/s²]")
        axs[2].plot(t, ay, label="ay [m/s²]")
        axs[2].plot(t, az, label="az [m/s²]")
        axs[2].plot(t, ayaw, linestyle="--", alpha=0.8, label="yaẅ [rad/s²]")
        axs[2].set_ylabel("Acceleration")
        axs[2].set_xlabel("time [s]")
        axs[2].grid(True)
        axs[2].legend(loc="upper right")

        # Mark segment boundaries if available
        if traj.polynomial is not None:
            knots = traj.polynomial.x.x  # shared across axes by construction
            for axp in axs:
                for k in knots:
                    axp.axvline(float(k), color="k", alpha=0.15, linewidth=1)

        fig.tight_layout()

    plt.show()
