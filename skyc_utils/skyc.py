from skyc_utils.trajectory import Trajectory
from skyc_utils.light_program import LightProgram
import os
import shutil
from typing import Union, Optional
import json
import sys
import zipfile

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
    def __init__(self, lights: bool = False):
        self.has_lights = lights
        self.drones: list[Union[tuple[Trajectory], tuple[Trajectory, LightProgram]]] = []

    def add_drone(self, traj: Trajectory, light_program: Optional[LightProgram] = None) -> None:
        if self.has_lights:
            assert(light_program is not None), "Light Program required for Skyc file with has_lights=True!"
            self.drones.append((traj, light_program))
        else:
            assert(light_program is None), "Light Program not allowed for Skyc file with has_lights=False!"
            self.drones.append((traj,))

    @staticmethod
    def from_file(file: str) -> 'Skyc':
        pass

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