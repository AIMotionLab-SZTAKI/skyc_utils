import json
import math
import matplotlib.pyplot as plt
from scipy import interpolate as interpolate  # !
from scipy.interpolate import PPoly
import sys
import numpy as np  # !
import os
import shutil
import zipfile
from typing import List, Optional, Any, Union, Sequence, Tuple, ClassVar
import pickle
from skyc_utils.utils import *
from enum import Enum
from pyledctrl.compiler.compiler import BytecodeCompiler
from pyledctrl.compiler.formats import InputFormat
from pyledctrl.compiler.formats import OutputFormat

class Color:
    """
    Class to describe an RGB color. The standard RGB and CMY and White/Black colors are provided as class variables.
    """
    BLACK: ClassVar["Color"]
    RED: ClassVar["Color"]
    GREEN: ClassVar["Color"]
    BLUE: ClassVar["Color"]
    YELLOW: ClassVar["Color"]
    CYAN: ClassVar["Color"]
    MAGENTA: ClassVar["Color"]
    WHITE: ClassVar["Color"]

    def __init__(self, r: int, g: int, b: int):
        if not (0 <= r <= 255 and 0 <= g <= 255 and 0 <= b <= 255):
            raise ValueError("r, g, and b must be in the range 0-255")
        self.r: int = r
        self.g: int = g
        self.b: int = b

    def __repr__(self):
        return f"{self.r}, {self.g}, {self.b}"

Color.BLACK = Color(0, 0, 0)
Color.RED = Color(255, 0, 0)
Color.GREEN = Color(0, 255, 0)
Color.BLUE = Color(0, 0, 255)
Color.YELLOW = Color(255, 255, 0)
Color.CYAN = Color(0, 255, 255)
Color.MAGENTA = Color(255, 0, 255)
Color.WHITE = Color(255, 255, 255)

class LightProgram:
    """
    A light program can be described in a .led file, which is basically source code from which a BytecodeCompiler
    in the pyledctrl package can compile a light program byte string. This is the format in which skyc files expect
    light programs to be.
    """
    def __init__(self):
        self.source: bytes = b""
        self.colors: list[list[Union[str, float]]] = []

    def append_color(self, color: Color, duration: float):
        """
        Append a color for a duration to the light program.
        """
        self.source = self.source + bytes(f"set_color({color}, duration={duration})\n", "utf-8")
        self.colors.append([color.__repr__(), duration])

    def export_json(self):
        """
        Write the light program to a json file.
        """
        compiler = BytecodeCompiler()
        output = compiler.compile(input=self.source, input_format=InputFormat.LEDCTRL_SOURCE,
                                  output_format=OutputFormat.LEDCTRL_JSON)[0]
        json_dict = json.loads(output.decode('ascii'))
        json_dict["colors"] = self.colors
        json_object = json.dumps(json_dict, indent=2)
        with open("lights.json", "w") as f:
            f.write(json_object)



DEFAULT_LIGHT_PROGRAM = LightProgram()
DEFAULT_LIGHT_PROGRAM.append_color(Color.BLACK, 600)


class XYZYaw:
    """
    A way to encapsulate data that has an x, y, z and yaw dimension. For example, we may store a spline for each.
    Using this class, we can reach these both by indexing, iterating and directly addressing like .x, .y, etc.
    An XYZYaw is also iterable, therefore *obj is fine, and also, obj[0]=obj.x, obj[3]=obj.yaw, etc.
    You can initialize an XYZYaw object with:
    -keyword arguments: XYZYaw(x=.., y=..., z=..., yaw=...)
    -any iterable of length 4: XYZYaw([0, 0, 0, 0]) # TODO: with another XYZYaw?
    -four values of the same type (float/int are compatible here): XYZYaw(0, 0, 0, 0)
    """
    def __init__(self, *args, **kwargs):
        if args and kwargs:
            raise ValueError  # cannot have both keyword and positional arguments like this: XYZYaw(0, 0, 0, yaw=0)
        # if we used keyword argumens, we must take exactly the following ones: x, y, z and yaw
        if len(kwargs) == 4 and all(key in kwargs for key in ('x', 'y', 'z', 'yaw')):
            self.data = [kwargs['x'], kwargs['y'], kwargs['z'], kwargs['yaw']]
        elif len(args) == 4:  # if we took positional arguments, we may have four, corresponding to x, y, z, yaw
            self.data = args
        elif len(args) == 1:  # or we took exactly one, that is of length 4 (an iterable or sequence)
            if len(args[0]) == 4:
                self.data = args[0]
            else:
                raise ValueError
        else:
            raise ValueError
        # if X is a number, the others must be numbers as well
        if is_num(self.x):
            if not all(is_num(e) for e in self.data[1:]):
                raise ValueError
        else:  # if X is anything else other than a number, the rest must also be exactly that type
            expected_type = type(self.x)
            if not all(isinstance(e, expected_type) for e in self.data[1:]):
                raise ValueError

    @property
    def x(self):
        return self.data[0]

    @property
    def y(self):
        return self.data[1]

    @property
    def z(self):
        return self.data[2]

    @property
    def yaw(self):
        return self.data[3]

    def __getitem__(self, key):
        if isinstance(key, int) and key <= 3:
            return self.data[key]
        elif isinstance(key, slice):
            return self.data[key]
        else:
            return None

    def __len__(self):
        return 4

    def __iter__(self):
        for attr in (self.x, self.y, self.z, self.yaw):
            yield attr


class TrajectoryType(Enum):
    POLY4D = "POLY4D"
    COMPRESSED = "COMPRESSED"


class Trajectory:
    """
    A class representing all the information regarding a trajectory.
    """
    def __init__(self, traj_type: TrajectoryType = TrajectoryType.COMPRESSED, degree=3):
        self.parameters = []  # parameters that we set during flight
        # www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/trajectory_formats/
        self.type = traj_type
        if self.type == TrajectoryType.COMPRESSED:
            assert degree in [1, 3, 7]
        else:
            assert degree <= 7
        self.degree = degree
        self.start: XYZYaw = XYZYaw(0, 0, 0, 0)
        # most functions build self.ppoly, which carries all the data regarding the trajectory. It's an XYZYaw object,
        # with each dimension being an interpolate.PPoly object
        self.ppoly: Optional[XYZYaw] = None
        # regardless of whether the trajectory gets interpreted as poly4d or compressed, it will be packaged as bezier
        # segments in the skyc file for reasons discussed in the wiki of the skybrush server:
        # github.com/AIMotionLab-SZTAKI/skybrush-server/wiki/Changes-from-stock-Skybrush#adding-poly4d-trajectory-representation
        # to this end, we may have a bezier representation of the curve, which is calculated directly from the ppoly
        # representation by the function self.set_bezier_repr
        self.bezier_repr: Optional[List] = None

        self.lqr_params: dict[str, List[Union[float, int]]] = {}

    @property
    def end_condition(self) -> Tuple[XYZYaw, XYZYaw]:
        """
        Returns the end conditions at the first and last timestamp in the current trajectory, up to the 3rd derivative.
        These will be returned in a tuple (0=beginning, 1=end) of XYZYaw objects, where each dimension is of length 3,
        representing derivatives for that dimension (0th is position). The reason we return only position, velocity,
        acceleration and jerk is that the curves can only have up to degree 7, meaning 8 coefficients, making it
        possible to provide 4-4 end conditions.
        """
        if self.ppoly is None:
            # We suppose that the starting derivatives are 0. Technically this doesn't need to be the case, but this
            # is such a rare use case that it's fine. UNLESS... TODO
            start = XYZYaw([[pos, 0.0, 0.0, 0.0] for pos in self.start])
            end = XYZYaw([[pos, 0.0, 0.0, 0.0] for pos in self.start])
        else:
            start = XYZYaw([[ppoly(ppoly.x[0], nu=k) for k in range(4)] for ppoly in self.ppoly])
            end = XYZYaw([[ppoly(ppoly.x[-1], nu=k) for k in range(4)] for ppoly in self.ppoly])
        return start, end

    def set_start(self, start: XYZYaw):
        """
        Sets the starting point of the bezier representation, in case we want it different from 0, 0, 0, 0. This is
        necessary, since when we do go_to segments, meaning we must have somewhere to go *from* in order to go *to*
        somewhere. Self.start is presumed to be 0, 0, 0, 0 if not set.
        """
        assert isinstance(start.x, (int, float, np.number)) and isinstance(start.y, (int, float, np.number)) and \
               isinstance(start.z, (int, float, np.number)) and isinstance(start.yaw, (int, float, np.number))
        self.start = start

    def set_bezier_repr(self):
        """
        Constructs the bezier representation of the curve, in a state that's ready to be immediately written to a
        json object in a skyc file: a list, where each element is:
        [-a timestamp (arrive at this time!)
        -a point (arrive to this point at that time!)
        -a potentially empty list of auxiliary points, which are the inner points of each Bezier curve]
        This way, each element in the list corresponds to a Bezier curve, the first point of which is the arrival
        point of the last segment, the middle points are the auxiliary points, and the last point is the arrival
        point of the current segment.
        """
        assert self.ppoly is not None
        start = XYZYaw([float(ppoly(0.0)) for ppoly in self.ppoly])
        bezier_repr = [[0.0, [start.x, start.y, start.z, start.yaw], []]]
        bpolys = XYZYaw(*[interpolate.BPoly.from_power_basis(ppoly) for ppoly in self.ppoly])
        # These two lines below seem complicated but all they do is pack the data above into a convenient form: a list
        # of lists where each element looks like this: [t, (x,y,z), (x,y,z), (x,y,z)].
        bpoly_pts = list(zip(list(bpolys.x.x)[1:], *[list(bpoly.c.transpose()) for bpoly in bpolys]))
        # at this point bpoly_pts contains the control points for the segments, but that's not exactly what we need in
        # the skyc file: we need the last point, and the inside points
        bezier_curves = [[element[0]] + list(zip(*list(element[1:]))) for element in bpoly_pts]
        for bezier_curve in bezier_curves:
            curve_to_append = [bezier_curve[0],
                               bezier_curve[-1],
                               bezier_curve[2:-1]]
            bezier_repr.append(curve_to_append)
        self.bezier_repr = bezier_repr

    def add_goto(self, goto: XYZYaw, dt: float, continuity: int = 2):
        """
        Modifies self.ppoly to extend it with a smooth goto segment.
        goto: the setpoint of pos/vel/acc/jerk/.. to arrive with
        dt: the duration of the goto
        continuity: how many derivatives to be smooth in during the connection.
        1 = smooth position (required), 2 = smooth velocity, 3 = smooth acc, 4 = smooth jerk.
        Higher isn't possible, limited by the degree of the trajectory.
        XYZYaw may contain derivatives if we want to arrive with a certain speed/acceleration. This is different
        from continuity: if we give only an x-y-z yaw goto (no vel/acc), but we provide a continuity of 3, it is assumed
        that we want the arrival to be with velocity/acceleration of 0. If more derivatives are given as a setpoint
        than the continuity would allow for, the higher order derivatives are ignored. In addition, the degree of the
        trajectory may limit the curve: if it is of degree 3, the solver will have 4 parameters, allowing for 4 boundary
        conditions: 2 at each end (pos and vel), and a continuity of 1. The max continuity at degree 7 is accordingly 3.
        TODO: With how we handle the trajectory now, we could add a similar command to add segments before the traj.
        """
        free_params = self.degree + 1  # how many coefficients the degree allows for
        # how many derivatives we can guarantee continuity in, given the number of parameters
        max_continuity = math.floor(free_params / 2)
        continuity = min(continuity, max_continuity)
        _, goto_start_derivs = self.end_condition  # discard the beginning derivatives
        # make the derivatives at the start and end of the segment XYZYaw objects consisting of lists so that we don't
        # have awkward type checkings (is it a float? can it be indexed? does it have a length? etc.)
        goto_start_derivs = XYZYaw([as_list(thing) for thing in goto_start_derivs])
        goto = XYZYaw([as_list(thing) for thing in goto])
        # make sure the trajectory turns the shorter way around (when going from 0 to 270, go from 0 to -90 instead)
        goto.yaw[0] = determine_shorter_deg(goto_start_derivs.yaw[0], goto.yaw[0])
        goto_coeffs = []
        for derivs_end, derivs_start in zip(goto, goto_start_derivs):
            while len(derivs_end) < continuity:
                derivs_end += [0.0]  # if continuity allows for more derivatives than provided, fill them with 0s
            goto_end = derivs_end[:continuity]  # only consider the appropriate amount of derivatives
            goto_start = derivs_start[:continuity]
            goto_coeffs.append(calculate_polynomial_coefficients(goto_start, goto_end, 0, dt))
        # calculate_polynomial_coefficients returns a list of coeffs, in ascending order of degree. In a PPoly object,
        # they must be an ndarray of (deg+1, number_of_knots), in descending order of degree.
        goto_coeffs = [np.flip(coeffs.reshape(len(coeffs), 1)) for coeffs in goto_coeffs]
        if self.ppoly is None:
            # we had no previous ppolys: add them now
            self.ppoly = XYZYaw([PPoly(coeffs, np.array([0, dt])) for coeffs in goto_coeffs])
            # and then extend their dimension to match the degree of the curve (filling the higher coeffs with 0s)
            for ppoly in self.ppoly:
                extend_ppoly_coeffs(ppoly, self.degree + 1)
        else:
            # we had previous ppolys:
            for coeffs, ppoly in zip(goto_coeffs, self.ppoly):
                # a new polynomial breakpoint is appended to the end, as the last breakpoint + the goto duration
                ppoly.x = np.hstack((ppoly.x, ppoly.x[-1] + dt))
                # if there are too few coefficients, fill the higher ones with 0s again, and append them to
                # the end of the coefficients, corresponding to the new breakpoint
                coeffs_extended = np.vstack((np.zeros((self.degree+1 - len(coeffs), 1)), coeffs))
                ppoly.c = np.hstack((ppoly.c, coeffs_extended))

    def add_interpolated_traj(self, t_x_y_z_yaw: List[List[float]], number_of_segments: int, *,
                              method: str, fit_ends: bool = False, force_0_derivs: bool = False,
                              equidistant_knots: bool = False):
        """
        Add an interpolated segment to the trajectory, fitting to the t_x_y_z_yaw data with a given method, consisting
        of the given number of segments. You may additionally choose to 'fit ends', meaning to try to match the start
        and end of the points exactly, as well as force their derivatives to be 0.
        """
        assert 1 <= self.degree <= 7
        if self.type == TrajectoryType.POLY4D:
            assert number_of_segments < 60
        ppoly_lst = fit_ppoly_to_data(t_x_y_z_yaw, number_of_segments, self.degree, method=method, fit_ends=fit_ends,
                                      force_0_derivs=force_0_derivs, equidistant_knots=equidistant_knots)
        self.add_ppoly(XYZYaw(ppoly_lst))

    def add_ppoly(self, ppoly_lst: XYZYaw):
        """
        Appends the given ppoly to the end of the existing ppoly, by extending the breakpoints and the coefficients.
        """
        if self.ppoly is None:
            self.ppoly = ppoly_lst
        else:
            for ppoly, new_ppoly in zip(self.ppoly, ppoly_lst):
                ppoly.x = np.hstack((ppoly.x, new_ppoly.x[1:] + ppoly.x[-1]))
                ppoly.c = np.hstack((ppoly.c, new_ppoly.c))

    def export_json(self, write_file: bool = True) -> str:
        """
        Returns the json formatted string of the bezier representation, and also writes it to a file if we wish.
        """
        self.set_bezier_repr()
        # this is the format that a TrajectorySpecification requires:
        json_dict = {
            "version": 1,
            "points": self.bezier_repr,
            "takeoffTime": self.bezier_repr[0][0],
            "landingTime": self.bezier_repr[-1][0],
            "type": self.type.value
        }
        json_dict = json_dict | self.lqr_params
        json_object = json.dumps(json_dict, indent=2)
        if write_file:
            with open("trajectory.json", "w") as f:
                f.write(json_object)
        return json_object

    def add_parameter(self, t: Union[int, float], param: str, value: Union[int, float]):
        """Using this function instead of directly setting trajectory.parameters ensures that we don't mess up by
        writing the parameters in traj.parameters in the wrong order (such as parameter, value, time)."""
        self.parameters.append([t, param, value])

    def add_lqr_params(self, lqr_params: List[int], bounds: List[float], LQR_N=240):
        assert len(lqr_params) == LQR_N * 64
        assert len(bounds) == 128
        self.lqr_params["K"] = lqr_params
        self.lqr_params["bounds"] = bounds


def write_skyc(trajectories: list[Trajectory], light_programs: Optional[list[LightProgram]] = None,
               name=sys.argv[0][:-3]):
    """
    Constructs a skyc file from the provided trajectory, and optionally given lights with the given name.
    """
    cleanup(files=["show.json",
                   "cues.json",
                   f"{name}.zip",
                   f"{name}.skyc",
                   "trajectory.json"
                   "lights.json"],
            folders=["drones"])
    if not (light_programs is None or len(light_programs) == len(trajectories)):
        raise ValueError("If light programs are provided, their number must equal the number of trajectories!")
    # Create the 'drones' folder if it doesn't already exist
    os.makedirs('drones', exist_ok=True)
    drones = []
    for index, traj in enumerate(trajectories):
        traj.export_json()
        if light_programs is None: # if no light program was provided, use a default black program
            DEFAULT_LIGHT_PROGRAM.export_json()
        else:
            light_programs[index].export_json()
        assert traj.bezier_repr is not None
        Data = traj.bezier_repr
        parameters = traj.parameters
        # The trajectory is saved to a json file with the data below
        drone_settings = {
            "trajectory": {"$ref": f"./drones/drone_{index}/trajectory.json#"},
            "lights": {"$ref": f"./drones/drone_{index}/lights.json#"},
            "home": Data[0][1][0:3],
            "startYaw": Data[0][1][-1],
            "landAt": Data[-1][1][0:3],
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


def evaluate_pickle(pickle_name: str, *, x_offset=0.0, y_offset=0.0, z_offset=0.0, yaw_offset=0.0, granularity=100):
    """
    Helper function to evaluate pickles made by Dr. Prof. PhD Antal Péter
    """
    with open(pickle_name, "rb") as file:
        data = pickle.load(file)
        parameters = data.get("parameters", None)  #maybe parameters = data[1]
        segments = data["traj"]  # maybe segments = data[0]
        contains_yaw = len(segments) == 4
        # unpack the data so that the outer index is the slice index, and the inner index switches between x-y-z-yaw
        segments = list(zip(*segments))
        x, y, z, t, yaw = [], [], [], [], []
        for segment in segments:
            # for each segment, segment[0] is the spline for x, [1] is for y, [2] is for z, [3] is for yaw. Within
            # those, [0] is the knot vector, [1] is the coeffs, [2] is the degree of the spline
            # technically, in the pickle, x, y, z and yaw could have different knots, but for practical reasons, they
            # obviously don't. We make use of this fact here to simplify the code, however, if the pickle's composition
            # was changed, then this would have to change as well. TODO: discuss this with Peti
            # also, exclude [0] to avoid duplicate timestamps
            eval_time = np.linspace(segment[0][0][0], segment[0][0][-1], granularity+1)[1:]
            x = x + list(interpolate.splev(eval_time, segment[0]))
            y = y + list(interpolate.splev(eval_time, segment[1]))
            z = z + list(interpolate.splev(eval_time, segment[2]))
            yaw = yaw + list(interpolate.splev(eval_time, segment[3])) if contains_yaw else None
            eval_time = eval_time + t[-1] if len(t) > 1 else eval_time  # shift relative time to absolute
            t = t + list(eval_time)
        x = [element + x_offset for element in x]
        y = [element + y_offset for element in y]
        z = [element + z_offset for element in z]
        yaw = [np.rad2deg(element) + yaw_offset for element in yaw] if contains_yaw else None
    return ((t, x, y, z, yaw), parameters) if contains_yaw else ((t, x, y, z), parameters)
