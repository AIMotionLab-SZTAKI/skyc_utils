from pyledctrl.compiler.compiler import BytecodeCompiler
from pyledctrl.compiler.formats import InputFormat
from pyledctrl.compiler.formats import OutputFormat
from typing import ClassVar, Union
import json

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

    def export_json(self, write_file: bool = True) -> str:
        """
        Write the light program to a json file.
        """
        compiler = BytecodeCompiler()
        output = compiler.compile(input=self.source, input_format=InputFormat.LEDCTRL_SOURCE,
                                  output_format=OutputFormat.LEDCTRL_JSON)[0]
        json_dict = json.loads(output.decode('ascii'))
        json_dict["colors"] = self.colors
        json_object = json.dumps(json_dict, indent=2)
        if write_file:
            with open("trajectory.json", "w") as f:
                f.write(json_object)
        return json_object

DEFAULT_LIGHT_PROGRAM = LightProgram()
DEFAULT_LIGHT_PROGRAM.append_color(Color.BLACK, 600)