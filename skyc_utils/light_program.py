from dataclasses import dataclass

from pyledctrl.compiler.compiler import BytecodeCompiler
from pyledctrl.compiler.formats import InputFormat
from pyledctrl.compiler.formats import OutputFormat
from pyledctrl.parsers.bytecode import BytecodeParser
from typing import ClassVar, Union, Optional
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

    def __eq__(self, other):
        return self.r == other.r and self.g == other.g and self.b == other.b

    def as_list(self) -> list[int]: return [self.r, self.g, self.b]

Color.BLACK = Color(0, 0, 0)
Color.RED = Color(255, 0, 0)
Color.GREEN = Color(0, 255, 0)
Color.BLUE = Color(0, 0, 255)
Color.YELLOW = Color(255, 255, 0)
Color.CYAN = Color(0, 255, 255)
Color.MAGENTA = Color(255, 0, 255)
Color.WHITE = Color(255, 255, 255)

DEFAULT_COLOR = Color.BLACK

class LightProgram:
    """
    A light program can be described in a .led file, which is basically source code from which a BytecodeCompiler
    in the pyledctrl package can compile a light program byte string. This is the format in which skyc files expect
    light programs to be.
    """
    def __init__(self):
        self.colors: list[tuple[float, Color]] = []

    def set_color(self, t: float, color: Color):
        self.colors.append((t, color))
        self.colors.sort(key=lambda c: c[0])

    @property
    def source(self):
        source = ""
        color = DEFAULT_COLOR
        t = 0.0
        for t_next, new_color in self.colors:
            duration = t_next - t
            source += f"set_color({color}, duration={duration:.3f})\n"
            t = t_next
            color = new_color
        source += f"set_color({color}, duration=0)\nend()\n"
        return bytes(source, "utf-8")


    def export_json(self, write_file: bool = True) -> str:
        """
        Write the light program to a json file.
        """
        compiler = BytecodeCompiler()
        output = compiler.compile(input=self.source, input_format=InputFormat.LEDCTRL_SOURCE,
                                  output_format=OutputFormat.LEDCTRL_JSON)[0]
        json_dict = json.loads(output.decode('ascii'))
        colors_serializable = [(t, c.as_list()) for t, c in self.colors]
        json_dict["colors"] = colors_serializable
        json_object = json.dumps(json_dict, indent=2)
        if write_file:
            with open("lights.json", "w") as f:
                f.write(json_object)
        return json_object


    @staticmethod
    def from_json(file: str) -> 'LightProgram':
        lights = LightProgram()
        with open(file, "r") as f:
            data = json.load(f)
        colors = data["colors"]
        lights.colors = [(t, Color(*c)) for t, c in colors]
        return lights