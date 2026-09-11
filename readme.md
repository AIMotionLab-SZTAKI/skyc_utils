# skyc_utils

`skyc_utils` is a python package for building `.skyc` "show" files: the format that [Skybrush](https://skybrush.io/) uses do describe a drone show. It can also read `.skyc` files back, and plot them. If you want an animation rather than a plot, [AIMotionLab-Virtual](https://github.com/AIMotionLab-SZTAKI/AIMotionLab-Virtual) can help you.


A show file made here travels like this:

```
                                   ┌──> plot_skyc_trajectories()      (sanity check, matplotlib)
                                   │
your_script.py ──skyc_utils──> show.skyc ──> Skybrush Live ──> skybrush-server ──> Crazyradio ──> drone
                                   │
                                   └──> AIMotionLab-Virtual           (simulation/animation)
```

Everything downstream of the `.skyc` file can be found in other repositories:

| Repo | Role |
| --- | --- |
| [AIMotionLab-SZTAKI/skybrush-server](https://github.com/AIMotionLab-SZTAKI/skybrush-server) | Loads the show (via Skybrush Live), encodes each trajectory for the firmware and uploads it over the Crazyradio. **Read its readme for how to actually fly a show.** |
| [AIMotionLab-SZTAKI/crazyflie-firmware](https://github.com/AIMotionLab-SZTAKI/crazyflie-firmware) | Runs the trajectory on the drone. Defines the two trajectory formats (`POLY4D`, `COMPRESSED`) and their limits. |
| [AIMotionLab-Virtual](https://github.com/AIMotionLab-SZTAKI/AIMotionLab-Virtual) | Simulates a `.skyc` show in MuJoCo. It imports `skyc_utils.trajectory` **directly**, so changing this package's API can break it. |
| [skybrush-io/pyledctrl](https://github.com/skybrush-io/pyledctrl) | Dependency: compiles LED light programs into the bytecode the `.skyc` expects. |

# 1. Installation

You need **Python ≥ 3.9** and **git** (one dependency, `pyledctrl`, is installed straight from GitHub).

Clone this repository, then change into the cloned directory.

### On Linux

```bash
python3 -m venv venv
source venv/bin/activate
pip install -e .
```

### On Windows

```
python -m venv venv
cd venv\Scripts
activate
cd ..\..
pip install -e .
```

`-e` (editable) means the package is installed as a link to this folder: edit the source, and the change takes effect immediately, with no reinstall. Use it while developing.

Other lab repositories don't clone this one; they depend on it by git URL, e.g.

```toml
dependencies = ["skyc_utils @ git+https://github.com/AIMotionLab-SZTAKI/skyc_utils.git"]
```

which means **a breaking change pushed here is a breaking change in those repos** the next time somebody reinstalls.

# 2. What a `.skyc` file actually is

A `.skyc` file is a zip archive with the extension swapped to .skyc. Rename it to `.zip` and open it with any archive manager to see:
```
show.skyc
├── show.json                 <- show metadata, drone start positions
├── cues.json                 <- named time markers
└── drones/
    ├── drone_0/
    │   ├── trajectory.json   <- the flight path of drone 0
    │   └── lights.json       <- the LED program of drone 0 (if the show has lights)
    └── drone_1/
        └── ...
```

`show.json` references the per-drone files with JSON pointers (`{"$ref": "./drones/drone_0/trajectory.json#"}`) and stores, for each drone:

* `home` — the `(x, y, z)` the drone starts at. Skybrush Live matches physical drones to trajectories by looking at which `home` position they are standing closest to, so this has to reflect reality.
* `startYaw`, `landAt` — starting heading and final position.
* `parameters` — optional list of timed Crazyflie parameter writes, produced by `Trajectory.add_parameter`.

## 2.1 The trajectory format

`trajectory.json` describes the path as a **chain of Bézier curves**, one per segment:

```json
{
  "version": 1,
  "points": [
    [0.0, [0.0, 0.0, 0.0, 0.0], []],
    [3.0, [0.0, 0.0, 1.0, 0.0], [[0.0, 0.0, 0.2, 0.0], [0.0, 0.0, 0.7, 0.0]]],
    [7.0, [1.0, 0.0, 1.0, 0.0], [[0.3, 0.0, 1.1, 0.0], [0.8, 0.0, 1.1, 0.0]]]
  ],
  "takeoffTime": 0.0,
  "landingTime": 7.0,
  "type": "COMPRESSED"
}
```

Every element of `points` is `[t, end_point, inner_control_points]`:

* `t` — the time (seconds from show start) at which the drone arrives at this point,
* `end_point` — `[x, y, z, yaw]`, the physical arrival point,
* `inner_control_points` — the Bézier control points between the previous arrival point and this one. There are `degree - 1` of them: none for a straight line, 2 for a cubic, 6 for a degree-7 curve.

Notice that the starting point of the curve is not specified: it is the end point of the previous segment, ensuring that disjointed segments cannot be created.

The `type` field picks how the server encodes the trajectory for the firmware:

* **`POLY4D`** — each segment is sent as raw polynomial coefficients. Degree up to 7.
* **`COMPRESSED`** — a space-efficient encoding the firmware unpacks itself. Only degrees 1, 3 and 7 are representable. Lower resolution is the tradeoff, which might not be desirable with sensitive controller schemes.

# 3. Quick start

```python
from skyc_utils.trajectory import Trajectory, TrajectoryType, Pose, Velocity
from skyc_utils.skyc import Skyc, plot_skyc_trajectories

# a trajectory that starts on the ground at the origin, facing +x
traj = Trajectory(TrajectoryType.COMPRESSED, degree=3, start=Pose(0.0, 0.0, 0.0, 0.0))

traj.append_goto(Pose(0.0, 0.0, 1.0, 0.0), dt=3.0)                      # take off to 1 m, 3 s
traj.append_goto(Pose(1.0, 0.0, 1.0, 0.0), dt=4.0, end_vel=Velocity())  # fly 1 m in x, arrive stopped
traj.append_goto(Pose(0.0, 0.0, 0.0, 0.0), dt=4.0, end_vel=Velocity())  # come back and land

skyc = Skyc()
skyc.add_drone(traj)          # call once per drone
skyc.write("my_first_show")   # writes my_first_show.skyc into the current directory

plot_skyc_trajectories("my_first_show.skyc")   # look at it before you fly it
```

Worked examples live in [examples/](examples/).

# 4. Building a trajectory

The `Trajectory` class ([skyc_utils/trajectory.py](skyc_utils/trajectory.py)) is where most of the code lives: it represents one drone's trajectory from start to end.

## 4.1 The vocabulary

Everything is **4-dimensional**: `x`, `y`, `z` (metres) and `yaw` (radians). 
| Class | Meaning |
| --- | --- |
| `Pose`, `Velocity`, `Acceleration`, `Jerk` | A 4-vector at a given derivative order (0, 1, 2, 3). All subclass `State4D`. |
| `FullState` | A `Pose` plus its `vel`, `acc` and `jerk` — the complete kinematic state at an instant. |
| `AxisPPoly` | Four scipy `PPoly` objects (one per axis) sharing one set of breakpoints. |
| `Trajectory` | The whole flight path, plus its type, degree and timed parameters. |
| `TrajectoryType` | `POLY4D` or `COMPRESSED`: how the exported trajectory is encoded for the firmware. |

`State4D` objects are indexable and iterable over their four components, which is handy:

```python
p = Pose(1.0, 2.0, 0.5, 0.0)
p.x, p[0], p[-1]      # 1.0, 1.0, 0.0
x, y, z, yaw = p      # unpacking works
p.as_tuple            # (1.0, 2.0, 0.5, 0.0)
```

Internally a `Trajectory` is not stored as Bézier curves, but as piecewise polynomials in the power basis (scipy `PPoly`, one per axis). They are converted to Bézier form only on export. This is because power-basis polynomials are easy to solve for from boundary conditions, which is exactly what `append_goto` does.

## 4.2 Type and degree

```python
traj = Trajectory(TrajectoryType.COMPRESSED, degree=3, start=Pose(0, 0, 0, 0))
```

* `degree` is **fixed for the whole trajectory**; every segment is stored with `degree + 1` coefficients (lower-degree pieces are zero-padded).
* `POLY4D` allows any `degree <= 7`. `COMPRESSED` allows only `degree in {1, 3, 7}`. Both limits come from the firmware; violating them raises immediately in the constructor.
* `start` is the pose the drone sits at before the first segment. A `Trajectory` with no segments evaluates to this pose everywhere.

## 4.3 `append_goto` 

```python
traj.append_goto(end, dt, *, end_vel=None, end_acc=None, end_jerk=None)
```

Adds one segment of duration `dt` seconds, ending at pose `end`. The **start** of the segment is taken from wherever the trajectory currently ends, so segments always join up; you never specify a segment's start.

What you may constrain at the end depends on the degree, because a polynomial of degree `d` has `d + 1` free coefficients, and every constraint eats one:

| You pass | Constraints at the end | Minimum degree |
| --- | --- | --- |
| `end` only | 1 (position) | 1 |
| `+ end_vel` | 2 | 3 |
| `+ end_vel, end_acc` | 3 | 5 |
| `+ end_vel, end_acc, end_jerk` | 4 | 7 |

You must always supply the lower-order derivatives: `end_acc` without `end_vel` is an error.

The remaining coefficients are spent on **matching the previous segment**: `append_goto` always enforces `(degree + 1) // 2` conditions at the start — i.e. degree 3 joins continuously in position and velocity, degree 5 up to acceleration, degree 7 up to jerk. That is why the table above needs those minimum degrees: `start_cond + end_cond` must not exceed `degree + 1`.

Note that:
- A goto with no derivative does not stop the drone, the velocity at the end will not be 0 unless specified: pass `end_vel=Velocity()` (all zeros) wherever you want the drone to actually come to a halt.
- The target yaw is unwrapped relative to the current yaw (`_shortest_yaw`), so asking for `yaw=3.1` from `yaw=-3.1` turns the short way, not 6.2 radians the long way. 

`prepend_goto(start, dt, *, start_vel=..., start_acc=..., start_jerk=...)` is the mirror image: it inserts a segment in front of the trajectory, from the given start state to whatever the trajectory currently begins with. Use it to put a takeoff in front of a path you generated some other way. It shifts every existing time later by `dt` and updates `traj.start`, but does not update the light program.

## 4.4 `append_bspline` — arbitrary curves from data

When the path comes from sampled data, an optimiser or a formula rather than from waypoints, fit scipy B-splines to it and hand those over:

```python
import numpy as np
from scipy.interpolate import make_splrep

t = np.linspace(0, 9.0, 100)          # must start at 0
x, y, z = np.sin(t), np.cos(t), 1.0 + 0.2 * t
yaw = np.zeros_like(t)

traj.append_bspline(make_splrep(t, x), make_splrep(t, y),
                    make_splrep(t, z), make_splrep(t, yaw))
```

Rules:

* All four splines must share the same knot vector and the same degree; that degree must not exceed the trajectory's `degree`.
* The spline's parameter is time in seconds, and must start at 0. The knot vector's span becomes the duration of the appended part.
* `yaw` is optional; leave it out and yaw is held at 0 for that stretch.
* Continuity is not enforced. Unlike `append_goto`, this bolts the spline onto the end as-is. If the spline doesn't start where the trajectory currently ends, you get a jump. This can be avoided by bridging the gap with a goto.

`make_splrep` (smoothing fit) and `make_interp_spline` (exact interpolation) both work; see [examples/test.py](examples/test.py) for building a spiral by hand.

## 4.5 `append_ppoly` / `prepend_ppoly` 

If you generate polynomials yourself, wrap them in an `AxisPPoly` and splice them in directly. Each `PPoly` must start at `t = 0` and may not exceed the trajectory's degree; lower degrees are zero-padded automatically. `create_segment(start, end, start_cond, end_cond, dt)` is the routine `append_goto` uses to solve one segment from boundary conditions, and can be called on its own if you want asymmetric constraint counts.

## 4.6 Timed parameters

```python
traj.add_parameter(2.0, "stabilizer.controller", 1)
```

Records "at t = 2.0 s, set the Crazyflie parameter `stabilizer.controller` to 1". These end up in `show.json` next to the drone, and the lab's server fork forwards them on show upload (see `ext/crazyflie/driver.py` and `ext/aimotionlab` in skybrush-server). Use `add_parameter` rather than appending to `traj.parameters` by hand.

## 4.7 Inspecting a trajectory

| Call | Returns |
| --- | --- |
| `traj.duration` | Total length in seconds (`0.0` if empty). |
| `traj.evaluate(t)` | A `FullState` (pose, vel, acc, jerk) at time `t`. Clamps outside `[0, duration]`. |
| `traj.end_conditions` | `(FullState at the first knot, FullState at the last knot)`. |
| `traj.is_empty()` | Whether any segment has been added yet. |
| `traj.polynomial.x.x` | The raw breakpoint (segment boundary) times — useful for plotting. |

---

# 5. Light programs

LEDs are described by a `LightProgram` ([skyc_utils/light_program.py](skyc_utils/light_program.py)):

```python
from skyc_utils.light_program import LightProgram, Color

lights = LightProgram()
lights.set_color(0.0, Color.BLUE)          # blue from the start
lights.set_color(6.5, Color.MAGENTA)       # magenta from 6.5 s
lights.set_color(9.0, Color(120, 250, 50)) # custom RGB from 9 s onwards
```

* `set_color(t, color)` means "**from** `t` seconds, show this colour **until the next entry**". You give times, not durations; entries are kept sorted, so you can add them in any order.
* Before the first entry the drone is dark (`Color.BLACK`), and the last colour is held for the rest of the show.
* `Color(r, g, b)` takes 0–255 ints. The constants `Color.BLACK/RED/GREEN/BLUE/YELLOW/CYAN/MAGENTA/WHITE` are provided.
* Times are on the **same clock as the trajectory** (seconds from show start). Note that `prepend_goto` shifts the trajectory in time but leaves light programs alone — if you prepend after writing your light program, fix the light times up yourself.
* Only step changes are supported today. `pyledctrl` itself can do fades and blinking; adding them means extending `LightProgram.source`.

Under the hood, `LightProgram` renders a small `.led` source text (`set_color(r, g, b, duration=...)` lines) and lets `pyledctrl`'s `BytecodeCompiler` turn it into the bytecode `lights.json` needs. We additionally store a plain `colors` list in that JSON, which is a **non-standard extra field** that exists so we can read our own light programs back (the bytecode is not decompiled).

---

# 6. Writing and reading `.skyc` files

## 6.1 Writing

```python
from skyc_utils.skyc import Skyc

skyc = Skyc()
skyc.add_drone(traj0, lights0)
skyc.add_drone(traj1, lights1)
skyc.write("three_drones")     # -> three_drones.skyc
```

* `add_drone(traj, light_program=None)` appends one drone. Drones are named `drone_0`, `drone_1`, … in the order added.
* **Lights are all-or-nothing.** Whether the show has lights is decided by the *first* `add_drone` call; after that, passing a light program when the first one didn't (or omitting one when it did) raises an assertion. If one drone should be dark, give it an empty `LightProgram()`.
* `write(name)` produces `name.skyc`. If you omit `name`, it defaults to the filename of the script you ran, minus `.py`.
* Each drone's `home`, `startYaw` and `landAt` in `show.json` are taken automatically from the first and last points of its trajectory.

## 6.2 Reading back

```python
from skyc_utils.skyc import Skyc, plot_skyc_trajectories

skyc = Skyc.from_file("three_drones.skyc")
traj, lights = skyc.drones[0]          # tuples of (Trajectory,) or (Trajectory, LightProgram)
print(traj.duration, traj.evaluate(2.0))

plot_skyc_trajectories("three_drones.skyc")   # pose/velocity/acceleration plots, one window per drone
```

`Skyc.from_file` unzips the archive in memory, rebuilds a `Trajectory` per drone from its Bézier points (converting back to the internal power-basis form), restores timed parameters from `show.json`, and rebuilds light programs if the show has them. Trajectory type, start pose, segment times and polynomial degree all survive the round trip.

The individual pieces can also be read on their own, which is useful when you have extracted a JSON file from an archive by hand:

```python
traj = Trajectory.from_json("trajectory.json")
lights = LightProgram.from_json("lights.json")
```

One limitation: `LightProgram.from_json` relies on the extra `colors` field we write ourselves, so it can read *our* light programs but not ones produced by other Skybrush tooling (e.g. Skybrush Studio), whose `lights.json` only contains compiled bytecode.

`plot_skyc_trajectories` is the quickest correctness check you have: it plots position, velocity and acceleration for all four axes.
