import tkinter as tk
from tkinter import filedialog
from functools import partial
from typing import List, Optional, Union, Any, Tuple
import os
import shutil
from scipy.interpolate import splev, splrep, PPoly
from scipy import interpolate as interpolate
import numpy as np
import math
from .optimal_trajectory import find_optimal_poly
import trio
from typing import Union, List
from trio import Event, current_time
import re
from queue import PriorityQueue


def is_num(var):
    """Why isn't this a built-in?????"""
    return isinstance(var, float) or isinstance(var, int)


def open_file_dialog(file_path_var: List[Optional[str]], root: tk.Tk, filetype: str) -> None:
    """
    Helper function for select_file, which handles the selection window.
    """
    file_path = filedialog.askopenfilename(initialdir=os.path.dirname(__file__),
                                           title=f"Select a {filetype} file!",
                                           filetypes=[(f"{filetype} files", f"*.{filetype}"), ("all files", "*.*")])
    if file_path:
        print(f"Selected file: {file_path}")
        file_path_var[0] = file_path
        root.destroy()


def select_file(filetype: str) -> Union[None, str]:
    """ Function that prompts the user to select a skyc file, and returns the file's name if successful.
    Else returns None"""
    selecter = tk.Tk()
    selecter.title(f"Select {filetype} file!")
    selecter.geometry("300x100")
    # using a list to mimic a mutable object (I hate python so much why are you forcing me to
    # do this, just let me pass this by reference, horrible toy language for children and phds...)
    selected_file = [None]
    button_func = partial(open_file_dialog, selected_file, selecter, filetype)
    button = tk.Button(selecter, text=f"Select {filetype} file!", command=button_func, width=20, height=4)
    button.pack(pady=20)
    selecter.mainloop()
    return selected_file[0]


def cleanup(files: List[str], folders: List[str]):
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


def determine_shorter_deg(start: float, end: float):
    """If we did a turn from 350 to 10 degrees, the goto segment planner would turn the long way around, going
    from 350->340->330...20->10, instead of going the short way, which is 350->360->370=10. This function gives
    a target angle that yields turning the shortest way."""
    start_norm = start % 360
    end_norm = end % 360
    delta = end_norm - start_norm
    if delta > 180:
        delta = delta - 360
    elif delta < -180:
        delta = delta + 360
    return start + delta


def calculate_polynomial_coefficients(derivatives_start, derivatives_end, x_start: float, x_end: float):
    """
    Calculates the coefficients of a polynomial given the derivatives at the start and end,
    and the start and end abscissa coordinates.

    Parameters:
    - derivatives_start: List of derivatives at the start (0th derivative is the start value).
    - derivatives_end: List of derivatives at the end (0th derivative is the end value).
    - x_start: The abscissa coordinate of the start.
    - x_end: The abscissa coordinate of the end.

    Returns:
    - A list of polynomial coefficients.

    Thank you GPT-4 :))))))
    """
    assert len(derivatives_start) == len(derivatives_end), "Lists of derivatives must be of equal length."

    num_conditions = len(derivatives_start) + len(derivatives_end)
    A = np.zeros((num_conditions, num_conditions))
    b = np.array(derivatives_start + derivatives_end)

    # Construct matrix A for start conditions
    for i in range(len(derivatives_start)):
        for j in range(i, num_conditions):
            A[i, j] = (math.factorial(j) / math.factorial(j - i)) * (x_start**(j - i))

    # Construct matrix A for end conditions
    offset = len(derivatives_start)
    for i in range(len(derivatives_end)):
        for j in range(i, num_conditions):
            A[offset + i, j] = (math.factorial(j) / math.factorial(j - i)) * (x_end**(j - i))

    # Solve the system for the coefficients
    coefficients = np.linalg.solve(A, b)
    return coefficients  # coefficients are listed in increasing order of degree: c0, c1, ...


def derivatives(values: List[float], times: List[float], n: int, eval_at: list[float]):
    """returns a list of length n+1, where the ith row is a list of the ith derivative at times eval_at. 0th row
    is the values themselves"""
    spline = splrep(times, values, k=n)
    deriv = [splev(eval_at, spline, der=i) for i in range(n+1)]
    return deriv


def duplicate_end_knots(knots, degree):
    knots = np.r_[(knots[0],) * degree, knots, (knots[-1],) * degree]
    return knots


def calc_equidistant_knots(t_x_y_z_yaw: List[List[float]], number_of_segments: int):
    """Takes the t-x-y-z-yaw data, and returns a knot vector that will contain timestamps at which the distances
    traversed by the points are roughly equal. Basically, returns a knot configuration where knots are sparser over
    slower parts of the trajectory and more frequent."""
    t, x, y, z, _ = t_x_y_z_yaw
    dx_square = np.diff(x)**2
    dy_square = np.diff(y)**2
    dz_square = np.diff(z)**2
    # The distance between each two consecutive points:
    distances = [0] + [np.sqrt(d1 + d2 + d3) for d1, d2, d3 in zip(dx_square, dy_square, dz_square)]
    # The distance travelled up to a certain point, as approximated by the sum of the distances between points. Note,
    # that due to neglecting the curvature, this will be less than the actual curvature of the trajectory. However,
    # we're not really interested in the actual distance: we only need its distribution over time (the speed, basically)
    S = np.cumsum(distances)
    s_knots = np.linspace(S[0], S[-1], number_of_segments)
    knots = np.interp(s_knots, S, t)
    return knots


def trim_ppoly(ppoly: interpolate.PPoly) -> interpolate.PPoly:
    degree = ppoly.c.shape[0]-1
    ppoly.x = ppoly.x[degree:-degree]
    ppoly.c = ppoly.c[:, degree:-degree]
    return ppoly


def fit_ppoly_lsq(t_x_y_z_yaw: List[List[float]], knots, degree: int, *, fit_ends: bool = False,
                  force_0_derivs: bool = False) -> List[interpolate.PPoly]:
    """
    Returns a list of PPoly objects (piecewise polynomials) going between the given knots, one for each dimension
    t_x_y_z_yaw: the input, 0th element is the list of timestamps, 1st element is the x coordinates at those timestamps,
    2nd is the y coordinates at those timestamps, etc.
    knots: the breakpoints of the resulting piecewise polynomial
    degree: the degree of the resulting piecewise polynomial
    fit_ends: whether to make sure that the resulting ppoly fits perfectly to the start and end of the given data. This
    is done by cutting off the first and last segments of the trajectory and replacing them with smooth fitting segments
    that start or end at exactly the desired point and derivatives.
    force_0_derivs: whether to force the start and end derivatives to be 0
    """
    t, _, _, _, _ = t_x_y_z_yaw
    # when it comes to interpolate.make_lsq_spline, we need to give it duplicate end knots
    knots = duplicate_end_knots(knots, degree)
    # craft bsplines with the given knots, and transform them into piecewise polynomials
    bspline_lst = [interpolate.make_lsq_spline(t, dim, knots, k=degree) for dim in t_x_y_z_yaw[1:]]
    # remove duplicate end knots that don't influence the ppoly
    ppoly_lst = [trim_ppoly(interpolate.PPoly.from_spline(bspline)) for bspline in bspline_lst]
    if fit_ends:
        # end_conditions will be a list[list[list[float]]], where the 1st outermost index decides which dimension we're
        # looking at (x/y/z/yaw), the 2nd index decides the number of the derivative we're looking at (0=pos, 1=vel,
        # 2=acc, etc.), and the third index decides the timestamp at which we evaluate the derivative (t[0] and t[-1])
        end_conditions = [derivatives(data, t, 3, [knots[0], knots[-1]]) for data in t_x_y_z_yaw[1:]]
        if force_0_derivs:  # change the appropriate derivatives to 0
            for end_condition in end_conditions:
                end_condition[1:] = [[0, 0]] * len(end_condition[1:])
        for ppoly, derivs in zip(ppoly_lst, end_conditions):
            # start corresponds to t[0], end corresponds to the end of the 0th polynomial, which we're replacing
            start = [lst[0] for lst in derivs][:(degree + 1) // 2]
            end = [float(ppoly(ppoly.x[1], nu=n)) for n in range(4)][:(degree + 1) // 2]
            coeffs = calculate_polynomial_coefficients(start, end, 0, ppoly.x[1] - t[0])
            # coeffs must be shaped (N, 1), and the first element must correspond to the highest order -> reshape, flip
            coeffs = np.flip(coeffs.reshape(len(coeffs), 1))
            # replace the segment
            ppoly.c[:, 0] = np.reshape(coeffs, ppoly.c[:, 0].shape)
            # now start is the start of the last segment, end is the desired end of the trajectory
            start = [float(ppoly(ppoly.x[-2], nu=n)) for n in range(4)][:(degree + 1) // 2]
            end = [lst[-1] for lst in derivs][:(degree + 1) // 2]
            coeffs = calculate_polynomial_coefficients(start, end, 0, t[-1] - ppoly.x[-2])
            # again, flip and replace the segment
            coeffs = np.flip(coeffs.reshape(len(coeffs), 1))
            ppoly.c[:, -1] = np.reshape(coeffs, ppoly.c[:, -1].shape)
    return ppoly_lst


def fit_ppoly_mosek(t_x_y_z_yaw: List[List[float]], knots, degree: int, *,
                    force_0_derivs: bool = False) -> List[interpolate.PPoly]:
    """
    Returns a list of PPoly objects (piecewise polynomials) going between the given knots, one for each dimension
    t_x_y_z_yaw: the input, 0th element is the list of timestamps, 1st element is the x coordinates at those timestamps,
    2nd is the y coordinates at those timestamps, etc.
    knots: the breakpoints of the resulting piecewise polynomial
    degree: the degree of the resulting piecewise polynomial
    force_0_derivs: whether to force the start and end derivatives to be 0
    """
    t, _, _, _, _ = t_x_y_z_yaw
    bspline_lst = [interpolate.splrep(t, values, k=min(degree, 5)) for values in t_x_y_z_yaw[1:]]
    x, y, z, yaw = [interpolate.splev(knots, bspline) for bspline in bspline_lst]
    vel = [[knots[0], 0, 0, 0, 0]] + (len(knots) - 2) * [None] + [[knots[-1], 0, 0, 0, 0]]
    acc = [[knots[0], 0, 0, 0, 0]] + (len(knots) - 2) * [None] + [[knots[-1], 0, 0, 0, 0]]
    if not force_0_derivs:
        derivs = [derivatives(values, t, 3, [knots[0], knots[-1]]) for values in t_x_y_z_yaw[1:]]
        vel[0][1:] = [deriv_dim[1][0] for deriv_dim in derivs]
        vel[-1][1:] = [deriv_dim[1][-1] for deriv_dim in derivs]
        acc[0][1:] = [deriv_dim[2][0] for deriv_dim in derivs]
        acc[-1][1:] = [deriv_dim[2][-1] for deriv_dim in derivs]
    constraints = {
        "pos": [list(elem) for elem in list(zip(knots, x, y, z, yaw))],
        "vel": vel,
        "acc": acc,
        "v_max": 4 * [1],  # not used currently
        "a_max": 4 * [1]  # not used currently
    }
    return find_optimal_poly(constraints=constraints, poly_deg=degree, optim_order=2, continuity_order=2,
                             interior_point_prec=0.001)


def fit_ppoly_to_data(t_x_y_z_yaw: List[List[float]], number_of_segments: int, degree: int, *, method: str,
                      fit_ends: bool = False, force_0_derivs: bool = False,
                      equidistant_knots: bool = True) -> List[interpolate.PPoly]:
    """
    Fits a PPoly object to each dimension in the input data, and returns them in a list.
    t_x_y_z_yaw: the input, 0th element is the list of timestamps, 1st element is the x coordinates at those timestamps,
    2nd is the y coordinates at those timestamps, etc.
    number_of_segments: the resulting PPoly will have this many breakpoints
    degree: the degree of the resulting piecewise polynomial
    method: a string denoting which method to use:
    fit_ends: whether to make sure that the resulting ppoly fits perfectly to the start and end of the given data
    force_0_derivs: whether to force the start and end derivatives to be 0
    equidistant_knots: if set to True, the resulting knots will be spread so that the length of the path between them
    is roughly the same. If set to False, then the knots will be spaced evenly in time isntead
    """
    t = t_x_y_z_yaw[0]
    knots = calc_equidistant_knots(t_x_y_z_yaw, number_of_segments) if equidistant_knots else np.linspace(t[0], t[-1], number_of_segments)
    if method.lower() == "lsqspline":
        return fit_ppoly_lsq(t_x_y_z_yaw, knots, degree, fit_ends=fit_ends, force_0_derivs=force_0_derivs)
    elif method.lower() == "bspline":
        # in this method, fit_ends, force_0_derivs and end_conditions are ignored
        bspline_lst = [interpolate.splrep(t, values, k=degree, task=-1, t=knots[1:-1]) for values in t_x_y_z_yaw[1:]]
        return [trim_ppoly(interpolate.PPoly.from_spline(bspline)) for bspline in bspline_lst]
    elif method.lower() == "mosek":
        assert degree >= 5 and fit_ends
        return fit_ppoly_mosek(t_x_y_z_yaw, knots, degree, force_0_derivs=force_0_derivs)
    else:
        raise NotImplementedError


def as_list(thing: Any) -> List[Any]:
    """
    If the argument can be converted into a list, do so. If not, just pack it into a list of length 1.
    """
    if isinstance(thing, List):
        return thing
    elif isinstance(thing, (List, Tuple, np.ndarray)):
        return list(thing)
    else:
        return [thing]


def extend_ppoly_coeffs(ppoly: PPoly, coeff_num: int):
    """
    Add 0s as the higher order coefficients to make the ppoly have the provided number of coefficients, not affecting
    the actual shape of the resulting curve.
    """
    ppoly_deg = ppoly.c.shape[0]
    if ppoly_deg < coeff_num:
        zeros_shape = (coeff_num - ppoly_deg, ppoly.c.shape[1])
        ppoly.c = np.vstack((np.zeros(zeros_shape), ppoly.c))

PRIORITY_HIGH = 1
PRIORITY_MID = 2
PRIORITY_LOW = 3


class PrioEvent:
    """A class that has wait() and set() like trio.Event, but is comparable. Here, a subclass would be the obvious
    solution, instead of this. However, trio.Event doesn't allow subclassing."""

    def __init__(self, priority: int):
        self.event = Event()
        self.priority = priority

    async def wait(self) -> None:
        await self.event.wait()

    def set(self) -> None:
        self.event.set()

    def __lt__(self, other) -> bool:
        return self.priority < other.priority


class Semaphore:
    """
    Class that implements resource guarding. A trio asynchronous process may call wait_take() with
    a given priority in order to attempt to take command of the resource. If the resource is
    available, it gets taken, else the process will stand in a priority queue where the highest
    priority is given the right of way. Among processes that stood in queue with the same priority,
    this functions as a fifo. Once the process is done, it may let go with let_go(), at which point
    the next highest priority in queue may take the resource.
    """
    def __init__(self):
        self.taken = False
        self.queue = PriorityQueue()

    async def wait_take(self, priority: int) -> None:
        if not self.taken:
            self.taken = True
        else:
            ticket = PrioEvent(priority)
            self.queue.put(ticket)
            await ticket.wait()

    def let_go(self) -> None:  # make async? idk
        if self.queue.empty():
            self.taken = False
        else:
            ticket: PrioEvent = self.queue.get()
            ticket.set()


def warning(text: str) -> None:
    """
    Print stuff in scary red letters.
    """
    red = "\033[91m"
    reset = "\033[0m"
    formatted_text = f"WARNING: {text}"
    print(red + formatted_text + reset)


def determine_id(string: str) -> Union[None, str]:
    """
    takes a name as found in optitrack and returns the ID found in it, for example cf6 -> 06
    """
    number_match = re.search(r'\d+', string)
    if number_match:
        number = number_match.group(0)
        if len(number) == 1:
            number = '0' + number
        return number
    return None


async def establish_connection_with_handler(drone_id: str, port: int, ip):
    """ Attempts to connect to the skybrush server at the given ip and port, expecting the regular
     initialization steps for a drone handler. """
    drone_stream: trio.SocketStream = await trio.open_tcp_stream("127.0.0.1", port)
    request = f"REQ_{drone_id}"
    print(f"Requesting handler for drone {drone_id}")
    await drone_stream.send_all(request.encode('utf-8'))
    acknowledgement: bytes = await drone_stream.receive_some()
    if acknowledgement.decode('utf-8') == f"ACK_{drone_id}":
        print(f"successfully created server-side handler for drone {drone_id}")
        return drone_stream
    else:
        raise RuntimeError  # immediately stop if we couldn't reach one of the drones


def display_time():
    """returns the time since it was first called, in order to make time.time() more usable, since time.time() is
    a big number"""
    if not hasattr(display_time, 'start_time'):
        display_time.start_time = current_time()
    return current_time() - display_time.start_time


async def send_with_semaphore(data: bytes, semaphore: Semaphore, stream: trio.SocketStream, wait_ack: bool, timeout=2):
    """
    Waits for the given semaphore (which should be associated with the stream) to be available, then sends the data over
    the stream and waits for an acknowledgement. Raises a timeout error if the operations took too long.
    """
    await semaphore.wait_take(PRIORITY_HIGH)
    assert stream is not None
    with trio.move_on_after(timeout) as cancel_scope:
        await stream.send_all(data)
        if wait_ack:
            ack = await stream.receive_some()
            assert ack.endswith(b"ACK")
    if cancel_scope.cancelled_caught:
        raise TimeoutError
    semaphore.let_go()


