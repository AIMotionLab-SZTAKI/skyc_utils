import tkinter as tk
from tkinter import filedialog
from functools import partial
from typing import List, Optional, Union, Any, Tuple
import os
import shutil

import matplotlib.pyplot as plt
from scipy.interpolate import splev, splrep, PPoly
from scipy import interpolate as interpolate
import numpy as np
import math
from copy import deepcopy


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


def determine_knots(time_vector, N, isochronous: bool = False):
    """
    returns knot vector for the BSplines according to the incoming timestamps and the desired number of knots
    Problems start to arise when part_length becomes way smaller than N, so generally keep them longer :)
    Knots may either be spaced equally in time (isochronous=True), or by index (isochronous=False)
    """
    if isochronous:
        part_length = len(time_vector) // N
        result = time_vector[::part_length][:N]
        result.append(time_vector[-1])
    else:
        start = min(time_vector)
        end = max(time_vector)
        result = list(np.linspace(start, end, N))
    return result


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


def weight_ends(n, factor):
    """Generates a list used for weighting in a bspline: ends are weighted symmetrically, middle is 1."""
    mid_idx = n // 2

    def calculate_value(idx):
        if idx == mid_idx:
            return 1.0
        else:
            dist = abs(idx - mid_idx) / mid_idx
            value = factor ** (dist ** 20)
            return value

    weighted_list = [calculate_value(i) for i in range(n)]
    return weighted_list


def data_to_ppoly(values: List[float], t: List[float], num_segments: int, degree: int, method: str,
                  max_error: float = 1/100, max_der_error: float = 2/100, recalculate: bool = False, verbose=True):
    """
    Fits a ppoly to the provided raw data.
    t_x_y_z_yaw is the raw interpolated data that we need to organize into number_of_segments polynomials.
    We can use two (three) methods:
    1.a:
    "bspline", with recalculate set to False is the equivalent of the "old" skyc_utils method. This fits a spline
    to the data points using splrep, and then converts the BSpline to a PPoly. There are two issues:
    -Since we have to use a set number of knots, the fit won't be exact, and this means that the start and end
    points may differ from the start and end points of the given data. This means that when we are fitting together
    trajectory methods made this way, they may not link up perfectly. Some controllers are sensitive to this (like
    geometric and mellinger)
    -We can only fit up to degree 5
    1.b:
    "bspline", with recalculate set to True adds weights to the ends of the trajectory, forcing them to be closer
    to the data points. It will do this as many times as it takes to get a good fit at the start and end, raising
    the weights each time. This means that trajectory segments will link up nicely, but will take a longer time to ű
    calculate.
    -We can only fit up to degree 5 still
    2:
    "lsqspline": fits a spline of any degree (!) to the data, then replaces the first and last segments with
    segments calculated by hand, that link up to the data points' start and end perfectly. In theory this is better
    in every way than method 1, but it hasn't been tested as much.
    """
    assert 1 <= degree <= 7
    derivs = derivatives(values, t, 3, [t[0], t[-1]])
    knots = determine_knots(t, num_segments, isochronous=True)
    if method == "bspline":
        k = 5 if degree > 5 else degree
        vel = derivs[1]
        # interpolate.splrep when used with task=-1 requires that knots[-1] < x[-1] and knots[0] > x[0]. To this end,
        # we must either use only knots[1:-1], or increment knots[0] and knots[-1] by a tiny fraction. To me it seems
        # that the first option has worked better. We may also consider adding an extra data point to x/y in such a way
        # that it maintains the derivative (which has sometimes been violated by splrep, which is partly the reason I'm
        # reworking this system). We may also consider increasing W at the ends in a while loop until derivative errors
        # get minimized.
        bspl = splrep(t, values, k=k, task=-1, t=knots[1:-1])  # fit a bspline without weights, and check error
        error = abs(values[0] - splev(t[0], bspl)) + abs(values[-1] - splev(t[-1], bspl))
        der_error = abs(vel[0] - splev(t[0], bspl, der=1)) + abs(vel[-1] - splev(t[-1], bspl, der=1))
        w = 10
        # if the error is not acceptable, we might want to recalculate
        while recalculate and (error > max_error or der_error > max_der_error) and w < 5000:
            if verbose:
                print(f"deviation: {error}, deriv deviation: {der_error}, recalculating with a max weight of {w}")
            weights = weight_ends(len(values), w)  # add weights to the ends to enforce end conditions
            w *= 5  # for next iteration, if needed
            bspl = splrep(t, values, w=weights, k=k, task=-1, t=knots[1:-1])
            error = abs(values[0] - splev(t[0], bspl)) + abs(values[-1] - splev(t[-1], bspl))
            der_error = abs(vel[0] - splev(t[0], bspl, der=1)) + abs(vel[-1] - splev(t[-1], bspl, der=1))
        ppoly = PPoly.from_spline(bspl)
        # when forming a ppoly like this, extra knots are appended to the start and end: theoretically they don't bother
        # us, but it's cleaner to remove them
        ppoly.x = ppoly.x[degree:-degree]
        ppoly.c = ppoly.c[:, degree:-degree]
        if k < degree:
            zeros_shape = (degree-k, ppoly.c.shape[1])
            ppoly.c = np.vstack((np.zeros(zeros_shape), ppoly.c))
        return ppoly
    elif method == "lsqspline":
        assert 1 <= degree <= 7
        knots = np.r_[(knots[0],)*degree, knots, (knots[-1],)*degree]  # make duplicate end knots (as per scipy docs)
        lsq_spline = interpolate.make_lsq_spline(t, values, knots, k=degree)
        ppoly = PPoly.from_spline(lsq_spline)
        ppoly.x = ppoly.x[degree + 1:-(degree + 1)]  # remove the first and last segments: we'll replace them
        ppoly.c = ppoly.c[:, degree + 1:-(degree + 1)]
        start = [lst[0] for lst in derivs][:(degree+1)//2]
        end = [float(ppoly(ppoly.x[0], nu=n)) for n in range(4)][:(degree+1)//2]
        # calculate coefficients that satisfy the following:
        # their ppoly and its derivatives match up with the start of the data set at the start
        # their ppoly and its derivatives match up with the start of the TRIMMED trajectory
        coeffs = calculate_polynomial_coefficients(start, end, 0, ppoly.x[0] - t[0])
        # coeffs must be shaped (N, 1), and the first element must correspond to the highest order -> reshape, flip
        coeffs = np.flip(coeffs.reshape(len(coeffs), 1))
        ppoly.x = np.hstack((t[0], ppoly.x))
        ppoly.c = np.hstack((coeffs, ppoly.c))
        # now do the same for the end
        start = [float(ppoly(ppoly.x[-1], nu=n)) for n in range(4)][:(degree+1)//2]
        end = [lst[-1] for lst in derivs][:(degree+1)//2]
        coeffs = calculate_polynomial_coefficients(start, end, 0, t[-1] - ppoly.x[-1])
        coeffs = np.flip(coeffs.reshape(len(coeffs), 1))
        ppoly.x = np.hstack((ppoly.x, t[-1]))
        ppoly.c = np.hstack((ppoly.c, coeffs))
        return ppoly


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
