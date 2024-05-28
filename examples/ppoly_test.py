from typing import List, Optional
import scipy.interpolate as interpolate
import numpy as np
from skyc_utils.skyc_maker import Trajectory, write_skyc, XYZYaw, evaluate_pickle, TrajectoryType
import matplotlib.pyplot as plt
import math
from skyc_utils.optimal_trajectory import find_optimal_poly


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
    spline = interpolate.splrep(times, values, k=n)
    deriv = [interpolate.splev(eval_at, spline, der=i) for i in range(n+1)]
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
    bspline_lst = [interpolate.splrep(t, values, k=degree) for values in t_x_y_z_yaw[1:]]
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


if __name__ == "__main__":
    pickle_filename = "traj.pickle"
    N = 50
    t_x_y_z_yaw, parameters = evaluate_pickle(pickle_filename)
    t, _, _, _, _ = t_x_y_z_yaw
    eq_knots = False
    ppolys_1 = fit_ppoly_to_data(t_x_y_z_yaw, N, 5, method="lsqspline", equidistant_knots=eq_knots)
    ppolys_2 = fit_ppoly_to_data(t_x_y_z_yaw, N, 5, method="lsqspline", fit_ends=True, equidistant_knots=eq_knots)
    ppolys_3 = fit_ppoly_to_data(t_x_y_z_yaw, N, 5, method="lsqspline", fit_ends=True, force_0_derivs=True, equidistant_knots=eq_knots)
    ppolys_4 = fit_ppoly_to_data(t_x_y_z_yaw, N, 5, method="bspline", equidistant_knots=eq_knots)
    ppolys_5 = fit_ppoly_to_data(t_x_y_z_yaw, N, 5, method="mosek", fit_ends=True, equidistant_knots=eq_knots)
    ppolys_6 = fit_ppoly_to_data(t_x_y_z_yaw, N, 5, method="mosek", fit_ends=True, force_0_derivs=True, equidistant_knots=eq_knots)
    for i in range(4):
        plt.figure()
        # for knot in knots:
        #     plt.axvline(x=knot, color='r')
        plt.plot(t, ppolys_1[i](t), label="lsq")
        plt.plot(t, ppolys_2[i](t), label="lsq, endfit")
        plt.plot(t, ppolys_3[i](t), label="lsq, force0")
        # plt.plot(t, ppolys_4[i](t), label="bspline")
        plt.plot(t, ppolys_5[i](t), label="mosek")
        plt.plot(t, ppolys_6[i](t), label="mosek, force0")
        plt.plot(t, t_x_y_z_yaw[i+1], label="raw data")
        legend = plt.legend(loc='upper right')
    plt.show()

