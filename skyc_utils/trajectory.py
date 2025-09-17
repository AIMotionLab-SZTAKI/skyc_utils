from __future__ import annotations
from dataclasses import dataclass, field
from typing import Iterable, Iterator, Tuple
from enum import Enum
from typing import Union, Optional, Sequence
import numpy as np
import math
from copy import deepcopy
import json

from pkg_resources import require
from scipy.interpolate import PPoly, BPoly, BSpline, make_splrep, make_splprep, make_interp_spline


@dataclass(slots=True)
class State4D:
    """
    Pose-like 4-vector `(x, y, z, yaw)` associated with a derivative order.

    A :class:`State4D` represents either a configuration in 3D with heading
    (``deriv == 0``) or a higher-order derivative thereof (velocity for ``1``,
    acceleration for ``2``, jerk for ``3``, etc.).

    Indexing and iteration operate **only** on the four spatial components. The
    derivative order is stored in :pyattr:`deriv` and is *not* indexable.

    - Direct attributes: ``p.x, p.y, p.z, p.yaw, p.deriv``
    - Indexing: ``p[0] -> x``, ``p[1] -> y``, ``p[2] -> z``, ``p[3] -> yaw``
    - Slices return tuples (e.g., ``p[:2]``)
    - Negative indices are supported (``p[-1] -> yaw``)

    .. note::
       Units are not enforced. Keep units consistent across usages
       (e.g., meters / radians).

    Attributes:
        x (float): Component value at derivative order ``deriv`` for the x axis.
        y (float): Component value for the y axis.
        z (float): Component value for the z axis.
        yaw (float): Component value for the yaw axis (radians unless you decide otherwise).
        deriv (int): Derivative order, non-negative (0=pose, 1=velocity, ...).
    """

    x: float = 0
    y: float = 0
    z: float = 0
    yaw: float = 0
    deriv: int = 0

    def __post_init__(self) -> None:
        """
        Validate the derivative order.

        Raises:
            ValueError: If :pyattr:`deriv` is negative or not an integer.
        """
        if not isinstance(self.deriv, int) or self.deriv < 0:
            raise ValueError("deriv must be a non-negative integer")

    # ---------------- Sequence-like behavior (over x, y, z, yaw only) ----------------

    def __len__(self) -> int:
        """
        Number of indexable components.

        Returns:
            int: Always ``4`` (x, y, z, yaw).
        """
        return 4

    def __iter__(self) -> Iterator[float]:
        """
        Iterate over the four indexable components.

        Yields:
            float: The components in order: x, y, z, yaw.
        """
        yield self.x
        yield self.y
        yield self.z
        yield self.yaw

    def __getitem__(self, key):
        """
        Indexing and slicing over ``(x, y, z, yaw)`` only.

        Args:
            key (int | slice): Index or slice into the four components.

        Returns:
            float | tuple[float, ...]: A single component (for int) or a tuple (for slice).

        Raises:
            TypeError: If ``key`` is neither ``int`` nor ``slice``.
            IndexError: If the index is out of range.
        """
        data = (self.x, self.y, self.z, self.yaw)
        if isinstance(key, slice):
            return data[key]
        if isinstance(key, int):
            try:
                return data[key]  # supports negative indices
            except IndexError:
                raise IndexError("State4D index out of range (valid -4..3)")
        raise TypeError("State4D indices must be int or slice")

    def __setitem__(self, key, value) -> None:
        """
        Assign by index to one of ``(x, y, z, yaw)``.

        Args:
            key (int): Index in ``[-4..3]`` selecting a component.
            value (float): New component value.

        Raises:
            TypeError: If ``key`` is not an integer.
            IndexError: If the index is out of range.
        """
        if not isinstance(key, int):
            raise TypeError("State4D assignment index must be int")
        idx = key if key >= 0 else 4 + key
        if idx == 0:
            self.x = float(value)
        elif idx == 1:
            self.y = float(value)
        elif idx == 2:
            self.z = float(value)
        elif idx == 3:
            self.yaw = float(value)
        else:
            raise IndexError("State4D assignment index out of range (valid -4..3)")

    # ---------------- Convenience API ----------------

    @property
    def as_tuple(self) -> Tuple[float, float, float, float]:
        """
        Return the four indexable components as a tuple.

        Returns:
            tuple[float, float, float, float]: ``(x, y, z, yaw)``.
        """
        return self.x, self.y, self.z, self.yaw

    @classmethod
    def from_iterable(cls, values: Iterable[float], *, deriv: int = 0) -> "State4D":
        """
        Construct a :class:`State4D` from an iterable of four numeric values.

        Args:
            values (Iterable[float]): Exactly four values in order ``x, y, z, yaw``.
            deriv (int): Derivative order (default: ``0``).

        Returns:
            State4D: The constructed instance.

        Raises:
            ValueError: If ``values`` does not yield exactly four elements.
        """
        vals = tuple(values)
        if len(vals) != 4:
            raise ValueError("from_iterable expects exactly 4 values: (x, y, z, yaw)")
        x, y, z, yaw = vals
        return cls(float(x), float(y), float(z), float(yaw), deriv=deriv)

    def __repr__(self) -> str:
        """
        Developer-friendly representation.

        Returns:
            str: A string including all fields.
        """
        return (f"State4D(x={self.x!r}, y={self.y!r}, z={self.z!r}, "
                f"yaw={self.yaw!r}, deriv={self.deriv!r})")


# -------- Specialized subclasses that lock deriv --------

class Pose(State4D):
    """State with derivative order locked to ``0`` (position)."""

    def __init__(self, x: float = 0, y: float = 0, z: float = 0, yaw: float = 0):
        super().__init__(x, y, z, yaw, deriv=0)


class Velocity(State4D):
    """State with derivative order locked to ``1`` (velocity)."""

    def __init__(self, x: float = 0, y: float = 0, z: float = 0, yaw: float = 0):
        super().__init__(x, y, z, yaw, deriv=1)


class Acceleration(State4D):
    """State with derivative order locked to ``2`` (acceleration)."""

    def __init__(self, x: float = 0, y: float = 0, z: float = 0, yaw: float = 0):
        super().__init__(x, y, z, yaw, deriv=2)


class Jerk(State4D):
    """State with derivative order locked to ``3`` (jerk)."""

    def __init__(self, x: float = 0, y: float = 0, z: float = 0, yaw: float = 0):
        super().__init__(x, y, z, yaw, deriv=3)


@dataclass(slots=True)
class AxisPPoly:
    """
    Piecewise power-basis polynomials for each axis.

    Holds four :class:`scipy.interpolate.PPoly` objects corresponding to ``x``,
    ``y``, ``z``, and ``yaw``. All four are expected to share the same knot
    vector (breakpoints).

    Attributes:
        x (PPoly): PPoly for x.
        y (PPoly): PPoly for y.
        z (PPoly): PPoly for z.
        yaw (PPoly): PPoly for yaw.
    """

    x: PPoly
    y: PPoly
    z: PPoly
    yaw: PPoly

    def __iter__(self) -> Iterator[PPoly]:
        """
        Iterate over the four axis polynomials.

        Yields:
            PPoly: Axis polynomials in order: ``x, y, z, yaw``.
        """
        yield self.x
        yield self.y
        yield self.z
        yield self.yaw


@dataclass(slots=True)
class FullState:
    """
    Full kinematic state in 4D: pose and its first three derivatives.

    Attributes:
        pose (Pose): 0th-order state.
        vel (Velocity): 1st-order state (default zeros).
        acc (Acceleration): 2nd-order state (default zeros).
        jerk (Jerk): 3rd-order state (default zeros).
    """

    pose: Pose
    vel: Velocity = field(default_factory=lambda: Velocity())
    acc: Acceleration = field(default_factory=lambda: Acceleration())
    jerk: Jerk = field(default_factory=lambda: Jerk())

    def as_matrix(self) -> np.ndarray:
        """
        Return a matrix view of the state values across orders.

        Returns:
            numpy.ndarray: Array of shape ``(4, 4)`` where **rows** are dimensions
            ``(x, y, z, yaw)`` and **columns** are derivative orders
            ``(0: pose, 1: vel, 2: acc, 3: jerk)``. Index it as ``M[dim, order]``.

        .. note::
           The matrix contains only **values**, not derivative orders. Units are
           those used to populate :class:`FullState`.
        """
        by_order = (self.pose, self.vel, self.acc, self.jerk)  # 0..3
        axes = ("x", "y", "z", "yaw")
        return np.asarray([[getattr(by_order[o], a) for o in range(4)] for a in axes], dtype=float)


class TrajectoryType(Enum):
    """
    Encoding type for exporting/packaging the trajectory.

    Values:
        POLY4D: Uncompressed (power-basis) representation with degree ``<= 7``.
        COMPRESSED: Compressed representation; degrees restricted to ``{1, 3, 7}``.

    .. seealso::
       Bitcraze Crazyflie firmware documentation on trajectory formats.
    """
    POLY4D = "POLY4D"
    COMPRESSED = "COMPRESSED"


def _shortest_yaw(current: float, target: float) -> float:
    """
    Unwrap a target yaw to follow the shortest angular distance.

    Args:
        current (float): Current yaw (radians).
        target (float): Target yaw (radians).

    Returns:
        float: Target yaw unwrapped so that motion from ``current`` to result
        lies in ``[-π, π]``.
    """
    diff = (target - current + math.pi) % (2 * math.pi) - math.pi
    return current + diff


def _solve_poly_from_boundary(derivs_start: np.ndarray, derivs_end: np.ndarray, t0: float, t1: float):
    """
    Solve for power-basis polynomial coefficients that meet boundary derivatives.

    Given derivative values at two boundaries ``t0`` and ``t1`` up to order
    ``k-1`` (where ``k = len(derivs_start) = len(derivs_end)``), this builds
    and solves a linear system for the coefficients ``c0 .. cN`` in **ascending**
    powers such that all specified derivatives match at both ends.

    Args:
        derivs_start (numpy.ndarray): Derivatives at ``t0``; shape ``(k,)``.
        derivs_end (numpy.ndarray): Derivatives at ``t1``; shape ``(k,)``.
        t0 (float): Start abscissa **in the local variable used for the segment**.
        t1 (float): End abscissa **in the same local variable**.

    Returns:
        numpy.ndarray: Coefficients in ascending powers ``[c0, c1, ...]``.

    Raises:
        ValueError: If ``derivs_start`` and ``derivs_end`` lengths differ.

    .. important::
       This function expects **local** times for the segment (e.g., ``t ∈ [0, dt]``).
       When constructing a :class:`PPoly`, its coefficients are expressed in the
       local variable ``(t - x_j)`` per segment. If you solve in absolute time
       but build a segment on ``[0, dt]``, you will get incorrect magnitudes.
    """
    if len(derivs_start) != len(derivs_end):
        raise ValueError("Start/end derivative lists must be of equal length.")
    N = len(derivs_start) + len(derivs_end)
    A = np.zeros((N, N), dtype=float)
    b = np.concatenate((derivs_start, derivs_end))

    k = len(derivs_start)
    # rows 0..k-1: start derivatives
    for i in range(k):                # derivative order at boundary
        for j in range(i, N):         # coefficient index
            A[i, j] = (math.factorial(j) / math.factorial(j - i)) * (t0 ** (j - i))
    # rows k..2k-1: end derivatives
    for i in range(k):
        for j in range(i, N):
            A[k + i, j] = (math.factorial(j) / math.factorial(j - i)) * (t1 ** (j - i))

    coeffs = np.linalg.solve(A, b)   # ascending powers: [c0, c1, ...]
    return coeffs


class Trajectory:
    """
    Append-only quadcopter trajectory built from piecewise power-basis polynomials.

    Internally the trajectory holds four :class:`~scipy.interpolate.PPoly` objects,
    one per axis (x, y, z, yaw). Segments are **always appended to the end**. When
    exporting to ``.skyc``, the piecewise power-basis representation is converted to
    Bernstein/Bezier segments.

    Args:
        traj_type (TrajectoryType): Packaging/encoding type (POLY4D or COMPRESSED).
        degree (int): Fixed polynomial degree per segment. For ``POLY4D``, ``degree <= 7``.
            For ``COMPRESSED``, ``degree in {1, 3, 7}``.
        start (Pose): Initial pose used when the trajectory is empty.

    Attributes:
        type (TrajectoryType): Encoding type.
        degree (int): Fixed degree of every segment.
        start (Pose): Starting pose (used for the very first splice).
        polynomial (Optional[AxisPPoly]): The internal piecewise representation (``None`` until first segment).
        bezier (Optional[list]): Cached Bezier export (if you build it later).

    .. note::
       Lower-degree behavior is emulated by zeroing higher-order coefficients as needed.
       All axes share the same breakpoints by construction.
    """

    def __init__(self, traj_type: TrajectoryType, degree: int = 3, start: Pose = Pose(0.0, 0.0, 0.0, 0.0)):
        if traj_type == TrajectoryType.COMPRESSED:
            if degree not in (1, 3, 7):
                raise ValueError("COMPRESSED trajectories must have degree in {1, 3, 7}.")
        else:
            if degree > 7:  # optionally restrict to odd degrees for symmetry
                raise ValueError("POLY4D trajectories must have degree <= 7.")
        self.type: TrajectoryType = traj_type
        self.degree: int = degree
        self.start: Pose = start

        # Four PPolys once anything is added; until then, None
        self.polynomial: Optional[AxisPPoly] = None

        # Cached Bezier representation (built on demand by to_bezier/export_json)
        self.bezier: Optional[list] = None

    # -----------------------------------------------------------------------
    # Public API
    # -----------------------------------------------------------------------

    def is_empty(self) -> bool:
        """
        Whether the trajectory currently has no segments.

        Returns:
            bool: ``True`` if no segments have been added; ``False`` otherwise.
        """
        return self.polynomial is None

    @property
    def duration(self) -> float:
        """
        Total trajectory duration (end time).

        Returns:
            float: End time of the last segment, or ``0.0`` if empty.
        """
        if self.polynomial is None:
            return 0.0
        # All axes share the same breakpoints by construction
        return float(self.polynomial.x.x[-1])

    def eval_fullstate(self, t: float) -> FullState:
        """
        Evaluate pose, velocity, acceleration, and jerk at time ``t``.

        Args:
            t (float): Time at which to evaluate.

        Returns:
            FullState: The evaluated full kinematic state.

        .. note::
           If the trajectory is empty, this returns the ``start`` pose with zeros
           for all derivatives.
        """
        if self.polynomial is None:
            return FullState(pose=self.start)

        px  = float(self.polynomial.x(t, nu=0)); py  = float(self.polynomial.y(t, nu=0))
        pz  = float(self.polynomial.z(t, nu=0)); pyw = float(self.polynomial.yaw(t, nu=0))
        vx  = float(self.polynomial.x(t, nu=1)); vy  = float(self.polynomial.y(t, nu=1))
        vz  = float(self.polynomial.z(t, nu=1)); vyw = float(self.polynomial.yaw(t, nu=1))
        ax  = float(self.polynomial.x(t, nu=2)); ay  = float(self.polynomial.y(t, nu=2))
        az  = float(self.polynomial.z(t, nu=2)); ayw = float(self.polynomial.yaw(t, nu=2))
        jx  = float(self.polynomial.x(t, nu=3)); jy  = float(self.polynomial.y(t, nu=3))
        jz  = float(self.polynomial.z(t, nu=3)); jyw = float(self.polynomial.yaw(t, nu=3))
        return FullState(
            pose=Pose(px, py, pz, pyw),
            vel=Velocity(vx, vy, vz, vyw),
            acc=Acceleration(ax, ay, az, ayw),
            jerk=Jerk(jx, jy, jz, jyw),
        )

    @property
    def end_conditions(self) -> Tuple[FullState, FullState]:
        """
        Kinematic end conditions at the first and last breakpoints.

        Returns:
            tuple[FullState, FullState]: ``(start, end)`` full states evaluated at
            the first and last knots. If empty, both are the ``start`` pose with
            zero derivatives.

        .. note::
           All axes are assumed to share the same knot vector; the x-axis is used
           to read the first/last times.
        """
        t0 = float(self.polynomial.x.x[0]) if self.polynomial is not None else 0
        t1 = float(self.polynomial.x.x[-1]) if self.polynomial is not None else 0
        return self.eval_fullstate(t0), self.eval_fullstate(t1)

    def add_ppoly(self, ppoly: AxisPPoly):
        """
        Append an :class:`AxisPPoly` to the trajectory.

        If the trajectory is empty, this becomes the initial representation.
        Otherwise, the new polynomials' breakpoints are shifted to start at the
        current end time and concatenated to the existing piecewise representation.

        Args:
            ppoly (AxisPPoly): Segment(s) to append (must share the same knots across axes).
        """
        for p in ppoly:
            assert p.x[0] == 0, "PPoly must start at time 0!"
            height, width = p.c.shape
            padding = self.degree + 1 - height
            if padding < 0:
                raise ValueError("PPoly degree cannot be higher than the Trajectory degree! ")
            p.c = np.vstack((np.zeros((padding, width)), p.c))
        if self.polynomial is None:
            self.polynomial = ppoly
        else:
            for existing_ppoly, new_ppoly in zip(self.polynomial, ppoly):
                existing_ppoly.x = np.hstack((existing_ppoly.x, new_ppoly.x[1:] + existing_ppoly.x[-1]))
                existing_ppoly.c = np.hstack((existing_ppoly.c, new_ppoly.c))
        # Invalidate any cached Bezier export (if present)
        self.bezier = None

    def add_goto(
        self,
        end: Pose,
        dt: float,
        *,
        end_vel: Optional[Velocity] = None,
        end_acc: Optional[Acceleration] = None,
        end_jerk: Optional[Jerk] = None,
    ) -> None:
        """
        Append a ``dt``-long segment that reaches ``end`` (and optional end derivatives),
        matching the current trajectory state at the splice point.

        The number of derivatives enforced **per endpoint** (continuity order) is
        determined by which end derivatives are provided:

        - Position only        → continuity ``= 1`` (pos)
        - + velocity           → continuity ``= 2`` (pos, vel)
        - + acceleration       → continuity ``= 3`` (pos, vel, acc)
        - + jerk               → continuity ``= 4`` (pos, vel, acc, jerk)

        Constraints are bounded by the polynomial degree: one needs
        ``continuity ≤ floor((degree + 1) / 2)``. Any higher coefficients beyond what is
        required are set to zero, effectively mimicking a lower-degree polynomial.

        Yaw is unwrapped to follow the shortest turn from the current yaw.

        Args:
            end (Pose): Target pose at the end of the new segment.
            dt (float): Segment duration (must be positive).
            end_vel (Optional[Velocity]): Desired end velocity (optional).
            end_acc (Optional[Acceleration]): Desired end acceleration (requires velocity).
            end_jerk (Optional[Jerk]): Desired end jerk (requires acceleration and velocity).

        Raises:
            AssertionError: If a higher-order derivative is provided without its lower-order
                prerequisites (e.g., jerk without acceleration/velocity), or if the degree
                cannot support the requested continuity.
            ValueError: If ``dt`` is not positive.

        .. important::
           The boundary problem is solved in **local time** over ``[0, dt]`` and then
           appended to the global piecewise representation (which shifts the knots).
        """
        if not (dt > 0):
            raise ValueError("dt must be positive")

        if end_jerk is not None:
            assert end_acc is not None and end_vel is not None, "Specify all lower order derivatives!"
            continuity = 4
        elif end_acc is not None:
            assert end_vel is not None, "Specify all lower order derivatives!"
            continuity = 3
        elif end_vel is not None:
            continuity = 2
        else:
            continuity = 1

        assert (self.degree + 1) // 2 >= continuity, "Trajectory degree too low for the required continuity!"

        # Build end state (fill unspecified orders with zeros)
        _, cur = self.end_conditions
        end_state = FullState(
            end,
            end_vel if end_vel is not None else Velocity(),
            end_acc if end_acc is not None else Acceleration(),
            end_jerk if end_jerk is not None else Jerk(),
        )
        end_state.pose.yaw = _shortest_yaw(cur.pose.yaw, end.yaw)

        # Solve per axis in local time [0, dt]
        cur_M = cur.as_matrix()
        end_M = end_state.as_matrix()
        coeffs = np.zeros((4, self.degree + 1))

        for dim_idx in range(4):
            derivs_start = cur_M[dim_idx, :continuity]
            derivs_end   = end_M[dim_idx, :continuity]
            coeffs[dim_idx, : 2 * continuity] = _solve_poly_from_boundary(derivs_start, derivs_end, 0.0, dt)

        new_segment = AxisPPoly(
            x=PPoly(np.flip(coeffs[0, : self.degree + 1].reshape(-1, 1)), np.array([0.0, dt])),
            y=PPoly(np.flip(coeffs[1, : self.degree + 1].reshape(-1, 1)), np.array([0.0, dt])),
            z=PPoly(np.flip(coeffs[2, : self.degree + 1].reshape(-1, 1)), np.array([0.0, dt])),
            yaw=PPoly(np.flip(coeffs[3, : self.degree + 1].reshape(-1, 1)), np.array([0.0, dt])),
        )
        self.add_ppoly(new_segment)

    def add_bspline(self, x: BSpline, y: BSpline, z: BSpline, yaw: Optional[BSpline]=None):
        if yaw is None:
            yaw = deepcopy(x)
            yaw.c = np.zeros_like(yaw.c)
        assert (x.t == y.t).all() and (y.t == z.t).all() and (z.t == yaw.t).all(), "BSpline knots must match!"
        assert x.k == y.k == z.k == yaw.k, "BSpline degrees must match!"
        deg = x.k # same as y.k, z.k, yaw.k
        ppoly = AxisPPoly(
            x=PPoly.from_spline(x),
            y=PPoly.from_spline(y),
            z=PPoly.from_spline(z),
            yaw=PPoly.from_spline(yaw),
        )
        for p in ppoly:
            p.x = p.x[deg:-deg]
            p.c = p.c[:, deg:-deg]
        self.add_ppoly(ppoly)

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
        assert self.polynomial is not None
        bezier_repr = [[0.0, [self.start.x, self.start.y, self.start.z, self.start.yaw], []]]
        bpolys = [BPoly.from_power_basis(ppoly) for ppoly in self.polynomial]
        # These two lines below seem complicated but all they do is pack the data above into a convenient form: a list
        # of lists where each element looks like this: [t, (x,y,z), (x,y,z), (x,y,z)].
        bpoly_pts = list(zip(list(bpolys[0].x)[1:], *[list(bpoly.c.transpose()) for bpoly in bpolys]))
        # at this point bpoly_pts contains the control points for the segments, but that's not exactly what we need in
        # the skyc file: we need the last point, and the inside points
        bezier_curves = [[element[0]] + list(zip(*list(element[1:]))) for element in bpoly_pts]
        for bezier_curve in bezier_curves:
            curve_to_append = [bezier_curve[0],
                               bezier_curve[-1],
                               bezier_curve[2:-1]]
            bezier_repr.append(curve_to_append)
        self.bezier_repr = bezier_repr

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
        json_object = json.dumps(json_dict, indent=2)
        if write_file:
            with open("trajectory.json", "w") as f:
                f.write(json_object)
        return json_object

