from typing import List, Tuple
import os
import pickle
import numpy as np
import scipy.interpolate as interpolate
import matplotlib.pyplot as plt
import casadi as ca
import mosek
import sys


def streamprinter(text):
    # Helper function for Mosek status printing
    sys.stdout.write(text)
    sys.stdout.flush()


def generate_trajectory(poly_deg, optim_order, constraints, continuity_order, interior_point_prec=0.0001,
                        mosek_printing=False, optim_printing=False):
    n = poly_deg + 1  # no. of coefficients (polynomial degree + 1)
    m = len(constraints["pos"])  # no. of waypoints
    T_lst = [pos[0] for pos in constraints["pos"]]
    dt_lst = [T_lst[i + 1] - T_lst[i] for i in range(m - 1)]
    T_lst_rel = [0] + dt_lst
    # Compute hessian of objective function
    t = ca.SX.sym('t')
    c = ca.SX.sym('c', n)
    # Polynomial represented in power basis
    basis = ca.vertcat(*[t ** i for i in range(n)])
    basis_ders = [basis]
    for der in range(optim_order):
        basis_ders = basis_ders + [ca.jacobian(basis_ders[der], t)]
    poly = c.T @ basis

    # compute the derivative of given order and take the square
    poly_der_sqr = poly
    for _ in range(optim_order):
        poly_der_sqr = ca.gradient(poly_der_sqr, t)
    poly_der_sqr = poly_der_sqr ** 2

    # Integrate the polynomial basis to get cost function
    c_poly_der = ca.poly_coeff(poly_der_sqr, t)[::-1]
    c_poly_der_int = ca.vertcat(0, ca.vertcat(*[c_poly_der[i] / (i + 1) for i in range(c_poly_der.shape[0])]))
    basis_int = ca.vertcat(*[t ** i for i in range(c_poly_der_int.shape[0])])
    int_exp = c_poly_der_int.T @ basis_int
    # Compute hessian of cost function to arrive at a quadratic form J = c.T @ Q @ c
    Q = ca.hessian(int_exp, c)[0]
    Q_f = ca.Function('Q', [t], [Q])
    basis_f = [ca.Function(f'basis{i}', [t], [basis_ders[i]]) for i in range(len(basis_ders))]

    # Put together Mosek optimization
    # For details about the formulation of the QP in Mosek, see https://docs.mosek.com/latest/pythonapi/tutorial-qo-shared.html
    res = np.zeros((4 * n, m - 1))

    for dim in range(4):
        task = mosek.Task()
        # Attach a log stream printer to the task
        if mosek_printing:
            task.set_Stream(mosek.streamtype.log, streamprinter)
        else:
            task.putintparam(mosek.iparam.log, 0)

        # Bound keys for variables
        bkx = n * (m - 1) * [mosek.boundkey.fr]

        # Bound values for variables
        blx = n * (m - 1) * [0]  # dummy value because bundkey is free
        bux = n * (m - 1) * [0]  # dummy value because bundkey is free

        # Objective: x.T @ Q @ x
        Q_lst = [np.array(Q_f(elem)) for elem in T_lst_rel[1:]]

        # Append nonzero elements of Q
        qsubi, qsubj, qval = [], [], []
        for i in range(m - 1):
            for j in range(n):
                for k in range(j + 1):
                    if abs(Q_lst[i][j, k]) > 1e-10:
                        qsubi.append(i * n + j)
                        qsubj.append(i * n + k)
                        qval.append(Q_lst[i][j, k])

        # Constraints: lb <= Ax <= ub
        # Below is the sparse representation of the A
        # matrix stored by row.
        asub, aval, bkc, buc, blc, buc = [], [], [], [], [], []

        # Continuity constraints
        for j in range(m - 2):  # joints between sections
            for der in range(continuity_order):
                asub.append([j * n + i for i in range(2 * n)])
                aval.append(np.array(basis_f[der](T_lst_rel[j + 1])).flatten().tolist() + (
                    -np.array(basis_f[der](0)).flatten()).tolist())
        bkc += (m - 2) * continuity_order * [mosek.boundkey.fx]
        blc += (m - 2) * continuity_order * [0.0]
        buc += (m - 2) * continuity_order * [0.0]

        # Waypoint constraints
        for j in range(m - 1):
            if j == 0 or j == m - 2:  # hard constraints for the endpoints
                for iter in range(2):
                    asub.append([j * n + i for i in range(n)])
                    aval.append(np.array(basis_f[0](iter * T_lst_rel[j + iter])).flatten().tolist())
                    bkc.append(mosek.boundkey.fx)
                    blc.append(constraints["pos"][j + iter][dim + 1])
                    buc.append(constraints["pos"][j + iter][dim + 1])
            else:  # soft constraints for interior points
                for iter in range(2):
                    asub.append([j * n + i for i in range(n)])
                    aval.append(np.array(basis_f[0](iter * T_lst_rel[j + iter])).flatten().tolist())
                    bkc.append(mosek.boundkey.ra)
                    blc.append(constraints["pos"][j + iter][dim + 1] - interior_point_prec)
                    buc.append(constraints["pos"][j + iter][dim + 1] + interior_point_prec)
            for iter in range(2):
                if constraints["vel"][j + iter] is not None:
                    asub.append([j * n + i for i in range(n)])
                    aval.append(np.array(basis_f[1](iter * T_lst_rel[j + iter])).flatten().tolist())
                    bkc.append(mosek.boundkey.fx)
                    blc.append(constraints["vel"][j + iter][dim + 1])
                    buc.append(constraints["vel"][j + iter][dim + 1])
                if constraints["acc"][j + iter] is not None:
                    asub.append([j * n + i for i in range(n)])
                    aval.append(np.array(basis_f[2](iter * T_lst_rel[j + iter])).flatten().tolist())
                    bkc.append(mosek.boundkey.fx)
                    blc.append(constraints["acc"][j + iter][dim + 1])
                    buc.append(constraints["acc"][j + iter][dim + 1])

        numvar = len(bkx)
        numcon = len(bkc)
        task.appendcons(numcon)
        task.appendvars(numvar)
        for j in range(numvar):
            task.putvarbound(j, bkx[j], blx[j], bux[j])
        for j in range(numcon):
            task.putconbound(j, bkc[j], blc[j], buc[j])
            task.putarow(j, asub[j], aval[j])
        task.putqobj(qsubi, qsubj, qval)
        task.putobjsense(mosek.objsense.minimize)

        task.optimize()  # Compute solution

        solsta = task.getsolsta(mosek.soltype.itr)
        if (solsta == mosek.solsta.optimal):
            xx = task.getxx(mosek.soltype.itr)  # Extract solution
            tm = task.getdouinf(mosek.dinfitem.optimizer_time)
            if optim_printing:
                print(f"Optimization solved successfully, solve time: {tm}")
            res[dim * n:(dim + 1) * n, :] = np.reshape(xx, (n, m - 1), order="F")
        elif (solsta == mosek.solsta.dual_infeas_cer or
              solsta == mosek.solsta.prim_infeas_cer):
            print("Primal or dual infeasibility certificate found.")
        elif solsta == mosek.solsta.unknown:
            print("Unknown solution status")
        else:
            print("Other solution status")
    return res


def find_optimal_poly(constraints, poly_deg=5, optim_order=2, continuity_order=2, interior_point_prec=0.01,
                      mosek_printing=False, optim_printing=False):
    # get time scaling factor
    T_lst = [pos[0] for pos in constraints["pos"]]
    m = len(T_lst)
    dt_lst = [T_lst[i + 1] - T_lst[i] for i in range(m - 1)]
    dt_mean = np.mean(dt_lst)
    # scale waypoint times, velocities, and accelerations
    for i in range(m):
        constraints["pos"][i][0] /= dt_mean
        if constraints["vel"][i] is not None:
            constraints["vel"][i][0] /= dt_mean
            for j in range(4):
                constraints["vel"][i][j + 1] *= dt_mean
        if constraints["acc"][i] is not None:
            constraints["acc"][i][0] /= dt_mean
            for j in range(4):
                constraints["acc"][i][j + 1] *= dt_mean ** 2

    for i in range(4):
        if "v_max" in constraints:
            constraints["v_max"][i] *= dt_mean
        if "a_max" in constraints:
            constraints["a_max"][i] *= dt_mean

    coeff = generate_trajectory(poly_deg=poly_deg,
                                optim_order=optim_order,
                                constraints=constraints,
                                continuity_order=continuity_order + 1,
                                interior_point_prec=interior_point_prec,
                                mosek_printing=mosek_printing,
                                optim_printing=optim_printing
                                )

    # descale coefficients
    for i in range(4):
        for j in range(poly_deg + 1):
            coeff[i * (poly_deg + 1) + j, :] *= dt_mean ** (-j)

    breakpoints = T_lst
    # generate scipy ppolys from coefficients and breakpoints
    polys = create_ppoly(breakpoints, coeff, poly_deg)
    return polys


def create_ppoly(breakpoints, coefficients, poly_deg):
    # generate scipy ppolys from coefficients and breakpoints
    polys: List[interpolate.PPoly] = []
    for dim in range(4):
        coeff = coefficients[dim * (poly_deg + 1):(dim + 1) * (poly_deg + 1), :]
        # casadi poly coeff order is reversed compared to scipy ppoly
        coeff = coeff[::-1, :]
        polys += [interpolate.PPoly(coeff, breakpoints)]
    return polys