#!/usr/bin/env python
# Copyright (c) 2017 Universitaet Bremen - Institute for Artificial Intelligence (Prof. Beetz)
#
# Author: Minerva Gabriela Vargas Gleason <minervavargasg@gmail.com>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY   without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.

# This program simulates a 2 DOF controller with multiple goals

import rospy
import numpy as np
from qpoases import PyReturnValue as returnValue
from qpoases import PySQProblem as SQProblem
from qpoases import PyOptions as Options
from qpoases import PyPrintLevel as PrintLevel
import plotly
import plotly.graph_objs as go


def main():
    rospy.init_node('controller_2dof', anonymous=True)
    rospy.Rate(10)
    rospy.sleep(0.2)

    # Setup data of QP.
    # Weights.
    w1 = 1e-3
    w2 = 1e-3
    w3 = 1e-2  # goal 1 weight
    w4 = 10  # goal 2 weight
    # Joint limits.
    q0_max = 0.05
    q1_max = 0.15
    # Links
    l1 = 0.1
    l2 = 0.2
    l3 = 0.3
    # Initial joint values.
    q0 = 0.03
    q1 = 0.12
    # Joint target.
    q_des = 0.6
    q_des_min = 0.57
    q_des_max = 0.62
    # Slack limits.
    e_max = 1000
    e_lim_max = 1000
    # Velocity limits.(+-)
    v0_max = 0.05
    v1_max = 0.1
    # Acceleration limits.(+-)
    a0_max = 0.05 * 0.5
    a1_max = 0.1 * 0.5
    # Others
    precision = 1e-3
    p = 10
    p_range = 2
    q_eef = l1 + l2 + l3 + q0 + q1
    error = p * (q_des - q_eef)
    vel_init = 0
    nWSR = np.array([100])

    # Acceleration
    a0_const = (v0_max - a0_max) / v0_max
    a1_const = (v1_max - a1_max) / v1_max

    example = SQProblem(4, 6)

    H = np.array([w1, 0.0, 0.0, 0.0,
                  0.0, w2, 0.0, 0.0,
                  0.0, 0.0, w3, 0.0,
                  0.0, 0.0, 0.0, w4]).reshape((4, 4))

    A = np.array([1.0, 1.0, 1.0, 0.0,
                  1.0, 0.0, 0.0, 0.0,
                  0.0, 1.0, 0.0, 0.0,
                  1.0, 0.0, 0.0, 0.0,
                  0.0, 1.0, 0.0, 0.0,
                  0.0, 0.0, 0.0, 1.0]).reshape((6, 4))

    g = np.array([0.0, 0.0, 0.0, 0.0])
    lb = np.array([-v0_max, -v1_max, -e_max, -e_lim_max])
    ub = np.array([v0_max, v1_max, e_max, e_lim_max])
    lbA = np.array([error, (-q0_max - q0), (-q1_max - q0), -a0_max, -a1_max, (q_des_min - q_eef)])
    ubA = np.array([error, (q0_max - q0), (q1_max - q0), a0_max, a1_max, (q_des_max - q_eef)])

    # Setting up QProblem object
    options = Options()
    options.setToReliable()
    options.printLevel = PrintLevel.LOW
    example.setOptions(options)

    print("Init pos = %g,  goal = %g, error = %g, q0 =%g, q1 = %g\n" %
          (q_eef, q_des, error, q0, q1))
    print A

    i = 0
    n = 0
    limit = abs(error)
    ok = False
    Opt = np.zeros(4)

    # Plotting
    t = np.array(i)
    pos_eef = np.array(q_eef)
    pos_0 = np.array(q0)
    pos_1 = np.array(q1)
    vel_0 = np.array(vel_init)
    vel_1 = np.array(vel_init)
    p_error = np.array(error)
    r_max = np.array(q_des_max)
    r_min = np.array(q_des_min)

    return_value = example.init(H, g, A, lb, ub, lbA, ubA, nWSR)

    if return_value != returnValue.SUCCESSFUL_RETURN:
        print "Init of QP-Problem returned without success! ERROR MESSAGE: "
        return -1

    while not rospy.is_shutdown():
        # while (limit > precision or not ok) and i < 400:
        while n < 40 and i < 400:
            # Solve QP.
            i += 1
            nWSR = np.array([100])
            lbA[0] = error
            lbA[1] = -q0_max - q0
            lbA[2] = -q1_max - q1
            lbA[3] = a0_const * Opt[0] - a0_max
            lbA[4] = a1_const * Opt[1] - a1_max
            lbA[5] = p_range * (q_des_min - q_eef)
            ubA[0] = error
            ubA[1] = q0_max - q0
            ubA[2] = q1_max - q1
            ubA[3] = a0_const * Opt[0] + a0_max
            ubA[4] = a1_const * Opt[1] + a1_max
            ubA[5] = p_range * (q_des_max - q_eef)

            return_value = example.hotstart(H, g, A, lb, ub, lbA, ubA, nWSR)

            if return_value != returnValue.SUCCESSFUL_RETURN:
                print "Hotstart of QP-Problem returned without success! ERROR MESSAGE: "
                return -1

            # Get and  print solution of QP.
            example.getPrimalSolution(Opt)
            q0 += Opt[0] / 100
            q1 += Opt[1] / 100
            q_eef = l1 + l2 + l3 + q0 + q1
            error = p * (q_des - q_eef)
            limit = abs(error)

            print "\nOpt = [ %g, %g, %g, %g ] \n posit= %g, error= %g, q0= %g q1= %g \n" % (
                Opt[0], Opt[1], Opt[2], Opt[3], q_eef, error, q0, q1)

            # Depending on joint goals
            if q_des_min <= q_eef <= q_des_max:
                ok = True
                n += 1
            else:
                ok = False

            # Plotting arrays
            pos_eef = np.hstack((pos_eef, q_eef))
            pos_0 = np.hstack((pos_0, q0))
            pos_1 = np.hstack((pos_1, q1))
            vel_0 = np.hstack((vel_0, Opt[0]))
            vel_1 = np.hstack((vel_1, Opt[1]))
            p_error = np.hstack((p_error, error))
            r_max = np.hstack((r_max, q_des_max))
            r_min = np.hstack((r_min, q_des_min))
            t = np.hstack((t, i))

        # Plot
        t_eef = go.Scatter(
            y=pos_eef, x=t,
            mode='lines+markers', name='pos_eef')
        t_p0 = go.Scatter(
            y=pos_0, x=t,
            mode='lines+markers', name='pos_0')
        t_p1 = go.Scatter(
            y=pos_1, x=t,
            mode='lines+markers', name='pos_1')
        t_v0 = go.Scatter(
            y=vel_0, x=t,
            mode='lines+markers', name='vel0')
        t_v1 = go.Scatter(
            y=vel_1, x=t,
            mode='lines+markers', name='vel_1')
        t_er = go.Scatter(
            y=p_error, x=t,
            mode='lines+markers', name='error')
        t_min = go.Scatter(
            y=r_min, x=t,
            mode='lines', name='min')
        t_max = go.Scatter(
            y=r_max, x=t,
            mode='lines', name='max')

        data = [t_eef, t_p0, t_p1, t_v0, t_v1, t_min, t_max]
        plotly.offline.plot(data, filename='goal_range.html')
        print "\n ok = %g, i = %g \n" % (ok, i)
        return 0


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
