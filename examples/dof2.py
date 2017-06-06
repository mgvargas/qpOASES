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
    w2 = 1e-5
    w3 = 1  # slack (attractor)
    # Joint limits.
    q0_max = 0.05
    q1_max = 0.15
    # Links
    l1 = 0.1
    l2 = 0.2
    l3 = 0.3
    # Initial joint values.
    q0_init = q0 = -0.04
    q1_init = q1 = -0.08
    # Joint target.
    q_des = 0.65
    # Slack limits.
    e_max = 1000
    # Velocity limits.(+-)
    v0_max = 0.05
    v1_max = 0.1
    # Acceleration limits.(+-)
    a0_max = 0.05 * 0.5
    a1_max = 0.1 * 0.5
    # Others
    precision = 1e-3
    p = 10
    q_eef_init = q_eef = l1 + l2 + l3 + q0 + q1
    error = p * (q_des - q_eef)
    vel_init = 0
    nWSR = np.array([100])

    # Acceleration
    a0_const = (v0_max - a0_max) / v0_max
    a1_const = (v1_max - a1_max) / v1_max

    example = SQProblem(3, 5)

    H = np.array([w1, 0.0, 0.0,
                  0.0, w2, 0.0,
                  0.0, 0.0, w3]).reshape((3, 3))

    A = np.array([1.0, 1.0, 1.0,
                  1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0]).reshape((5, 3))

    g = np.array([0.0, 0.0, 0.0, 0.0])
    lb = np.array([-v0_max, -v1_max, -e_max])
    ub = np.array([v0_max, v1_max, e_max])
    lbA = np.array([error, (-q0_max - q0), (-q1_max - q0), -a0_max, -a1_max])
    ubA = np.array([error, (q0_max - q0), (q1_max - q0), a0_max, a1_max])

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
    Opt = np.zeros(3)

    # Plotting
    t = np.array(i)
    pos_eef = np.array(q_eef)
    pos_0 = np.array(q0)
    pos_1 = np.array(q1)
    vel_0 = np.array(vel_init)
    vel_1 = np.array(vel_init)
    vel_eef = np.array(vel_init)
    p_error = np.array(error)
    goal = np.array(q_des)
    lim1 = np.array(q0_max)
    lim2 = np.array(q1_max)

    return_value = example.init(H, g, A, lb, ub, lbA, ubA, nWSR)

    if return_value != returnValue.SUCCESSFUL_RETURN:
        print "Init of QP-Problem returned without success! ERROR MESSAGE: "
        return -1

    while not rospy.is_shutdown():
        while limit > precision and i < 400:
            # Solve QP.
            i += 1
            nWSR = np.array([100])
            lbA[0] = error
            lbA[1] = -q0_max - q0
            lbA[2] = -q1_max - q1
            lbA[3] = a0_const * Opt[0] - a0_max
            lbA[4] = a1_const * Opt[1] - a1_max
            ubA[0] = error
            ubA[1] = q0_max - q0
            ubA[2] = q1_max - q1
            ubA[3] = a0_const * Opt[0] + a0_max
            ubA[4] = a1_const * Opt[1] + a1_max

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

            # print "\nOpt = [ %g, %g, %g, %g ] \n posit= %g, error= %g, q0= %g q1= %g \n" % (
            #    Opt[0], Opt[1], Opt[2], Opt[3], q_eef, error, q0, q1)

            # Plotting arrays
            pos_eef = np.hstack((pos_eef, q_eef))
            pos_0 = np.hstack((pos_0, q0))
            pos_1 = np.hstack((pos_1, q1))
            vel_0 = np.hstack((vel_0, Opt[0]))
            vel_1 = np.hstack((vel_1, Opt[1]))
            vel_eef = np.hstack((vel_eef, Opt[0]+Opt[1]))
            goal = np.hstack((goal, q_des))
            p_error = np.hstack((p_error, error))
            t = np.hstack((t, i))
            lim1 = np.hstack((lim1, q0_max))
            lim2 = np.hstack((lim2, q1_max))

        # Plot
        t_eef = go.Scatter(
            y=pos_eef, x=t, marker=dict(size=4,),
            mode='lines+markers', name='pos_eef')
        t_p0 = go.Scatter(
            y=pos_0, x=t, marker=dict(size=4,),
            mode='lines+markers', name='pos_q0')
        t_p1 = go.Scatter(
            y=pos_1, x=t, marker=dict(size=4,),
            mode='lines+markers', name='pos_q1')
        t_v0 = go.Scatter(
            y=vel_0, x=t, marker=dict(size=4,),
            mode='lines+markers', name='vel_q0')
        t_v1 = go.Scatter(
            y=vel_1, x=t, marker=dict(size=4,),
            mode='lines+markers', name='vel_q1')
        t_q0 = go.Scatter(
            y=lim1, x=t, marker=dict(size=4,),
            mode='lines', name='limit_0')
        t_q1 = go.Scatter(
            y=lim2, x=t, marker=dict(size=4,),
            mode='lines', name='limit_1')
        t_veef = go.Scatter(
            y=vel_eef, x=t, marker=dict(size=4, ),
            mode='lines+markers', name='vel_eef')
        t_er = go.Scatter(
            y=p_error, x=t, marker=dict(size=4,),
            mode='lines+markers', name='error')
        t_goal = go.Scatter(
            y=goal, x=t, marker=dict(size=4, ),
            mode='lines', name='goal')

        print "\n i = %g \n" % (i)

        data_eef = [t_eef, t_veef, t_goal]
        layout_eef = dict(title="Initial position EEF = %g.  Goal = %g, \n" %
                            (q_eef_init, q_des),
                      xaxis=dict(title='Iterations',autotick=False,dtick=25,gridwidth=2,),
                      yaxis=dict(title='Position / Velocity'), )
        fig = dict(data=data_eef, layout=layout_eef)
        plotly.offline.plot(fig, filename='basic_example_eef.html')

        # data = [t_p0, t_p1, t_v0, t_v1, t_q0, t_q1]
        data = [t_p0, t_p1, t_v0, t_v1]
        layout = dict(title="Initial position q0 =%g, q1 = %g.\n" %
                            (q0_init, q1_init),
                      xaxis=dict(title='Iterations',autotick=False,dtick=25,gridwidth=2,),
                      yaxis=dict(title='Position / Velocity'),
                      )
        fig = dict(data=data, layout=layout)
        plotly.offline.plot(fig, filename='basic_example_joints.html')

        return 0


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass