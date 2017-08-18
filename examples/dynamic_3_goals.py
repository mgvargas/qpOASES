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

# This program simulates a 2 DOF controller with 2 possible goals for the EEF with dynamic weights

import rospy
import numpy as np
from qpoases import PyReturnValue as returnValue
from qpoases import PySQProblem as SQProblem
from qpoases import PyOptions as Options
from qpoases import PyPrintLevel as PrintLevel
import plotly
import plotly.graph_objs as go

# The range of the EEF is: 0.6+-0.2

def main():
    rospy.init_node('controller_2dof', anonymous=True)
    rospy.Rate(10)

    # Setup data of QP.
    # Joint Weights.
    w1 = 1e-3
    w2 = 1e-3
    # Joint limits.
    q0_max = 0.05
    q1_max = 0.15
    # Links
    l1 = 0.1
    l2 = 0.2
    l3 = 0.25
    # Initial joint values.
    q0_init = q0 = -0.03
    q1_init = q1 = 0.0
    # Joint target.
    q_des1 = 0.4
    q_des2 = 0.7
    q_des3 = 0.8
    # Slack limits.
    e1_max = 1000
    e2_max = 1000
    e3_max = 1000
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
    error1 = p * (q_des1 - q_eef)
    error2 = p * (q_des2 - q_eef)
    error3 = p * (q_des3 - q_eef)
    vel_init = 0
    nWSR = np.array([100])
    # Dynamic goal weights
    sum_error = abs(error1) + abs(error2) + abs(error3)
    min_error = min(abs(error1), abs(error2), abs(error3))
    max_error = max(abs(error1), abs(error2), abs(error3))
    w3 = (1 - (abs(error1) - min_error) / (max_error-min_error)) ** 3  # goal 1 weight
    w4 = (1 - (abs(error2) - min_error) / (max_error-min_error)) ** 3  # goal 2 weight
    w5 = (1 - (abs(error3) - min_error) / (max_error-min_error)) ** 3  # goal 3 weight


    # Acceleration
    a0_const = (v0_max - a0_max) / v0_max
    a1_const = (v1_max - a1_max) / v1_max

    example = SQProblem(5, 7)

    H = np.array([w1, 0.0, 0.0, 0.0, 0.0,
                  0.0, w2, 0.0, 0.0, 0.0,
                  0.0, 0.0, w3, 0.0, 0.0,
                  0.0, 0.0, 0.0, w4, 0.0,
                  0.0, 0.0, 0.0, 0.0, w5]).reshape((5, 5))

    A = np.array([1.0, 1.0, 1.0, 0.0, 0.0,
                  1.0, 1.0, 0.0, 1.0, 0.0,
                  1.0, 1.0, 0.0, 0.0, 1.0,
                  1.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 1.0, 0.0, 0.0, 0.0,
                  1.0, 0.0, 0.0, 0.0, 0.0,
                  0.0, 1.0, 0.0, 0.0, 0.0]).reshape((7, 5))

    g = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    lb = np.array([-v0_max, -v1_max, -e1_max, -e2_max, -e3_max])
    ub = np.array([v0_max, v1_max, e1_max, e2_max, e3_max])
    lbA = np.array([error1, error2, error3, (-q0_max - q0), (-q1_max - q0), -a0_max, -a1_max])
    ubA = np.array([error1, error2, error3, (q0_max - q0), (q1_max - q0), a0_max, a1_max])

    # Setting up QProblem object
    options = Options()
    options.setToReliable()
    options.printLevel = PrintLevel.LOW
    example.setOptions(options)
    Opt = np.zeros(5)

    print("Init pos = %g,  goal_1 = %g, goal_2 = %g, error_1 = %g, error_2 = %g, q0 =%g, q1 = %g\n" %
          (q_eef, q_des1, q_des2, error1, error2, q0, q1))
    print 'A = {}\n'.format(A)

    i = 0
    # Stopping conditions
    limit1 = abs(error1)
    limit2 = abs(error2)
    limit3 = abs(error3)
    lim = min([limit1, limit2, limit3])
    diff1 = abs(error1)
    diff2 = abs(error2)
    diff3 = abs(error3)
    diff = min([diff1, diff2, diff3])

    # Plotting
    t = np.array(i)
    pos_eef = np.array(q_eef)
    pos_0 = np.array(q0)
    pos_1 = np.array(q1)
    vel_0 = np.array(vel_init)
    vel_1 = np.array(vel_init)
    vel_eef = np.array(vel_init)
    p_error = np.array(error1)
    go_1 = np.array(q_des1)
    go_2 = np.array(q_des2)
    go_3 = np.array(q_des3)

    return_value = example.init(H, g, A, lb, ub, lbA, ubA, nWSR)

    if return_value != returnValue.SUCCESSFUL_RETURN:
        print "Init of QP-Problem returned without success! ERROR MESSAGE: "
        return -1

    while not rospy.is_shutdown():
        while (diff > 0.0009) and lim > precision and i < 400:

            # Solve QP.
            i += 1
            nWSR = np.array([100])
            lbA[0] = error1
            lbA[1] = error2
            lbA[2] = error3
            lbA[3] = -q0_max - q0
            lbA[4] = -q1_max - q1
            lbA[5] = a0_const * Opt[0] - a0_max
            lbA[6] = a1_const * Opt[1] - a1_max
            ubA[0] = error1
            ubA[1] = error2
            ubA[2] = error3
            ubA[3] = q0_max - q0
            ubA[4] = q1_max - q1
            ubA[5] = a0_const * Opt[0] + a0_max
            ubA[6] = a1_const * Opt[1] + a1_max

            return_value = example.hotstart(H, g, A, lb, ub, lbA, ubA, nWSR)

            if return_value != returnValue.SUCCESSFUL_RETURN:
                print "Hotstart of QP-Problem returned without success! ERROR MESSAGE: "
                return -1

            old_error1 = error1
            old_error2 = error2
            old_error3 = error3

            example.getPrimalSolution(Opt)

            # Update limits
            q0 += Opt[0] / 100
            q1 += Opt[1] / 100
            q_eef = l1 + l2 + l3 + q0 + q1
            error1 = p * (q_des1 - q_eef)
            error2 = p * (q_des2 - q_eef)
            error3 = p * (q_des3 - q_eef)
            # Update weights
            min_error = min(abs(error1), abs(error2), abs(error3))
            max_error = max(abs(error1), abs(error2), abs(error3))
            w3 = (1 - (abs(error1) - min_error) / (max_error - min_error)) ** 3  # goal 1
            w4 = (1 - (abs(error2) - min_error) / (max_error - min_error)) ** 3  # goal 2
            w5 = (1 - (abs(error3) - min_error) / (max_error - min_error)) ** 3  # goal 3

            H = np.array([w1, 0.0, 0.0, 0.0, 0.0,
                          0.0, w2, 0.0, 0.0, 0.0,
                          0.0, 0.0, w3, 0.0, 0.0,
                          0.0, 0.0, 0.0, w4, 0.0,
                          0.0, 0.0, 0.0, 0.0, w5]).reshape((5, 5))

            # Stopping conditions
            limit1 = abs(error1)
            limit2 = abs(error2)
            limit3 = abs(error3)
            lim = min([limit1, limit2, limit3])
            diff1 = abs(error1-old_error1)
            diff2 = abs(error2-old_error2)
            diff3 = abs(error3-old_error3)
            diff = min([diff1, diff2, diff3])

            # print 'weights= [ %g, %g, %g ]'%(w3, w4, w5)
            # print 'vel = [ %g, %g ], error = [ %g, %g, %g ] \n'% (Opt[0], Opt[1], error1, error2, error3)

            # Plotting arrays
            pos_eef = np.hstack((pos_eef, q_eef))
            pos_0 = np.hstack((pos_0, q0))
            pos_1 = np.hstack((pos_1, q1))
            vel_0 = np.hstack((vel_0, Opt[0]))
            vel_1 = np.hstack((vel_1, Opt[1]))
            vel_eef = np.hstack((vel_eef, Opt[0]+Opt[1]))
            p_error = np.hstack((p_error, error1))
            go_1 = np.hstack((go_1, q_des1))
            go_2 = np.hstack((go_2, q_des2))
            go_3 = np.hstack((go_3, q_des3))
            t = np.hstack((t, i))

        # Plot
        t_eef = go.Scatter(
            y=pos_eef, x=t, marker=dict(size=4,),
            mode='lines', name='pos_eef', line = dict(dash = 'dash'))
        t_p0 = go.Scatter(
            y=pos_0, x=t, marker=dict(size=4,),
            mode='lines', name='pos_0', line = dict(dash = 'dash'))
        t_p1 = go.Scatter(
            y=pos_1, x=t, marker=dict(size=4,),
            mode='lines', name='pos_1', line = dict(dash = 'dash'))
        t_v0 = go.Scatter(
            y=vel_0, x=t, marker=dict(size=4,),
            mode='lines', name='vel_0')
        t_v1 = go.Scatter(
            y=vel_1, x=t, marker=dict(size=4,),
            mode='lines', name='vel_1')
        t_er = go.Scatter(
            y=p_error, x=t, marker=dict(size=4,),
            mode='lines', name='error')
        t_veef = go.Scatter(
            y=vel_eef, x=t, marker=dict(size=4, ),
            mode='lines', name='vel_eef')
        t_g1 = go.Scatter(
            y=go_1, x=t, marker=dict(size=4,),
            mode='lines', name='goal_1', line = dict(dash = 'dot'))
        t_g2 = go.Scatter(
            y=go_2, x=t, marker=dict(size=4,),
            mode='lines', name='goal_2', line = dict(dash = 'dot', width = 4))
        t_g3 = go.Scatter(
            y=go_3, x=t, marker=dict(size=4, ),
            mode='lines', name='goal_3', line = dict(dash = 'dot', width = 6))

        data = [t_eef, t_veef, t_g1, t_g2, t_g3]
        layout = dict(title="Initial position eef = %g. Goals: goal_1 = %g, goal_2 = %g, goal_3 = %g\n" %
                      (q_eef_init, q_des1, q_des2, q_des3),
                      xaxis=dict(title='Iterations',autotick=False,dtick=25,gridwidth=2,),
                      yaxis=dict(title='Position / Velocity'), font=dict(size=18),
                      )
        fig = dict(data=data, layout=layout)

        plotly.offline.plot(fig, filename='html/dynamic_3_weights_EEF.html', image='png', image_filename='dyn_3_eef_2')

        data = [t_p0, t_v0]
        layout = dict(title="Initial position q0 =%g.\n" %(q0_init),
                      xaxis=dict(title='Iterations',autotick=False,dtick=25,gridwidth=2),
                      yaxis=dict(title='Position / Velocity',range=[-0.11,.045],), font=dict(size=18),
                      )
        fig = dict(data=data, layout=layout)
        plotly.offline.plot(fig, filename='html/dynamic_3_weights_q0.html')

        data = [t_p1, t_v1]
        layout = dict(title="Initial position q1 = %g.\n" %(q1_init),
                      xaxis=dict(title='Iterations',autotick=False,dtick=25,gridwidth=2),
                      yaxis=dict(title='Position / Velocity',range=[-0.11,.045],), font=dict(size=18),
                      )
        fig = dict(data=data, layout=layout)
        plotly.offline.plot(fig, filename='html/dynamic_3_weights__q1.html')

        print "\n i = %g \n" % i
        return 0


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
