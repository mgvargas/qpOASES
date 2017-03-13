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

import rospy
import numpy as np
from qpoases import PyReturnValue as returnValue
from qpoases import PySQProblem as SQProblem
from qpoases import PyOptions as Options
from qpoases import PyPrintLevel as PrintLevel
from sensor_msgs.msg import JointState


def main():
    rospy.init_node('controller_2dof', anonymous=True)
    rospy.Rate(10)
    joint_pub = rospy.Publisher('joint_state', JointState, queue_size=100)
    rospy.sleep(0.2)

    # Setup data of QP.
    # Weights.  
    w1 = 1e-3
    w2 = 1e-3
    w3 = 1
    w4 = 1  # joint goal
    # Joint limits.
    q0_min = -0.05  
    q0_max = 0.05  
    q1_min = -0.15
    q1_max = 0.15  
    # Links
    l1 = 0.1  
    l2 = 0.2  
    l3 = 0.3  
    # Initial joint values.
    q0 = 0.02
    q1 = -0.15
    # Joint target.
    q_des = 0.75
    q0_des = 0.02
    q1_des = 0.05
    q0_goal = True
    q1_goal = False
    # Slack limits.
    e_max = 1000
    e0_max = 1000
    # Velocity limits.(+-)
    v0_max = 0.05
    v1_max = 0.1
    # Acceleration limits.(+-)
    a0_max = 0.05 * 0.5
    a1_max = 0.1 * 0.5
    # Others
    precision = 1e-3
    joint_precision = 5e-3
    p = 1
    pj = 1
    q_eef = l1 + l2 + l3 + q0 + q1
    error = p * (q_des - q_eef)
    vel_init = 0
    nWSR = np.array([100])

    # Acceleration
    a0_const = (v0_max - a0_max) / v0_max
    a1_const = (v1_max - a1_max) / v1_max

    example = SQProblem(4, 7)

    H = np.array([w1, 0.0, 0.0, 0.0,
                  0.0, w2, 0.0, 0.0,
                  0.0, 0.0, w3, 0.0,
                  0.0, 0.0, 0.0, w4]).reshape((4, 4))

    A = np.array([ 1.0, 1.0, 1.0, 0.0,
                   1.0, 0.0, 0.0, 0.0,
                   0.0, 1.0, 0.0, 0.0,
                   1.0, 0.0, 0.0, 0.0,
                   0.0, 1.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0,
                   0.0, 0.0, 0.0, 0.0]).reshape((7, 4))

    g = np.array([0.0, 0.0, 0.0, 0.0])
    lb = np.array([-v0_max, -v1_max, -e_max, -e0_max])
    ub = np.array([v0_max, v1_max, e_max, e0_max])
    lbA = np.array([error, (q0_min - q0), (q1_min - q0), -a0_max, -a1_max, 0, 0])
    ubA = np.array([error, (q0_max - q0), (q1_max - q0), a0_max, a1_max, 0, 0])

    # Setting up QProblem object.
    if q0_goal is False and q1_goal is False:
        print("\nNo joint goal specified\n")
    elif (q0_goal is True and q1_goal is False) or (q0_goal is False and q1_goal is True):
        if q0_goal is True:
            lbA[5] = (q0_des-q0)
            ubA[5] = (q0_des-q0)
            A[5, 0] = 1.0
            A[5, 3] = 1.0
        else:
            lbA[5] = (q1_des-q1)
            ubA[5] = (q1_des-q1)
            A[5, 1] = 1.0
            A[5, 3] = 1.0
    else:
        A[0, 0] = 0.0
        A[0, 1] = 0.0
        A[0, 2] = 0.0
        A[5, 0] = 1.0
        A[5, 2] = 1.0
        A[6, 1] = 1.0
        A[6, 3] = 1.0
        p = 0
        error = 0
        lbA[0] = 0
        ubA[0] = 0

    options = Options()
    options.setToReliable()
    options.printLevel = PrintLevel.LOW
    example.setOptions(options)

    print("Init pos = %g,  goal = %g, error = %g, q0 =%g, q1 = %g\n" %
          (q_eef, q_des, error, q0, q1))
    print A

    i = 0
    limit = abs(error)
    first = True
    ok = False
    Opt = np.zeros(4)

    # ROS Message definition
    vel_pos = JointState()
    vel_pos.header.frame_id = "q0"  
    vel_pos.position.append(q0)
    vel_pos.position.append(q1)
    vel_pos.position.append(q_eef)
    vel_pos.velocity.append(vel_init)
    vel_pos.effort.append(error)
    vel_pos.effort.append(0)
    vel_pos.effort.append(lbA[4])
    vel_pos.effort.append(ubA[4])
    vel_pos.header.stamp = rospy.Time.now()
    rospy.sleep(0.2)

    return_value = example.init(H, g, A, lb, ub, lbA, ubA, nWSR)

    if return_value != returnValue.SUCCESSFUL_RETURN:
        print "Init of QP-Problem returned without success! ERROR MESSAGE: "
        return -1

    while not rospy.is_shutdown():
            while (limit > precision or not ok) and i < 2000:
                if first is True:
                    joint_pub.publish(vel_pos)
                else:
                    # Solve QP.
                    i += 1
                    nWSR = np.array([100])
                    lbA[0] = error
                    lbA[1] = q0_min - q0
                    lbA[2] = q1_min - q1
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
                    q0 += Opt[0] /100
                    q1 += Opt[1] /100
                    q_eef = l1 + l2 + l3 + q0 + q1
                    error = p * (q_des - q_eef)
                    limit = abs(error)

                    print "\nOpt = [ %g, %g, %g, %g ] \n posit= %g, error= %g, q0= %g q1= %g \n" % (
                    Opt[0], Opt[1], Opt[2], Opt[3], q_eef, error, q0, q1)

                    # Depending on joint goals
                    if q0_goal is False and q1_goal is False:
                        ok = True
                        error = 0
                        limit = 0
                    elif q0_goal is True and q1_goal is False:
                        lbA[5] = pj * (q0_des - q0)
                        ubA[5] = pj * (q0_des - q0)
                        if abs(lbA[5]) < joint_precision:
                            ok = True
                            print "\n q0_error = %g, i = %g \n" % (lbA[5], i)
                        else:
                            ok = False
                    elif q0_goal is False and q1_goal is True:
                        lbA[5] = pj * (q1_des - q1)
                        ubA[5] = pj * (q1_des - q1)
                        if abs(lbA[5]) < joint_precision:
                            ok = True
                            print "\n q0_error = %g, i = %g \n" % (lbA[5], i)
                    else:
                        lbA[5] = pj * (q0_des - q0)
                        ubA[5] = pj * (q0_des - q0)
                        lbA[6] = pj * (q1_des - q1)
                        ubA[6] = pj * (q1_des - q1)
                        if abs(lbA[5]) < joint_precision and abs(lbA[6]) < joint_precision:
                            ok = True
                            print "\n q0_error = %g, q1_error = %g \n" % (lbA[5], lbA[6])

                    vel_pos.position[0] = q0
                    vel_pos.position[1] = q1
                    vel_pos.position[2] = q_eef
                    vel_pos.velocity[0] = Opt[0]
                    vel_pos.effort[0] = error
                    vel_pos.effort[1] = Opt[1]
                    vel_pos.effort[2] = lbA[4]
                    vel_pos.effort[3] = ubA[4]
                    vel_pos.header.stamp += rospy.Duration(0.01)

                    joint_pub.publish(vel_pos)

                first = False
            print ok
            return 0


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

# PLOT: rqt_plot /joint_state/position[0] /joint_state/velocity[0] /joint_state/effort[0] /joint_state/effort[1]
