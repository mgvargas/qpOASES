/*
 *	This file is part of qpOASES.
 *
 *	qpOASES -- An Implementation of the Online Active Set Strategy.
 *	Copyright (C) 2007-2015 by Hans Joachim Ferreau, Andreas Potschka,
 *	Christian Kirches et al. All rights reserved.
 *
 *	qpOASES is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU Lesser General Public
 *	License as published by the Free Software Foundation; either
 *	version 2.1 of the License, or (at your option) any later version.
 *
 *	qpOASES is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *	See the GNU Lesser General Public License for more details.
 *
 *	You should have received a copy of the GNU Lesser General Public
 *	License along with qpOASES; if not, write to the Free Software
 *	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */


#include <qpOASES.hpp>
#include <cmath>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"


/** Example for obtaining the velocity for 1DOF using the QProblem class. */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_1dof");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_state", 10);
    ros::Rate loop_rate(10);
    ros::Duration(0.5).sleep();

    USING_NAMESPACE_QPOASES;

    /* Setup data of first QP. */
    float w1 = 1;            /* Weights. */
    float w2 = 1e-4;
    float q_min = -1;           /* Joint limits. */
    float q_max = 1;
    float q0 = -0.8;             /* Initial joint value. */
    float q_des = 0.7;         /* Joint target. */
    float e_min = -20;          /* Slack limits. */
    float e_max = 20;
    float v_min = -0.4;         /* Velocity limits. */
    float v_max = 0.4;
    float precision= 1e-4;
    float error = q_des-q0;
    float vel_init = 0;

    real_t H[2*2] = { w1, 0.0, 0.0, w2 };
    real_t A[2*2] = { 1.0, 1.0, 1.0, 0.0 };
    real_t g[2] = { 0.0, 0.0 };
    real_t lb[2] = { v_min, e_min };
    real_t ub[2] = { v_max, e_max };
    real_t lbA[2] = { error, (q_min - q0) };
    real_t ubA[2] = { error, (q_max - q0) };

    /* Setting up QProblem object. */
    QProblem example( 2,2 );

    Options options;
    //options.enableRegularisation = BT_TRUE;
    example.setOptions( options );
    int nWSR = 10;
    real_t Opt[2];


    printf( "Init pos = %f,  goal = %f, error = %f, w1 =%f, w2 = %f\n",
            q0,q_des,error,H[0],H[3]);

    int i = 0;
    float limit = std::abs(error);;

    sensor_msgs::JointState vel_pos;
    vel_pos.header.frame_id = "q0";
    vel_pos.position.push_back(q0);
    vel_pos.velocity.push_back(vel_init);
    vel_pos.effort.push_back(error);
    vel_pos.effort.push_back(0);
    vel_pos.header.stamp  = ros::Time::now();
    loop_rate.sleep();

    float cost;
    float condition;
    bool first = true;

    while (ros::ok()) {
        do {

            if (first == true) {
                printf ("\n Hi %f \n",q0);
                joint_pub.publish(vel_pos);
            }
            else {
                /* Solve QP. */
                ++i;
                lbA[0] = error;
                lbA[1] = q_min - q0;
                ubA[0] = error;
                ubA[0] = q_max - q0;
                example.setOptions( options );
                example.init( H,g,A,lb,ub,lbA,ubA, nWSR );

                /* Get and print solution of second QP. */
                example.getPrimalSolution( Opt );
                q0 = q0 + Opt[0];
                error = q_des-q0;
                limit = std::abs(error);

                cost = w1*Opt[0]*Opt[0] + w2*Opt[1]*Opt[1];
                condition = Opt[0]*A[0] + Opt[1]*A[1];

                printf( "\nOpt = [ %e, %e ];  posit = %f, error = %f, \n cost = %e, c1 = %e \n",
                        Opt[0],Opt[1],q0,error,cost,condition);

                vel_pos.position[0] = q0;
                vel_pos.velocity[0] = Opt[0];
                vel_pos.effort[0] = error;
                vel_pos.effort[1] = condition;
                vel_pos.header.stamp  = ros::Time::now();

                joint_pub.publish(vel_pos);
                example.reset();
            }
            first = false;
            loop_rate.sleep();
            ros::spinOnce();

        }
        while ( limit>precision &&  i<30 );

        return 0;
    }
}

/* PLOT: rqt_plot /joint_state/position[0] /joint_state/velocity[0]
NOTES:
- Precision HIGH (1e-4)     VEL != ERROR
   - w1 LOW w2 LOW: Error decreases, but before reaching 0, it grows again. IF q0 is +, VEL = ERROR
   - w1 LOW w2 1:   Correct, eps almost 0 (sometimes).
   - w1 LOW w2 low: Correct, eps almost 0.
   - w1 low w2 1:   Correct, eps almost 0.
   - w1 low w2 10:  Position keeps changing, error decreases, increases, stays constant.
   - w1 1   w2 10:  Position keeps changing, error decreases, increases, stays constant.
   - w1 1   w2 1:   Correct, error slowly converges to 0.
   - w1 1   w2 LOW: Error decreases, but before reaching 0, it grows again. vel almost 0


*/

