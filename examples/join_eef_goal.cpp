#include <qpOASES.hpp>
#include <cmath>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"


/** Example for obtaining the velocity for 1DOF using the QProblem class. */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "controller_1dof");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_state", 100);
    ros::Rate loop_rate(10);
    ros::Duration(0.5).sleep();

    USING_NAMESPACE_QPOASES;

    /* Setup data of first QP. */
    /* Weights. */
    float w1 = 1e-6;
    float w2 = 1e-4;
    float w3 = 10;
    /* Joint limits. */
    float q0_min = -0.05;
    float q0_max = 0.05;
    float q1_min = -0.15;
    float q1_max = 0.15;
    /* Links */
    float l1 = 0.1;
    float l2 = 0.2;
    float l3 = 0.3;
    /* Initial joint values. */
    float q0 = 0.02;
    float q1 = 0.05;
    /* Joint target. */
    float q_des = 0.7;
    float q0_des = 0.0;
    float q1_des = 0.1;
    bool q0_goal = true;
    bool q1_goal = false;
    /* Slack limits. */
    float e_min = -10000;
    float e_max = 10000;
    /* Velocity limits. (+-) */
    float v0_max = 0.05;
    float v1_max = 0.1;
    /* Acceleration limits. (+-) */
    float a0_max = 0.05*0.5;
    float a1_max = 0.1*0.5;
    /* Others */
    float precision= 1e-4;
    float p = 10;
    float q_eef = l1 +l2 +l3 + q0 + q1;
    float error = p*(q_des-q_eef);
    float vel_init = 0;
    float v0_old = 0;
    float v1_old = 0;

    int nWSR = 100;

    /* Acceleration */
    float a0_const = (v0_max-a0_max)/v0_max;
    float a1_const = (v1_max-a1_max)/v1_max;
    SQProblem example( 3,7 );

    real_t H[3*3] = { w1, 0.0, 0.0,  0.0, w2, 0.0,  0.0, 0.0, w3 };
    real_t A[7*3] = { 1.0, 1.0, 1.0,  1.0, 0.0, 0.0,  0.0, 1.0, 0.0,  1.0, 0.0, 0.0,  0.0, 1.0, 0.0,  0.0, 0.0, 0.0,  0.0, 0.0, 0.0 };
    real_t g[3] = { 0.0, 0.0, 0.0 };
    real_t lb[3] = { -v0_max, -v1_max, e_min };
    real_t ub[3] = { v0_max, v1_max, e_max };
    real_t lbA[7] = { error, (q0_min - q0), (q1_min - q0), -a0_max, -a1_max, 0, 0 };
    real_t ubA[7] = { error, (q0_max - q0), (q1_max - q0), a0_max, a1_max,  0, 0 };

    /* Setting up QProblem object. */
    if (q0_goal == false && q1_goal == false){
        printf ("\nNo joint goal specified\n");
    }
    else if ( (q0_goal == true && q1_goal == false) || (q0_goal == false && q1_goal == true)) {
        if (q0_goal== true) {
            lbA[5] = (q0_des-q0);
            ubA[5] = (q0_des-q0);
            A[15] = 1.0;
        }
        else {
            lbA[5] = (q1_des-q1);
            ubA[5] = (q1_des-q1);
            A[19] = 1.0;
        }
    }
    else {
        A[15] = 1.0;
        A[19] = 1.0;
    }
    Options options;
    options.setToReliable();
    options.printLevel = qpOASES::PL_HIGH;
    example.setOptions(options);


    printf( "Init pos = %g,  goal = %g, error = %g, q0 =%g, q1 = %g\n",
            q_eef,q_des,error,q0,q1);

    int i = 0;
    float limit = std::abs(error);;
    bool first = true;
    bool ok = false;
    returnValue return_value;
    real_t Opt[3];

    /* ROS Message definition */
    sensor_msgs::JointState vel_pos;
    vel_pos.header.frame_id = "q0";
    vel_pos.position.push_back(q_eef);
    vel_pos.velocity.push_back(vel_init);
    vel_pos.effort.push_back(error);
    vel_pos.effort.push_back(0);
    vel_pos.effort.push_back(lbA[4]);
    vel_pos.effort.push_back(ubA[4]);
    vel_pos.header.stamp  = ros::Time::now();
    loop_rate.sleep();

    return_value = example.init( H,g,A,lb,ub,lbA,ubA, nWSR );

    if(return_value != qpOASES::SUCCESSFUL_RETURN)
    {
              std::cout << "Init of QP-Problem returned without success! ERROR MESSAGE: " <<
                qpOASES::MessageHandling::getErrorCodeMessage(return_value) << std::endl;
              return -1;
    }


    while (ros::ok()) {
        do {

            if (first == true) {
                joint_pub.publish(vel_pos);
            }
            else {
                /* Solve QP. */
                ++i;
                nWSR = 100;
                lbA[0] = error;
                lbA[1] = q0_min - q0;
                lbA[2] = q1_min - q1;
                lbA[3] = a0_const*Opt[0]- a0_max;
                lbA[4] = a1_const*Opt[1]- a1_max;
                ubA[0] = error;
                ubA[1] = q0_max - q0;
                ubA[2] = q1_max - q1;
                ubA[3] = a0_const*Opt[0] + a0_max;
                ubA[4] = a1_const*Opt[1] + a1_max;



                printf( "\n lbA[5] = %g, lbA[6] = %g",lbA[5],lbA[6]);
                printf( "\n ubA[5] = %g, ubA[6] = %g \n",ubA[5],ubA[6]);

                return_value = example.hotstart( H,g,A,lb,ub,lbA,ubA, nWSR );

                if(return_value != qpOASES::SUCCESSFUL_RETURN)
                {
                          std::cout << "Hotstart of QP-Problem returned without success! ERROR MESSAGE: " <<
                            qpOASES::MessageHandling::getErrorCodeMessage(return_value) << std::endl;
                          return -1;
                }

                /* Get and print solution of QP. */
                example.getPrimalSolution( Opt );
                q0 = q0 + Opt[0]/100;
                q1 = q1 + Opt[1]/100;
                q_eef = l1 +l2 +l3 + q0 + q1;
                error = p*(q_des-q_eef);
                limit = std::abs(error);

                printf( "\nOpt = [ %g, %g, %g ]; \n posit= %g, error= %g, q0= %g q1= %g \n a0 = %g a1 = %g \n",
                        Opt[0],Opt[1],Opt[2],q_eef,error,q0,q1,(Opt[0]-v0_old),(Opt[1]-v1_old));

                /* Depending on joing goals*/
                if (q0_goal == false && q1_goal == false){}
                else if (q0_goal == true && q1_goal == false){
                    lbA[5] = q0_des-q0;
                    ubA[5] = q0_des-q0;
                    if (std::abs(lbA[5])<precision){
                        ok = true;
                        printf ("\n q0_error = %g, prec = %g \n",lbA[5],precision);
                    }
                }
                else if (q0_goal == false && q1_goal == true) {
                    lbA[5] = q1_des-q1;
                    ubA[5] = q1_des-q1;
                    if (std::abs(lbA[5])<precision){
                        ok = true;
                        printf ("\n q1_error = %g \n ",lbA[5]);
                    }
                }
                else{
                    lbA[5] = q0_des-q0;
                    ubA[5] = q0_des-q0;
                    lbA[6] = q1_des-q1;
                    ubA[6] = q1_des-q1;
                    if (std::abs(lbA[5])<precision && std::abs(lbA[6])<precision){
                        ok = true;
                        printf ("\n q0_error = %g  \n",lbA[5]);
                        printf ("\n q1_error = %g  \n",lbA[6]);
                    }
                }

                vel_pos.position[0] = q_eef;
                vel_pos.velocity[0] = Opt[0];
                vel_pos.effort[0] = error;
                vel_pos.effort[1] = Opt[1];
                vel_pos.effort[2] = lbA[4];
                vel_pos.effort[3] = ubA[4];
                vel_pos.header.stamp += ros::Duration(0.01);

                joint_pub.publish(vel_pos);

            }
            first = false;
            ros::spinOnce();

        }
        while ( (limit>precision || ok==false) &&  i<2000 );

        return 0;
    }
}

/* PLOT: rqt_plot /joint_state/position[0] /joint_state/velocity[0] /joint_state/effort[0] /joint_state/effort[1] */
