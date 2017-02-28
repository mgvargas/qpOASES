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
    /* Weights. */
    float w1 = 1e-4;
    float w2 = 1e-4;
    float w3 = 1;
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
    float q0 = -0.05;
    float q1 = -0.15;
    /* Joint target. */
    float q_des = 0.6;
    /* Slack limits. */
    float e_min = -10;
    float e_max = 10;
    /* Velocity limits. */
    float v0_min = -0.01;
    float v0_max = 0.01;
    float v1_min = -0.1;
    float v1_max = 0.1;
    /* Acceleration limits. */
    float a0_min = -0.005;
    float a0_max = 0.005;
    float a1_min = -0.01;
    float a1_max = 0.01;
    /* Others */
    float precision= 1e-4;
    float p = 4;
    float q_eef = l1 +l2 +l3 + q0 + q1;
    float error = p*(q_des-q_eef);
    float vel_init = 0;

    int nWSR = 100;

    real_t H[3*3] = { w1, 0.0, 0.0,  0.0, w2, 0.0,  0.0, 0.0, w3 };
    real_t A[3*3] = { 1.0, 1.0, 1.0,  1.0, 0.0, 0.0,  0.0, 1.0, 0.0 };
    real_t g[3] = { 0.0, 0.0, 0.0 };
    real_t lb[3] = { v0_min, v1_min, e_min };
    real_t ub[3] = { v0_max, v1_max, e_max };
    real_t lbA[3] = { error, (q0_min - q0), (q1_min - q0) };
    real_t ubA[3] = { error, (q0_max - q0), (q1_max - q0) };

    /* Setting up QProblem object. */
    SQProblem example( 3,3 );
    Options options;
    options.setToReliable();
    options.printLevel = qpOASES::PL_HIGH;
    example.setOptions(options);


    printf( "Init pos = %g,  goal = %g, error = %g, q0 =%g, q1 = %g\n",
            q_eef,q_des,error,q0,q1);

    int i = 0;
    float limit = std::abs(error);;
    float condition;
    bool first = true;
    returnValue return_value;
    real_t Opt[3];

    /* ROS Message definition */
    sensor_msgs::JointState vel_pos;
    vel_pos.header.frame_id = "q0";
    vel_pos.position.push_back(q_eef);
    vel_pos.velocity.push_back(vel_init);
    vel_pos.effort.push_back(error);
    vel_pos.effort.push_back(0);
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
                ubA[0] = error;
                ubA[1] = q0_max - q0;
                ubA[2] = q1_max - q1;

                return_value = example.hotstart( H,g,A,lb,ub,lbA,ubA, nWSR );

                if(return_value != qpOASES::SUCCESSFUL_RETURN)
                {
                          std::cout << "Hotstart of QP-Problem returned without success! ERROR MESSAGE: " <<
                            qpOASES::MessageHandling::getErrorCodeMessage(return_value) << std::endl;
                          return -1;
                }

                /* Get and print solution of QP. */
                example.getPrimalSolution( Opt );
                q0 = q0 + Opt[0]/10;
                q1 = q1 + Opt[1]/10;
                q_eef = l1 +l2 +l3 + q0 + q1;
                error = p*(q_des-q_eef);
                limit = std::abs(error);

                condition = Opt[0]*A[0] + Opt[1]*A[1] + Opt[2]*A[2];

                printf( "\nOpt = [ %e, %e, %e ]; \n posit= %f, error= %f, q0= %f q1= %f \n",
                        Opt[0],Opt[1],Opt[2],q_eef,error,q0,q1);

                vel_pos.position[0] = q_eef;
                vel_pos.velocity[0] = Opt[0];
                vel_pos.effort[0] = error;
                vel_pos.effort[1] = Opt[1];
                vel_pos.header.stamp  += ros::Duration(0.1);

                joint_pub.publish(vel_pos);

            }
            first = false;
            ros::spinOnce();

        }
        while ( limit>precision &&  i<80 );

        return 0;
    }
}

/* PLOT: rqt_plot /joint_state/position[0] /joint_state/velocity[0]
NOTES:
- Matrix A constant
   - w1 LOW w2 LOW w3 LOW:
   - w1 LOW w2 LOW w3 1:
   - w1 LOW w2 LOW w3 10:
   - w1 LOW w2 1   w3 LOW:
   - w1 LOW w2 1   w3 1:
   - w1 LOW w2 1   w3 10:
   - w1 1   w2 LOW w3 LOW:
   - w1 1   w2 LOW w3 1:
   - w1 1   w2 LOW w3 10:
   - w1 1   w2 1   w3 LOW:
   - w1 1   w2 1   w3 1:
   - w1 1   w2 1   w3 10:



*/

