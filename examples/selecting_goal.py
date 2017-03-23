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
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.

import rospy
import tf2_ros
import math
import rospkg
import sys
import numpy as np
import PyKDL as kdl
from urdf_parser_py import urdf
from iai_markers_tracking.msg import Object
from iai_markers_tracking.srv import ObjGrasping
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF
from kdl_parser import kdl_tree_from_urdf_model


class SelectGoal:
    def __init__(self):
        rospy.init_node('goal_selector', anonymous=True)
        r = rospy.Rate(2)
        self.objects = rospy.Subscriber('/detected_objects', Object, self.callback_obj)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.grasping_poses = []
        self.object_list = []
        self.old_list =[]
        self.joint_limits_lower = []
        self.joint_limits_upper = []
        self.joint_names = []

        # Arm selection
        self.left_arm = False
        self.right_arm = False
        # Gripper frames:
        self.grip_left = 'left_gripper_tool_frame'
        self.grip_right = 'right_gripper_tool_frame'

        # KDL chains:
        self.right_chain = kdl.Chain()
        self.left_chain = kdl.Chain()
        self.ik_lambda = 0.35  # how much should singularities be avoided?
        self.arms_chain()  # generate chains for both arms

        self.jnt_pos = kdl.JntArray(self.nJoints)
        self.jac_solver_left = kdl.ChainJntToJacSolver(self.left_chain)
        self.jac_solver_right = kdl.ChainJntToJacSolver(self.right_chain)

        self.joint_list = rospy.Subscriber('/joint_states', JointState, self.joint_callback)
        rospy.sleep(0.5)

    def callback_obj(self, data):
        obj = data.data
        if len(obj) > 0:
            self.object_list = obj

    def joint_callback(self, data):
        #Setting the current joint angles
        self.joint_names = data.name
        self.joint_values = data.position
        self.set_joints()

    def set_joints(self, joint_name='left_arm'):
        # getting current joint values
        items =[]
        a = 0
        for i,x in enumerate(self.joint_names):
            if joint_name in x:
                items.append(i)
                self.jnt_pos[a] = self.joint_values[i]
                a += 1
        return self.jnt_pos

    def get_urdf(self):
        rospack = rospkg.RosPack()
        dir = rospack.get_path('iai_markers_tracking') + '/urdf/boxy_description.urdf'
        try:
            self.urdf_model = urdf.Robot.from_xml_file(dir)
            #self.urdf_model = URDF.from_parameter_server()

        except (OSError, LookupError):
            rospy.logerr('URDF not found in parameter server')

    # TODO: Change to a smarter selector (from a ROS Topic maybe)
    def object_selector(self):
        # From the found objects, select one to grasp
        if self.old_list != self.object_list:
            for obj in self.object_list:
                if obj == 'bowl':
                    goal_obj = obj
                    self.grasping_poses_service(goal_obj)
        self.old_list = self.object_list
        return self.grasping_poses

    def grasping_poses_service(self, goal_obj):
        # Calling a service to obtain the name of the grasping poses of an object
        rospy.wait_for_service('object_grasping_poses')
        try:
            g_p = rospy.ServiceProxy('object_grasping_poses', ObjGrasping)
            grasping_poses_list = g_p(goal_obj)
            self.grasping_poses = grasping_poses_list.grasp_poses
            print self.grasping_poses
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def goal_by_distance(self):
        # Find the grasping pose that is closer to one of the grippers
        trans_l = [0]*len(self.grasping_poses)
        trans_r = [0]*len(self.grasping_poses)
        dist_l = [0]*len(self.grasping_poses)
        dist_r = [0] * len(self.grasping_poses)
        pose_found = False

        for n, pose in enumerate(self.grasping_poses):
            try:
                trans_l[n] = self.tfBuffer.lookup_transform(self.grip_left, pose, rospy.Time(0), rospy.Duration(1, 5e8))
                trans_r[n] = self.tfBuffer.lookup_transform(self.grip_right, pose, rospy.Time(0), rospy.Duration(1, 5e8))
                pose_found = True

            except (tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException, tf2_ros.LookupException) as exc:
                print '\nNo TF found\n',exc
                continue
            else:
                dist_l[n] = math.sqrt(
                    trans_l[n].transform.translation.x ** 2 + trans_l[n].transform.translation.y ** 2
                    + trans_l[n].transform.translation.z ** 2)
                dist_r[n] = math.sqrt(
                    trans_r[n].transform.translation.x ** 2 + trans_r[n].transform.translation.y ** 2
                    + trans_r[n].transform.translation.z ** 2)

        if pose_found:
            min_dist_l = min(d for d in dist_l)
            min_dist_r = min(d for d in dist_r)
            if min_dist_l < min_dist_r:
                elem = [i for i, d in enumerate(dist_l) if d == min_dist_l]
                closest_pose = trans_l[elem[0]]
                self.left_arm = True
                self.right_arm = False
            else:
                elem = [i for i, d in enumerate(dist_r) if d == min_dist_r]
                closest_pose = trans_r[elem[0]]
                self.right_arm = True
                self.left_arm = False
            return closest_pose

    def kinem_chain(self, joints, name_frame_base='triangle_base_link', name_frame_end='right_gripper_tool_frame'):
        # Transform URDF to Chain() for the joints that have th given string
        self.chain = kdl.Chain()

        try:
            self.joint_names = self.urdf_model.get_chain(name_frame_base, name_frame_end, links=False, fixed=False)
            self.name_frame_in = name_frame_base
            self.name_frame_out = name_frame_end
            self.njoints = len(self.joint_names)

            rospy.loginfo("Will control the following joints: %s" %(self.joint_names))

            self.kdl_tree = kdl_tree_from_urdf_model(self.urdf_model)
            self.chain = self.kdl_tree.getChain(name_frame_base, name_frame_end)
            self.kdl_fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
            self.kdl_ikv_solver = kdl.ChainIkSolverVel_wdls(self.chain)
            self.kdl_ikv_solver.setLambda(self.ik_lambda)
            # Default Task and Joint weights
            self.tweights = np.identity(6)
            # weight matrix with 1 in diagonal to make use of all the joints.
            self.jweights = np.identity(self.njoints)

            self.kdl_ikv_solver.setWeightTS(self.tweights.tolist())
            self.kdl_ikv_solver.setWeightJS(self.jweights.tolist())

            # Fill the list with the joint limits
            for jnt_name in self.joint_names:
                jnt = self.urdf_model.joint_map[jnt_name]
                if jnt.limit is not None:
                    self.joint_limits_lower.append(jnt.limit.lower)
                    self.joint_limits_upper.append(jnt.limit.upper)
            self.nJoints = self.chain.getNrOfJoints()

        except:
            print "Unexpected error:", sys.exc_info()[0]
            rospy.logerr('Could not re-init the kinematic chain')
            self.name_frame_out = ''

            '''for joint in self.urdf_model.joints:
            if joints not in joint.name:
                continue

            if joint.origin is not None:
                x = joint.origin.xyz[0]
                y = joint.origin.xyz[1]
                z = joint.origin.xyz[2]
                r = joint.origin.rpy[0]
                p = joint.origin.rpy[1]
                ry = joint.origin.rpy[2]
            else:
                x = y = z = r = p = ry = 0.0

            if joint.type == 'fixed':
                continue

            elif joint.type == 'prismatic':
                if joint.axis[0] == 1:
                    j_type = kdl.Joint.TransX
                elif joint.axis[1] == 1:
                    j_type = kdl.Joint.TransY
                else:
                    j_type = kdl.Joint.TransZ
            else:
                j_type = kdl.Joint.RotZ

            print joint.name

            self.chain.addSegment(
                kdl.Segment(joint.child, kdl.Joint(joint.name, j_type),
                            kdl.Frame(kdl.Rotation.RPY(r, p, ry), kdl.Vector(x, y, z))))
            # print '\nChain: {} \n'.format(self.chain.getSegment(-1))
            self.nJoints = self.chain.getNrOfJoints()'''
        return self.chain

    def arms_chain(self):
        self.get_urdf()
        self.right_chain = self.kinem_chain('right_arm')
        self.left_chain = self.kinem_chain('left_arm', name_frame_end='left_gripper_tool_frame')

    def get_jacobian(self, d_chain='left_chain'):
        # Obtain jacobian for the selected arm
        if d_chain == 'left_chain':
            jacobian = kdl.Jacobian(self.left_chain.getNrOfJoints())
            jnt_pos = self.set_joints(joint_name='left_arm')
            self.jac_solver_left.JntToJac(jnt_pos, jacobian)
            print '\n Left: \n', jacobian
            print jnt_pos

        elif d_chain == 'right_chain':
            jacobian = kdl.Jacobian(self.right_chain.getNrOfJoints())
            jnt_pos = self.set_joints(joint_name='right_arm')
            self.jac_solver_right.JntToJac(jnt_pos, jacobian)
            print '\n Right \n', jacobian
            print jnt_pos
        else:
            print 'Wrong chain specified for Jacobian'
            jacobian = kdl.Jacobian(self.chain.getNrOfJoints())

        return jacobian

    def manipulability(self, jacobian):
        col = jacobian.columns()
        manipulability = 1
        for n in range(col):
            column = jacobian.getColumn(n)
            for elem in column:
                #if elem != 0:
                    manipulability *= elem
        return manipulability

    def eef_pos(self):
        l = kdl.Frame()
        r = kdl.Frame()
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.right_chain)
        jnt_pos = self.set_joints(joint_name='right_arm')
        self.fk_solver.JntToCart(jnt_pos, r)
        self.fk_solver_l = kdl.ChainFkSolverPos_recursive(self.left_chain)
        jnt_pos = self.set_joints(joint_name='left_arm')
        self.fk_solver_l.JntToCart(jnt_pos, l)
        #print '\n EEF_l: \n', l
        #print '\n EEF_r: \n', r

def main():
    c_goal = SelectGoal()

    while not rospy.is_shutdown():

        # c_goal.object_selector()
        # closest_goal = c_goal.goal_by_distance()
        # print closest_goal
        jacobian_l = c_goal.get_jacobian(d_chain='left_chain')
        jacobian_r = c_goal.get_jacobian(d_chain='right_chain')
        manip_l = c_goal.manipulability(jacobian_l)
        manip_r = c_goal.manipulability(jacobian_r)

        c_goal.eef_pos()



        #print manip_l, manip_r

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
