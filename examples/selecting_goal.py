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
import yaml
from iai_markers_tracking.msg import Object
from iai_markers_tracking.srv import ObjGrasping
from tf.msg import tfMessage


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

        # Arm selection
        self.left_arm = False
        self.right_arm = False

        # Gripper frames:
        self.grip_left = 'left_gripper_tool_frame'
        self.grip_right = 'right_gripper_tool_frame'

    def callback_obj(self, data):
        obj = data.data
        if len(obj) > 0:
            self.object_list = obj

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


def main():
    c_goal = SelectGoal()

    while not rospy.is_shutdown():

        c_goal.object_selector()
        closest_goal = c_goal.goal_by_distance()
        print closest_goal

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
