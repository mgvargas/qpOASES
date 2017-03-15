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
import tf
import yaml
from iai_markers_tracking.msg import Object
from iai_markers_tracking.srv import ObjGrasping
from tf.msg import tfMessage


class SelectGoal:
    def __init__(self):
        rospy.init_node('goal_selector', anonymous=True)
        r = rospy.Rate(2)
        self.objects = rospy.Subscriber('/detected_objects', Object, self.callback_obj)
        self.listener = tf.TransformListener(False, rospy.Duration(1))
        self.grasping_poses = []
        self.object_list = []
        self.old_list =[]

        # Gripper frames:
        self.grip_left = '/left_gripper_tool_frame'
        self.grip_right = '/right_gripper_tool_frame'


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
                    self.goal_obj = obj
                    self.grasping_poses_service(self.goal_obj)
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

    def distance(self):
        tf_listener = tf.TransformListener()
        trans_l = []
        trans_r = []
        rot_l = []
        rot_r = []
        print self.grasping_poses
        for n, pose in enumerate(self.grasping_poses):
            try:

                #(trans_l[n],rot_l[n]) = tf_listener.lookupTransform(self.grip_left, '/'+pose, rospy.Time(0))
                #(trans_r[n], rot_r[n]) = tf_listener.lookupTransform(self.grip_right, '/'+pose, rospy.Time(0))

                # (trans_l[n], rot_l[n]) = tf_listener.lookupTransform('/right_gripper_tool_frame', '/bowl_gp5', rospy.Time(0))

                tf_listener.waitForTransform("/right_gripper_tool_frame", "/base_link", rospy.Time(0), rospy.Duration(1, 5e8))
                (trans_r, rot)=tf_listener.lookupTransform("/right_gripper_tool_frame", "/base_link", rospy.Time.now())

            except (tf.ConnectivityException, tf.ExtrapolationException, tf.LookupException, tf.Exception) as exc:
                print '/' + pose
                print 'No TF found\n',exc
                continue
        print 'left: ', trans_r
        print 'right ', trans_l


def main():
    goal = SelectGoal()

    while not rospy.is_shutdown():

        grasping_poses = goal.object_selector()
        goal.distance()

        '''tf_listener = tf.TransformListener()
        trans_r = []

        for n, pose in enumerate(grasping_poses):
            try:
                tf_listener.waitForTransform("/right_gripper_tool_frame", "/base_link", rospy.Time(0),
                                             rospy.Duration(1.0))
                (trans_r, rot) = tf_listener.lookupTransform("/right_gripper_tool_frame", "/base_link", rospy.Time(0))

            except (tf.ConnectivityException, tf.ExtrapolationException, tf.LookupException, tf.Exception):  # ):
                print '/' + pose
                print 'No tf \n'
                continue
        print 'left: ', trans_r'''

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

# right_gripper_base_link
# right_gripper_tool_frame
# left_gripper_base_link
# left_gripper_tool_frame