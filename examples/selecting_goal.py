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
        r = rospy.Rate(20)
        self.objects = rospy.Subscriber('/found_objects', Object, self.callback_obj)
        self.listener = tf.TransformListener(False, rospy.Duration(1))
        self.object_list = []
        self.old_list =[]

    def callback_obj(self, data):
        obj = data.data
        if len(obj) > 0:
            self.object_list = obj

    def object_selector(self):
        # From the found objects, select one to grasp
        if self.old_list != self.object_list:
            print self.old_list
            for obj in self.object_list:
                if obj == 'bowl':
                    self.goal_obj = obj
                    self.grasping_poses_service(self.goal_obj)
        self.old_list = self.object_list

    def grasping_poses_service(self, goal_obj):
        # Calling a service to obtain the name of the grasping poses of an object
        rospy.wait_for_service('object_grasping_poses')
        try:
            g_p = rospy.ServiceProxy('object_grasping_poses', ObjGrasping)
            self.grasping_poses = g_p(goal_obj)
            print self.grasping_poses
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e



def main():
    goal = SelectGoal()

    while not rospy.is_shutdown():

        goal.object_selector()

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass