#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2019 Wuhan PS-Micro Technology Co., Itd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import rospy, sys
import tf

def move_object_sim():
    # 初始化ROS节点
    count = 0.3
    rospy.init_node('move_object_sim')
    while not rospy.is_shutdown():
        point_tf = tf.TransformBroadcaster()
        point_tf.sendTransform((0.52, count, -0.05), #the translation of the transformtion as a tuple (x, y, z)
                        tf.transformations.quaternion_from_euler(0, 1.57, -1.57), 
                                                    #the rotation of the transformation as a tuple (x, y, z, w)
                        rospy.Time.now(), #the time of the transformation, as a rospy.Time()
                        "/iron_block", #child frame in tf, string
                        "/base_link") #parent frame in tf, string
        count = count - 0.005
        if (count <= -0.3):
            count = 0.3

        rospy.sleep(0.1)

if __name__ == "__main__":   
    move_object_sim()
