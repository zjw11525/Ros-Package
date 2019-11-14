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
import moveit_commander
import os

class MoveItFkDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_fk_demo', anonymous=True)
 
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('manipulator')
        
        # 设置机械臂运动的允许误差值
        arm.set_goal_joint_tolerance(0.001)

        # 设置允许的最大速度和加速度
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)
        # 回零点
        # joint_positions =  [0,0,0,0,0,0]
        # arm.set_joint_value_target(joint_positions)
        # arm.go()
        # rospy.sleep(3)
        # while not rospy.is_shutdown():
        # joint_positions =  [0.0008115183246073298, -0.9789458987783596, 0.14212914485165795, 1.7452006980802795e-06, -0.7361413612565446, 1.5352879581151833]
        # arm.set_joint_value_target(joint_positions)
        # arm.go()
        # rospy.sleep(3)

        joint_positions =  [0.0009424083769633508, -1.0826963350785341, 0.18063874345549738, -2.9668411867364745e-05, -0.6719267015706806, 1.5365706806282724]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.sleep(2)

        os.system("python ~/MQTT/ros_gripper_control.py 1100")
        rospy.sleep(1)

        joint_positions =  [0.0008900523560209424, -0.8427521815008727, 0.13186561954624784, 5.235602094240838e-05, -0.8623944153577662, 1.5366335078534032]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.sleep(2)

        joint_positions =  [0.8961780104712043, -1.113692844677138, 0.4683089005235602, -0.005966841186736475, -0.9215898778359511, 2.44526352530541]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.sleep(3)

        joint_positions =  [0.8954258289703315, -1.3264729493891798, 0.5701012216404887, -0.0065043630017452, -0.8102757417102968, 2.457930191972077]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.sleep(2)

        os.system("python ~/MQTT/ros_gripper_control.py 0")
        rospy.sleep(1)

        joint_positions = [0.8961780104712043, -1.113692844677138, 0.4683089005235602, -0.005966841186736475, -0.9215898778359511, 2.44526352530541]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.sleep(2)

        joint_positions =  [0.0008115183246073298, -0.9789458987783596, 0.14212914485165795, 1.7452006980802795e-06, -0.7361413612565446, 1.5352879581151833]
        arm.set_joint_value_target(joint_positions)
        arm.go()
        rospy.sleep(3)
        
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItFkDemo()
    except rospy.ROSInterruptException:
        pass