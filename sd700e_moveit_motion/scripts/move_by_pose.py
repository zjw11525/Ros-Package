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
from geometry_msgs.msg import PoseStamped, Pose 
import tf  
import os
from copy import deepcopy

class MoveItIkDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('moveit_ik_demo')
                
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('manipulator')
                
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()
                        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'base_link'
        arm.set_pose_reference_frame(reference_frame)
                
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.01)
       
        # 设置允许的最大速度和加速度
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)
        

        listener = tf.TransformListener() 

        while not rospy.is_shutdown():  
                try: 
                    # 获取源坐标系base_link与目标坐标系iron_block之间最新的一次坐标转换
                    (trans,rot) = listener.lookupTransform('/camera_color_optical_frame', '/iron_block', rospy.Time(0))

                    object_detect_x = trans[0]
                    object_detect_y = trans[1]
                    object_detect_z = trans[2]

                    if(object_detect_x>0.033 or object_detect_x<0.027):
                        # print("detect x error!")
                        continue

                    if(object_detect_y>0.040 or object_detect_y<0.020):
                        # print("detect y error!")
                        continue

                    # if(trans[2]>-0.04 or trans[2]<-0.08):
                    #     print("detect z error!")
                    #     continue

                    print(trans)
                   
                    # 获取当前位姿数据最为机械臂运动的起始位姿
                    start_pose = arm.get_current_pose(end_effector_link).pose
                    #print start_pose
                    wpose = deepcopy(start_pose)

                    wpose.position.y -= 0.2
                    wpose.position.z -= 0.158
                    arm.set_pose_target(wpose)
                    arm.go()

                    os.system("python ~/MQTT/ros_gripper_control.py 1100")
                    #rospy.sleep(1)

                    start_pose = arm.get_current_pose(end_effector_link).pose
                    #print start_pose
                    wpose = deepcopy(start_pose)

                    wpose.position.y -= 0.1
                    arm.set_pose_target(wpose)
                    arm.go()
                    


                    #回起始点
                    joint_positions = [-0.007717277486910995, -0.9330994764397906, 0.46196335078534034, 7.155322862129146e-05, -1.08595462478185, -0.007682373472949389]
                    arm.set_joint_value_target(joint_positions)
                    # 控制机械臂完成运动
                    arm.go()

                    # # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
                    # joint_positions = [-0.8188115183246074, -0.43414310645724263, -0.2707591623036649, -0.008221640488656196, -0.8719249563699826, -0.8136649214659686]
                    # arm.set_joint_value_target(joint_positions) 
                    # # 控制机械臂完成运动
                    # arm.go()

                    os.system("python ~/MQTT/ros_gripper_control.py 0")
                    rospy.sleep(1)

                    # # 获取源坐标系base_link与目标坐标系iron_block之间最新的一次坐标转换
                    # (trans,rot) = listener.lookupTransform('/base_link', '/iron_block', rospy.Time(0))

                    # object_detect_x = trans[0]
                    # object_detect_y = trans[1]


                    # if(object_detect_x>0.75 or object_detect_x<0.55):
                    #     print("detect x error!")
                    #     continue

                    # if(object_detect_y>0.32 or object_detect_y<-0.25):
                    #     print("detect y error!")
                    #     continue

                    # if(trans[2]>-0.04 or trans[2]<-0.08):
                    #     print("detect z error!")
                    #     continue

                    # object_detect_y = object_detect_y - 0.015
                    # print(trans)
                    # print(rot)
                    # #continue
                    # #设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
                    # #姿态使用四元数描述，基于base_link坐标系
                    # target_pose = PoseStamped()
                    # target_pose.header.frame_id = reference_frame
                    # target_pose.header.stamp = rospy.Time.now()     
                    # target_pose.pose.position.x = object_detect_x
                    # target_pose.pose.position.y = object_detect_y
                    # target_pose.pose.position.z = 0.15
                    # target_pose.pose.orientation.x = -rot[0]
                    # target_pose.pose.orientation.y = -rot[1]
                    # target_pose.pose.orientation.z = -rot[2]
                    # target_pose.pose.orientation.w = -rot[3]
                    
                    # # 设置机器臂当前的状态作为运动初始状态
                    # arm.set_start_state_to_current_state() 
                    # # 设置机械臂终端运动的目标位姿
                    # arm.set_pose_target(target_pose, end_effector_link) 
                    # # 规划运动路径
                    # traj = arm.plan()
                    # # 按照规划的运动路径控制机械臂运动
                    # arm.execute(traj)
                    # #rospy.sleep(1)

                    # target_pose = PoseStamped()
                    # target_pose.header.frame_id = reference_frame
                    # target_pose.header.stamp = rospy.Time.now()     
                    # target_pose.pose.position.x = object_detect_x
                    # target_pose.pose.position.y = object_detect_y
                    # target_pose.pose.position.z = 0.08
                    # target_pose.pose.orientation.x = -rot[0]
                    # target_pose.pose.orientation.y = -rot[1]
                    # target_pose.pose.orientation.z = -rot[2]
                    # target_pose.pose.orientation.w = -rot[3]
                    
                    # arm.set_start_state_to_current_state()
                    # arm.set_pose_target(target_pose, end_effector_link)
                    # traj = arm.plan()
                    # arm.execute(traj)

                    # os.system("python ~/MQTT/ros_gripper_control.py 1100")
                    # rospy.sleep(1)

                    # # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
                    # joint_positions = [-0.8188115183246074, -0.43414310645724263, -0.2707591623036649, -0.008221640488656196, -0.8719249563699826, -0.8136649214659686]
                    # arm.set_joint_value_target(joint_positions) 
                    # # 控制机械臂完成运动
                    # arm.go()

                    # os.system("python ~/MQTT/ros_gripper_control.py 0")
                    # rospy.sleep(1)

                    # #回起始点
                    # joint_positions = [-3.490401396160559e-05, -0.6098499127399651, 1.0162722513089006, 8.37696335078534e-05, -1.959607329842932, 2.268760907504363e-05]
                    # arm.set_joint_value_target(joint_positions)
                    # # 控制机械臂完成运动
                    # arm.go()
                    # rospy.sleep(2)

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):  
                    continue 
                    
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


if __name__ == "__main__":
    MoveItIkDemo()

    
    