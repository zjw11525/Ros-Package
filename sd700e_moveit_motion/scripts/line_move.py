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
from moveit_commander import MoveGroupCommander
from geometry_msgs.msg import PoseStamped, Pose
from copy import deepcopy
import tf  
import os
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class MoveItCartesianDemo:
    def scale_trajectory_speed(self, traj, scale):
       # Create a new trajectory object
       new_traj = RobotTrajectory()
       
       # Initialize the new trajectory to be the same as the input trajectory
       new_traj.joint_trajectory = traj.joint_trajectory
       
       # Get the number of joints involved
       n_joints = len(traj.joint_trajectory.joint_names)
       
       # Get the number of points on the trajectory
       n_points = len(traj.joint_trajectory.points)
        
       # Store the trajectory points
       points = list(traj.joint_trajectory.points)
       
       # Cycle through all points and joints and scale the time from start,
       # as well as joint speed and acceleration
       for i in range(n_points):
           point = JointTrajectoryPoint()
           
           # The joint positions are not scaled so pull them out first
           point.positions = traj.joint_trajectory.points[i].positions

           # Next, scale the time_from_start for this point
           point.time_from_start = traj.joint_trajectory.points[i].time_from_start / scale
           
           # Get the joint velocities for this point
           point.velocities = list(traj.joint_trajectory.points[i].velocities)
           
           # Get the joint accelerations for this point
           point.accelerations = list(traj.joint_trajectory.points[i].accelerations)
           
           # Scale the velocity and acceleration for each joint at this point
           for j in range(n_joints):
               point.velocities[j] = point.velocities[j] * scale
               point.accelerations[j] = point.accelerations[j] * scale * scale
        
           # Store the scaled trajectory point
           points[i] = point

       # Assign the modified points to the new trajectory
       new_traj.joint_trajectory.points = points

       # Return the new trajecotry
       return new_traj

    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_cartesian_demo', anonymous=True)
        
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = MoveGroupCommander('manipulator')
        
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'base_link'
        arm.set_pose_reference_frame(reference_frame)

        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.001)
        
        # 设置允许的最大速度和加速度
        arm.set_max_acceleration_scaling_factor(1)
        arm.set_max_velocity_scaling_factor(1)
        
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()
        
        # # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
        # joint_positions = [-1.1825355285207154e-05, -0.5670001723145425, 0.9280267992893177, 5.3613237943855825e-06, -1.9227286805556647, -6.314999961870072e-05[-1.1825355285207154e-05, -0.5670001723145425, 0.9280267992893177, 5.3613237943855825e-06, -1.9227286805556647, -6.314999961870072e-05]
        # arm.set_joint_value_target(joint_positions)
        # # 控制机械臂完成运动
        # arm.go()

        listener = tf.TransformListener() 

        while not rospy.is_shutdown():
            try:
                # 获取源坐标系base_link与目标坐标系iron_block之间最新的一次坐标转换
                (trans,rot) = listener.lookupTransform('/base_link', '/iron_block', rospy.Time(0))

                object_detect_x = trans[0]
                object_detect_y = trans[1]
                # object_detect_z = trans[2]

                # if(object_detect_x>0.033 or object_detect_x<0.027):
                #     # print("detect x error!")
                #     continue

                if(object_detect_y>0.25 or object_detect_y<0.2):
                    # print("detect y error!")
                    continue

                # if(trans[2]>-0.04 or trans[2]<-0.08):
                #     print("detect z error!")
                #     continue

                print(trans)
                # print(rot)
                # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
                # 姿态使用四元数描述，基于base_link坐标系
                target_pose = PoseStamped()
                target_pose.header.frame_id = reference_frame
                target_pose.header.stamp = rospy.Time.now()
                target_pose.pose.position.x = object_detect_x
                target_pose.pose.position.y = object_detect_y-0.05
                target_pose.pose.position.z = 0.15
                target_pose.pose.orientation.x = -rot[0]
                target_pose.pose.orientation.y = -rot[1]
                target_pose.pose.orientation.z = -rot[2]
                target_pose.pose.orientation.w = -rot[3]
                
                # 设置机器臂当前的状态作为运动初始状态
                arm.set_start_state_to_current_state()
                # 设置机械臂终端运动的目标位姿
                arm.set_pose_target(target_pose, end_effector_link) 
                # 规划运动路径
                traj = arm.plan()
                num_of_points = len(traj.joint_trajectory.points)
                d = rospy.Duration.from_sec(1)
                print traj.joint_trajectory.points[num_of_points-1].time_from_start
                for index in range(num_of_points):
                    traj.joint_trajectory.points[index].time_from_start *= \
                    d/traj.joint_trajectory.points[num_of_points-1].time_from_start
                # 按照规划的运动路径控制机械臂运动
                arm.execute(traj)
                # # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
                # # 姿态使用四元数描述，基于base_link坐标系
                # target_pose = PoseStamped()
                # target_pose.header.frame_id = reference_frame
                # target_pose.header.stamp = rospy.Time.now()
                # target_pose.pose.position.x = object_detect_x
                # target_pose.pose.position.y = object_detect_y-0.1
                # target_pose.pose.position.z = 0.10
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
                # num_of_points = len(traj.joint_trajectory.points)
                # d = rospy.Duration.from_sec(0.2)
                # print traj.joint_trajectory.points[num_of_points-1].time_from_start
                # for index in range(num_of_points):
                #     traj.joint_trajectory.points[index].time_from_start *= \
                #     d/traj.joint_trajectory.points[num_of_points-1].time_from_start
                # # 按照规划的运动路径控制机械臂运动
                # arm.execute(traj)

                # 获取当前位姿数据最为机械臂运动的起始位姿
                start_pose = arm.get_current_pose(end_effector_link).pose
                
                # 初始化路点列表
                waypoints = []

                # 将初始位姿加入路点列表
                waypoints.append(start_pose)
                wpose = deepcopy(start_pose)
                
                wpose.position.y -= 0.3
                waypoints.append(deepcopy(wpose))

                fraction = 0.0   #路径规划覆盖率
                maxtries = 10   #最大尝试规划次数
                attempts = 0     #已经尝试规划次数
                
                # 设置机器臂当前的状态作为运动初始状态
                arm.set_start_state_to_current_state()

                # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
                while fraction < 1.0 and attempts < maxtries:
                    (plan, fraction) = arm.compute_cartesian_path (
                                            waypoints,   # waypoint poses，路点列表
                                            0.1,        # eef_step，终端步进值
                                            0.0,         # jump_threshold，跳跃阈值
                                            True)        # avoid_collisions，避障规划
                    
                    # 尝试次数累加
                    attempts += 1
                    
                    # 打印运动规划进程
                    if attempts % 10 == 0:
                        rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                # print plan
                # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
                if fraction == 1.0:
                    rospy.loginfo("Path computed successfully. Moving the arm.")

                    # num_of_points = len(plan.joint_trajectory.points)
                    # d = rospy.Duration.from_sec(6)
                    # scale_factor = d/plan.joint_trajectory.points[num_of_points-1].time_from_start
                    # print scale_factor
                    # for index in range(num_of_points):
                    #     plan.joint_trajectory.points[index].time_from_start *= scale_factor
                    # print plan

                    # Scale the trajectory speed by a factor of 0.25
                    new_traj = self.scale_trajectory_speed(plan, 0.25)
                    arm.execute(new_traj)

                    rospy.loginfo("Path execution complete.")
                # 如果路径规划失败，则打印失败信息
                else:
                    rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")    

                # #rospy.sleep(1)


                # # os.system("python ~/MQTT/ros_gripper_control.py 1100")
                # # rospy.sleep(1)

                # 回起始点
                # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
                # rospy.loginfo("go origin position")
                # joint_positions = [-1.1825355285207154e-05, -0.5670001723145425, 0.9280267992893177, 5.3613237943855825e-06, -1.9227286805556647, -6.314999961870072e-05[-1.1825355285207154e-05, -0.5670001723145425, 0.9280267992893177, 5.3613237943855825e-06, -1.9227286805556647, -6.314999961870072e-05]
                # arm.set_joint_value_target(joint_positions)
                # # 控制机械臂完成运动
                # arm.go()


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
                rospy.sleep(2)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):  
                continue 


      



        # listener = tf.TransformListener() 

        # while not rospy.is_shutdown():  
        #         try:  
        #             # 获取源坐标系base_link与目标坐标系iron_block之间最新的一次坐标转换
        #             (trans,rot) = listener.lookupTransform('/camera_color_optical_frame', '/iron_block', rospy.Time(0))

        #             object_detect_x = trans[0]
        #             object_detect_y = trans[1]


        #             if(object_detect_x>0.033 or object_detect_x<0.027):
        #                 # print("detect x error!")
        #                 continue

        #             if(object_detect_y>0.040 or object_detect_y<0.020):
        #                 # print("detect y error!")
        #                 continue

        #             # if(trans[2]>-0.04 or trans[2]<-0.08):
        #             #     print("detect z error!")
        #             #     continue

        #             print(trans)
        #             # print(rot)
                   
        #             # 获取当前位姿数据最为机械臂运动的起始位姿
        #             start_pose = arm.get_current_pose(end_effector_link).pose
                    
        #             # 初始化路点列表
        #             waypoints = []

        #             # 将初始位姿加入路点列表
        #             waypoints.append(start_pose)
        #             wpose = deepcopy(start_pose)

        #             wpose.position.y -= 0.2
        #             wpose.position.z -= 0.155
        #             waypoints.append(deepcopy(wpose))

        #             wpose.position.y -= 0.1
        #             waypoints.append(deepcopy(wpose))

        #             fraction = 0.0   #路径规划覆盖率
        #             maxtries = 100   #最大尝试规划次数
        #             attempts = 0     #已经尝试规划次数
                    
        #             # 设置机器臂当前的状态作为运动初始状态
        #             arm.set_start_state_to_current_state()

        #             # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
        #             while fraction < 1.0 and attempts < maxtries:
        #                 (plan, fraction) = arm.compute_cartesian_path (
        #                                         waypoints,   # waypoint poses，路点列表
        #                                         0.001,        # eef_step，终端步进值
        #                                         0.0,         # jump_threshold，跳跃阈值
        #                                         True)        # avoid_collisions，避障规划
                        
        #                 # 尝试次数累加
        #                 attempts += 1
                        
        #                 # 打印运动规划进程
        #                 if attempts % 10 == 0:
        #                     rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
        #             #print plan
        #             # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
        #             if fraction == 1.0:
        #                 rospy.loginfo("Path computed successfully. Moving the arm.")
        #                 num_of_points = len(plan.joint_trajectory.points)
        #                 d = rospy.Duration.from_sec(3)
        #                 print plan.joint_trajectory.points[num_of_points-1].time_from_start
        #                 for index in range(num_of_points):
        #                     plan.joint_trajectory.points[index].time_from_start *= \
        #                     d/plan.joint_trajectory.points[num_of_points-1].time_from_start
        #                 #print plan
        #                 arm.execute(plan)

        #                 rospy.loginfo("Path execution complete.")
        #             # 如果路径规划失败，则打印失败信息
        #             else:
        #                 rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  

        #             #rospy.sleep(1)


        #             # os.system("python ~/MQTT/ros_gripper_control.py 1100")
        #             # rospy.sleep(1)

        #             # 回起始点
        #             rospy.loginfo("go origin position")
        #             joint_positions = [-0.007717277486910995, -0.9330994764397906, 0.46196335078534034, 7.155322862129146e-05, -1.08595462478185, -0.007682373472949389]
        #             arm.set_joint_value_target(joint_positions)
        #             # 控制机械臂完成运动
        #             arm.go()

        #             # # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
        #             # joint_positions = [-0.8188115183246074, -0.43414310645724263, -0.2707591623036649, -0.008221640488656196, -0.8719249563699826, -0.8136649214659686]
        #             # arm.set_joint_value_target(joint_positions) 
        #             # # 控制机械臂完成运动
        #             # arm.go()

        #             # os.system("python ~/MQTT/ros_gripper_control.py 0")
        #             # rospy.sleep(1)

        #             # #回起始点
        #             # joint_positions = [-3.490401396160559e-05, -0.6098499127399651, 1.0162722513089006, 8.37696335078534e-05, -1.959607329842932, 2.268760907504363e-05]
        #             # arm.set_joint_value_target(joint_positions)
        #             # # 控制机械臂完成运动
        #             # arm.go()
        #             rospy.sleep(2)

        #         except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):  
        #             continue 
        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    try:
        MoveItCartesianDemo()
    except rospy.ROSInterruptException:
        pass