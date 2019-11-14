/***********************************************************************
Copyright 2019 Wuhan PS-Micro Technology Co., Itd.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
***********************************************************************/

#include <string>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "moveit_fk_demo");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface arm("manipulator");

  //获取终端link的名称
  std::string end_effector_link = arm.getEndEffectorLink();

  //设置目标位置所使用的参考坐标系
  std::string reference_frame = "base_link";
  arm.setPoseReferenceFrame(reference_frame);

  //当运动规划失败后，允许重新规划
  arm.allowReplanning(true);

  //设置位置(单位：米)和姿态（单位：弧度）的允许误差
  arm.setGoalPositionTolerance(0.001);
  arm.setGoalOrientationTolerance(0.01);

  //设置允许的最大速度和加速度
  arm.setMaxAccelerationScalingFactor(0.5);
  arm.setMaxVelocityScalingFactor(0.5);

  // 控制机械臂先回到初始化位置
  // arm.setNamedTarget("Home");
  // arm.move();
  // sleep(1);

  geometry_msgs::PoseStamped start_pose = arm.getCurrentPose("Link6");
  ROS_INFO("Link6 Position:%f,%f,%f", start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z);
  // 设置机器人终端的目标位置
  geometry_msgs::Pose target_pose;
  target_pose.orientation.x = start_pose.pose.orientation.x;
  target_pose.orientation.y = start_pose.pose.orientation.y;
  target_pose.orientation.z = start_pose.pose.orientation.z;
  target_pose.orientation.w = start_pose.pose.orientation.w;

  target_pose.position.x = start_pose.pose.position.x;
  target_pose.position.y = start_pose.pose.position.y - 0.2;
  target_pose.position.z = start_pose.pose.position.z - 0.155;

  // 设置机器臂当前的状态作为运动初始状态
  arm.setStartStateToCurrentState();

  arm.setPoseTarget(target_pose);

  // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  moveit::planning_interface::MoveItErrorCode success = arm.plan(plan);

  ROS_INFO("Plan (pose goal) %s", success ? "" : "FAILED");

  //让机械臂按照规划的轨迹开始运动。
  if (success)
    arm.execute(plan);
  sleep(1);

  double targetPose[6] = {-0.007717277486910995, -0.9330994764397906, 0.46196335078534034, 7.155322862129146e-05, -1.08595462478185, -0.007682373472949389};
  std::vector<double> joint_group_positions(6);
  joint_group_positions[0] = targetPose[0];
  joint_group_positions[1] = targetPose[1];
  joint_group_positions[2] = targetPose[2];
  joint_group_positions[3] = targetPose[3];
  joint_group_positions[4] = targetPose[4];
  joint_group_positions[5] = targetPose[5];

  arm.setJointValueTarget(joint_group_positions);
  arm.move();
  sleep(1);
  // // 控制机械臂先回到初始化位置
  // arm.setNamedTarget("home");
  // arm.move();
  // sleep(1);

  ros::shutdown();

  return 0;
}
