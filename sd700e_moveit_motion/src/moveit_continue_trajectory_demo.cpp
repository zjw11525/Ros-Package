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

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/OrientationConstraint.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_revise_trajectory_demo");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("manipulator");

    arm.setGoalJointTolerance(0.001);

    double accScale = 0.8;
    double velScale = 0.8;
    arm.setMaxAccelerationScalingFactor(accScale);
    arm.setMaxVelocityScalingFactor(velScale);

    // 获取机器人的起始位置
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
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    moveit::planning_interface::MoveItErrorCode success = arm.plan(plan1);

    ROS_INFO("Plan (pose1) %s", success ? "SUCCESS" : "FAILED");

    //设置第二个目标点
    // geometry_msgs::Pose target_pose1;
    // target_pose1.orientation.x = start_pose.pose.orientation.x;
    // target_pose1.orientation.y = start_pose.pose.orientation.y;
    // target_pose1.orientation.z = start_pose.pose.orientation.z;
    // target_pose1.orientation.w = start_pose.pose.orientation.w;

    // target_pose1.position.x = start_pose.pose.position.x;
    // target_pose1.position.y = start_pose.pose.position.y - 0.3;
    // target_pose1.position.z = start_pose.pose.position.z - 0.155;

    // arm.setPoseTarget(target_pose1);

    // // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
    // moveit::planning_interface::MoveGroupInterface::Plan plan2;
    // success = arm.plan(plan2);

    // //连接两条轨迹
    // moveit_msgs::RobotTrajectory trajectory;

    // trajectory.joint_trajectory.joint_names = plan1.trajectory_.joint_trajectory.joint_names;
    // trajectory.joint_trajectory.points = plan1.trajectory_.joint_trajectory.points;
    // for (size_t j = 1; j < plan2.trajectory_.joint_trajectory.points.size(); j++)
    // {
    //     trajectory.joint_trajectory.points.push_back(plan2.trajectory_.joint_trajectory.points[j]);
    // }

    // moveit::planning_interface::MoveGroupInterface::Plan joinedPlan;
    // robot_trajectory::RobotTrajectory rt(arm.getCurrentState()->getRobotModel(), "manipulator");
    // rt.setRobotTrajectoryMsg(*arm.getCurrentState(), trajectory);
    // trajectory_processing::IterativeParabolicTimeParameterization iptp;
    // iptp.computeTimeStamps(rt, velScale, accScale);

    // rt.getRobotTrajectoryMsg(trajectory);
    // joinedPlan.trajectory_ = trajectory;

    // if (!arm.execute(joinedPlan))
    // {
    //     ROS_ERROR("Failed to execute plan");
    //     return false;
    // }

    // sleep(1);

    ROS_INFO("Finished");

    // 控制机械臂先回到初始化位置
    // arm.setNamedTarget("home");
    // arm.move();
    sleep(1);

    ros::shutdown();

    return 0;
}
