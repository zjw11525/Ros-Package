#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

void scale_trajectory_speed(moveit::planning_interface::MoveGroupInterface::Plan &plan, double scale)
{
    int n_joints = plan.trajectory_.joint_trajectory.joint_names.size();

    for (int i = 0; i < plan.trajectory_.joint_trajectory.points.size(); i++)
    {
        plan.trajectory_.joint_trajectory.points[i].time_from_start *= 1 / scale;

        for (int j = 0; j < n_joints; j++)
        {
            plan.trajectory_.joint_trajectory.points[i].velocities[j] *= scale;
            plan.trajectory_.joint_trajectory.points[i].accelerations[j] *= scale * scale;
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "line_move");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    // 创建节点句柄
    ros::NodeHandle node;

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
    arm.setMaxAccelerationScalingFactor(0.2);
    arm.setMaxVelocityScalingFactor(0.2);

    // 控制机械臂先回到初始化位置
    // arm.setNamedTarget("Home");
    // arm.move();
    // sleep(1);

    // 创建tf的监听器
    tf::TransformListener listener;
    ros::Rate rate(100.0);
    while (node.ok())
    {
        // 获取base_link与iron_block坐标系之间的tf数据
        tf::StampedTransform transform;
        try
        {
            listener.waitForTransform("/base_link", "/iron_block", ros::Time(0), ros::Duration(3.0));
            listener.lookupTransform("/base_link", "/iron_block", ros::Time(0), transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        tf::Vector3 t = transform.getOrigin();

        if (t.getY() > 0.25 or t.getY() < 0.15)
        {
            ROS_INFO("detect y error!");
            ros::Duration(1.0).sleep();
            continue;
        }
        std::cout << "transform : " << t.getY() << std::endl;

        // 获取当前位姿数据最为机械臂运动的起始位姿
        geometry_msgs::Pose start_pose = arm.getCurrentPose(end_effector_link).pose;
        // 设置机器人终端的目标位置
        geometry_msgs::Pose target_pose;
        // target_pose.orientation.x = -t.x();
        // target_pose.orientation.y = -t.y();
        // target_pose.orientation.z = -t.z();
        // target_pose.orientation.w = -t.w();
        target_pose.orientation.x = start_pose.orientation.x;
        target_pose.orientation.y = start_pose.orientation.y;
        target_pose.orientation.z = start_pose.orientation.z;
        target_pose.orientation.w = start_pose.orientation.w;

        target_pose.position.x = t.getX();
        target_pose.position.y = t.getY() - 0.05 - 0.03;
        target_pose.position.z = 0.13;

        // 设置机器臂当前的状态作为运动初始状态
        arm.setStartStateToCurrentState();

        arm.setPoseTarget(target_pose);

        // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        moveit::planning_interface::MoveItErrorCode success = arm.plan(plan);

        ROS_INFO("Plan (pose goal) %s", success ? "" : "FAILED");

        //让机械臂按照规划的轨迹开始运动。
        if (success)
        {
            int num_of_points = plan.trajectory_.joint_trajectory.points.size();
            ros::Duration d = plan.trajectory_.joint_trajectory.points[num_of_points - 1].time_from_start;
            double h_time = double(d.sec) + double(d.nsec) * 1e-9;
            double scale = h_time / 1.0;
            std::cout << scale << std::endl;
            scale_trajectory_speed(plan, scale);
            // d = plan.trajectory_.joint_trajectory.points[num_of_points - 1].time_from_start;
            // h_time = double(d.sec) + double(d.nsec) * 1e-9;
            // std::cout << h_time << std::endl;
            arm.execute(plan);
        }

        start_pose = arm.getCurrentPose(end_effector_link).pose;
        std::vector<geometry_msgs::Pose> waypoints;

        //将初始位姿加入路点列表
        waypoints.push_back(start_pose);

        start_pose.position.y -= 0.3;
        start_pose.position.z -= 0.03;
        waypoints.push_back(start_pose);

        // 笛卡尔空间下的路径规划
        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.001;
        double fraction = 0.0;
        int maxtries = 10; //最大尝试规划次数
        int attempts = 0;  //已经尝试规划次数

        while (fraction < 1.0 && attempts < maxtries)
        {
            fraction = arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
            attempts++;

            if (attempts % 10 == 0)
                ROS_INFO("Still trying after %d attempts...", attempts);
        }

        if (fraction == 1)
        {
            ROS_INFO("Path computed successfully. Moving the arm.");

            // 生成机械臂的运动规划数据
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;

            int num_of_points = plan.trajectory_.joint_trajectory.points.size();

            ros::Duration d = plan.trajectory_.joint_trajectory.points[num_of_points - 1].time_from_start;

            double h_time = double(d.sec) + double(d.nsec) * 1e-9;

            double scale = h_time / (6.0 + 0.2);

            scale_trajectory_speed(plan, scale);
            // 执行运动
            arm.execute(plan);
            sleep(2);
        }
        else
        {
            ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
        }

        // 控制机械臂先回到初始化位置
        // arm.setNamedTarget("Home");
        // arm.move();

        rate.sleep();
    }

    // sleep(1);

    ros::shutdown();
    return 0;
}