#include <sd700e_control.h>
#include <list>

//#include "armplaning_client.h"

using namespace std;

typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> TrajectoryServer;

std::list<robotState> targetPointList; // list of points to move to

//***************************************************************************
// Processing and JointTrajectoryAction
void executeTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &r_goal, TrajectoryServer *r_as)
{
	// robotState rs;

	// float lastDuration = 0.0;

	// int nrOfPoints = r_goal->trajectory.points.size(); // Number of points to add
	// for (int i = 0; i < nrOfPoints; i++)
	// {
	// 	rs.j[0] = r_goal->trajectory.points[i].positions[0]; // ros values come in rad, internally we work in degree
	// 	rs.j[1] = r_goal->trajectory.points[i].positions[1];
	// 	rs.j[2] = r_goal->trajectory.points[i].positions[2];
	// 	rs.j[3] = r_goal->trajectory.points[i].positions[3];
	// 	rs.j[4] = r_goal->trajectory.points[i].positions[4];
	// 	rs.j[5] = r_goal->trajectory.points[i].positions[5];

	// 	float dtmp = r_goal->trajectory.points[i].time_from_start.toSec();
	// 	rs.duration = dtmp - lastDuration; // time_from_start is adding up, these values are only for the single motion
	// 	lastDuration = dtmp;

	// 	targetPointList.push_back(rs); //push_back() 在list的末尾添加一个元素
	// }
	ROS_INFO("receive trajectory success");
	r_as->setSucceeded();

	//debug msg
	// ROS_INFO("arm recv: %f %f %f %f %f %f,duration: %f", rs.j[0], rs.j[1], rs.j[2], rs.j[3], rs.j[4], rs.j[5], rs.duration);
}
//*************************************************************************
void quit(int sig)
{
	ros::shutdown();
	exit(0);
}

//******************** MAIN ************************************************
int main(int argc, char **argv)
{
	ros::init(argc, argv, "sd700e_control");
	ros::NodeHandle n2;

	//Start the ActionServer for JointTrajectoryActions and GripperCommandActions from MoveIT
	TrajectoryServer r_tserver(n2, "sd700e_grip/follow_joint_trajectory", boost::bind(&executeTrajectory, _1, &r_tserver), false);
	ROS_INFO("TrajectoryActionServer: Starting");
	r_tserver.start();

	ROS_INFO("Starting Main Loop");
	for (;;)
	{
		ros::spinOnce();
		// ros::Duration(50 / 1000.0).sleep(); // main loop with 20 Hz.(50/1000=0.05s=50ms)
	}
	ROS_INFO("Closing Main Loop");

	signal(SIGINT, quit);
	return (0);
}
