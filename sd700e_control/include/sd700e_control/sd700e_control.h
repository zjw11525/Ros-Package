#ifndef sd700e_control_H
#define sd700e_control_H

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <fstream>

#include <list>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>

#include <actionlib/server/simple_action_server.h> //header is the standard action library to implement an action server node

#include <control_msgs/FollowJointTrajectoryAction.h> ////The header is generated from the stored action files

struct robotState
{
	float j[6];		// joint position
	float duration; // duration for motion; needed for actionServer
};

#endif