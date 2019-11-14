#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 该例程将请求/show_person服务，服务数据类型learning_service::Person

import sys
import rospy
from moveit_msgs.srv import GetPositionIK

def client():
	# ROS节点初始化
    rospy.init_node('person_client')

	# 发现/spawn服务后，创建一个服务客户端，连接名为/spawn的service
    rospy.wait_for_service('/compute_ik')
    try:
        ik_client = rospy.ServiceProxy('/compute_ik', GetPositionIK)
 
		# 请求服务调用，输入请求数据
        response = ik_client()
        return response.result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
	#服务调用并显示调用结果
    print "Show person result : %s" %(client())


