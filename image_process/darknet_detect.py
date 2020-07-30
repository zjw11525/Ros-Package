#!/usr/bin/env python
#--coding: utf-8--
from __future__ import print_function
from ctypes import *
import math
import random
import roslib
roslib.load_manifest('image_process')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import tf

def sample(probs):
	s = sum(probs)
	probs = [a/s for a in probs]
	r = random.uniform(0, 1)
	for i in range(len(probs)):
		r = r - probs[i]
		if r <= 0:
			return i
	return len(probs)-1

def c_array(ctype, values):
	arr = (ctype*len(values))()
	arr[:] = values
	return arr

class BOX(Structure):
	_fields_ = [("x", c_float),
				("y", c_float),
				("w", c_float),
				("h", c_float)]

class DETECTION(Structure):
	_fields_ = [("bbox", BOX),
				("classes", c_int),
				("prob", POINTER(c_float)),
				("mask", POINTER(c_float)),
				("objectness", c_float),
				("sort_class", c_int)]


class IMAGE(Structure):
	_fields_ = [("w", c_int),
				("h", c_int),
				("c", c_int),
				("data", POINTER(c_float))]

class METADATA(Structure):
	_fields_ = [("classes", c_int),
				("names", POINTER(c_char_p))]

lib = CDLL("/home/zjw/darknet/libdarknet.so", RTLD_GLOBAL)
#lib = CDLL("libdarknet.so", RTLD_GLOBAL)
lib.network_width.argtypes = [c_void_p]
lib.network_width.restype = c_int
lib.network_height.argtypes = [c_void_p]
lib.network_height.restype = c_int

predict = lib.network_predict
predict.argtypes = [c_void_p, POINTER(c_float)]
predict.restype = POINTER(c_float)

set_gpu = lib.cuda_set_device
set_gpu.argtypes = [c_int]

make_image = lib.make_image
make_image.argtypes = [c_int, c_int, c_int]
make_image.restype = IMAGE

get_network_boxes = lib.get_network_boxes
get_network_boxes.argtypes = [c_void_p, c_int, c_int, c_float, c_float, POINTER(c_int), c_int, POINTER(c_int)]
get_network_boxes.restype = POINTER(DETECTION)

make_network_boxes = lib.make_network_boxes
make_network_boxes.argtypes = [c_void_p]
make_network_boxes.restype = POINTER(DETECTION)

free_detections = lib.free_detections
free_detections.argtypes = [POINTER(DETECTION), c_int]

free_ptrs = lib.free_ptrs
free_ptrs.argtypes = [POINTER(c_void_p), c_int]

network_predict = lib.network_predict
network_predict.argtypes = [c_void_p, POINTER(c_float)]

reset_rnn = lib.reset_rnn
reset_rnn.argtypes = [c_void_p]

load_net = lib.load_network
load_net.argtypes = [c_char_p, c_char_p, c_int]
load_net.restype = c_void_p

do_nms_obj = lib.do_nms_obj
do_nms_obj.argtypes = [POINTER(DETECTION), c_int, c_int, c_float]

do_nms_sort = lib.do_nms_sort
do_nms_sort.argtypes = [POINTER(DETECTION), c_int, c_int, c_float]

free_image = lib.free_image
free_image.argtypes = [IMAGE]

letterbox_image = lib.letterbox_image
letterbox_image.argtypes = [IMAGE, c_int, c_int]
letterbox_image.restype = IMAGE

load_meta = lib.get_metadata
lib.get_metadata.argtypes = [c_char_p]
lib.get_metadata.restype = METADATA

load_image = lib.load_image_color
load_image.argtypes = [c_char_p, c_int, c_int]
load_image.restype = IMAGE

rgbgr_image = lib.rgbgr_image
rgbgr_image.argtypes = [IMAGE]

predict_image = lib.network_predict_image
predict_image.argtypes = [c_void_p, IMAGE]
predict_image.restype = POINTER(c_float)

class image_converter:

	def __init__(self,net,meta):
		self.image_pub = rospy.Publisher("/object_detect",Image,queue_size=10)
		self.point_pub = rospy.Publisher("/point_find",PointStamped,queue_size=10)
		self.net = net
		self.meta = meta
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback)
		self.image_depth_sub = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw",Image,self.depth_callback)
		self.camera_info_sub = rospy.Subscriber("/camera/aligned_depth_to_color/camera_info",CameraInfo,self.info_callback)
		self.depth_image = Image()
		self.camera_info = CameraInfo()
		self.point = PointStamped()

	def depth_callback(self,data):
		try:
			self.depth_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
		except CvBridgeError as e:
			print(e)

	def info_callback(self,data):
		self.camera_info = data

	def location(self,image,x,y,angle):
		#先查询对齐的深度图像的深度信息，根据读取的camera info内参矩阵求解对应三维坐标
		real_z = 0.001 * self.depth_image[y][x]
		real_x = (x - self.camera_info.K[2]) / self.camera_info.K[0] * real_z
		real_y = (y - self.camera_info.K[5]) / self.camera_info.K[4] * real_z
		point_tf = tf.TransformBroadcaster()
		point_tf.sendTransform((real_x, real_y, real_z), #the translation of the transformtion as a tuple (x, y, z)
						tf.transformations.quaternion_from_euler(-1.57, 0, 0 + angle/57.3), 
													#the rotation of the transformation as a tuple (x, y, z, w)
						rospy.Time.now(), #the time of the transformation, as a rospy.Time()
						"/iron_block", #child frame in tf, string
						"/camera_color_optical_frame") #parent frame in tf, string
		self.point.header.frame_id	= "/camera_depth_optical_frame"
		self.point.header.stamp = rospy.Time.now()
		self.point.point.x = real_x
		self.point.point.y = real_y
		self.point.point.z = real_z
		# #校正rviz显示
		# correct_x = real_x - 0.015
		# self.point.point.x = correct_x
		try:
			self.point_pub.publish(self.point)
		except CvBridgeError as e:
			print(e)
		return real_x,real_y,real_z

	def callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		#(rows,cols,channels) = cv_image.shape
		cv2.imwrite("origin_image.jpg",cv_image)
		r = detect(self.net, self.meta, "origin_image.jpg")
		
		if(len(r)>0):
			for index in range(len(r)):
				x = int(r[index][2][0])
				y = int(r[index][2][1])
				x_w = int(r[index][2][2])*2
				y_w = int(r[index][2][3])
				crop_img = cv_image[y-y_w:y+y_w, x-x_w:x+x_w]
				cv_image = cv2.rectangle(cv_image, (x - 1, y - 1),\
									(x + 1, y + 1), (0, 0, 255), 2)
				# cv_image = cv2.rectangle(cv_image, (x - x_w, y - y_w),\
				#                     (x + x_w, y + y_w), (255, 0, 0), 2)
			
				angle = angle_detect(crop_img)
				X,Y,Z = self.location(cv_image,x,y,angle)
				word = '('+str(round(X,2)) + ',' + str(round(Y,2)) + ',' + str(round(Z,2)) + "," + str(int(angle)) + ')'
				#各参数依次是：图片，添加的文字，左上角坐标，字体，字体大小，颜色，字体粗细
				cv2.putText(cv_image, word, (x-150,y-20), cv2.FONT_HERSHEY_COMPLEX_SMALL,1, (0, 255, 0) ,1)
		
				#cv2.imshow("image", angle_image)
				#print(angle)
		#print(r)

		#cv2.imshow("Image window", cv_image)
		#cv2.waitKey(1)
		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
		except CvBridgeError as e:
			print(e)

def angle_detect(image):
	gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
	# cv2.imshow("Image window", gray)
	ret, binary = cv2.threshold(gray,100,255,cv2.THRESH_BINARY)  
	binary = cv2.bitwise_not(binary)
	#cv2.imshow("Binary window", binary)

	# 检测外轮廓
	binary, contours, hierarchy = cv2.findContours(binary,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
	# cv2.drawContours(image,contours,-1,(0,0,255),2)
	# print(len(contours))
	angle = 0
	x = 0
	y = 0
	for index in range(len(contours)):
		# 找到轮廓的最小外接矩形
		rect = cv2.minAreaRect(contours[index])
		# 找到4个顶点
		box = cv2.boxPoints(rect)
		# x = int((box[2][0] + box[0][0])/2)
		# y = int((box[2][1] + box[0][1])/2)
		# 找到最小外接矩形的两条边长
		line1 = math.sqrt((box[1][1] - box[0][1])*(box[1][1] - box[0][1]) + (box[1][0] - box[0][0])*(box[1][0] - box[0][0]))
		line2 = math.sqrt((box[3][1] - box[0][1])*(box[3][1] - box[0][1]) + (box[3][0] - box[0][0])*(box[3][0] - box[0][0]))
		# print line1 * line2
		# 面积太小的直接pass
		if line1 * line2 < 400:
			continue

		box = np.int0(box)
		cv2.drawContours(image,[box],0,(0,0,255),2)
		angle = rect[2]

		# image = cv2.rectangle(image, (x - 1, y - 1),\
		# 	(x + 1, y + 1), (0, 0, 255), 2)

		if line2 > line1:
			angle = 90 + angle  
		# print angle

	return angle

def detect(net, meta, image, thresh=.5, hier_thresh=.5, nms=.45):
	im = load_image(image, 0, 0)
	num = c_int(0)
	pnum = pointer(num)
	predict_image(net, im)
	dets = get_network_boxes(net, im.w, im.h, thresh, hier_thresh, None, 0, pnum)
	num = pnum[0]
	if (nms): do_nms_obj(dets, num, meta.classes, nms)

	res = []
	for j in range(num):
		for i in range(meta.classes):
			if dets[j].prob[i] > 0:
				b = dets[j].bbox
				res.append((meta.names[i], dets[j].prob[i], (b.x, b.y, b.w, b.h)))
	res = sorted(res, key=lambda x: -x[1])
	free_image(im)
	free_detections(dets, num)
	return res

def main(args):
	net = load_net("/home/zjw/darknet/cfg/yolov3-tiny.cfg", "/home/zjw/darknet/backup/yolov3-tiny_120000.weights", 0)
	meta = load_meta("/home/zjw/darknet/cfg/voc.data")
	ic = image_converter(net,meta)
	rospy.init_node('image_converter', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
		cv2.destroyAllWindows()

if __name__ == '__main__':
	main(sys.argv)
