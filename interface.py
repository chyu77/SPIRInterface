#!/usr/bin/env python
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import PoseStamped

class image_projection_transformator:
	def __init__(self):
		self.image_pub = rospy.Publisher("selected_image_if",Image, queue_size=1)
		self.bridge = CvBridge()
		self.model_sub = rospy.Subscriber("/selected_cc", Image, self.kakogazomodelCallBack)
		self.bg = cv2.imread("/home/haptic-pc/master_project/src/kakogazo_operation/src/background.png")
		self._buffer_background = 0 # totally black
		img2gray = cv2.cvtColor(self.bg, cv2.COLOR_BGR2GRAY)
		ret, mask = cv2.threshold(img2gray, 3, 255, cv2.THRESH_BINARY)
		self.mask_inv = cv2.bitwise_not(mask)
		self.img2_fg = cv2.bitwise_and(self.bg, self.bg, mask=mask)
		self.image_sub = rospy.Subscriber("/selected_image", Image, self.imageCallBack)
		self.dist_sub = rospy.Subscriber("/mobile_manipulator_mock/mobile_manipulator/distance", Float32, self.distCallBack)
		self.lcoll_sub = rospy.Subscriber("mobile_manipulator_mock/finger_l_coll_output", Bool, self.lcollCallBack)
		self.rcoll_sub = rospy.Subscriber("mobile_manipulator_mock/finger_r_coll_output", Bool, self.rcollCallBack)

		self.dist = 2.0
		self.point_x = 600
		self.point_y = 320 #height 120~320
		self.l_coll = False
		self.r_coll = False

	def distCallBack(self,dist):
		self.dist = dist.data
	def lcollCallBack(self,lcoll):
		self.l_coll = lcoll.data
	def rcollCallBack(self,rcoll):
		self.r_coll = rcoll.data

	def imageCallBack(self,data):
		try:
			image = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
		img1_bg = cv2.bitwise_and(image, image, mask=self.mask_inv)
		dst = cv2.add(img1_bg, self.img2_fg)
		# cv2.rectangle(dst, (20,320), (220, 470), (150, 150, 150), -1)
		dst = cv2.addWeighted(dst,0.7,image,0.3,0)
		self._buffer_background = dst

	def kakogazomodelCallBack(self, data):
		try:
			cv_image_t = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
		background_dst = self._buffer_background

		img2gray = cv2.cvtColor(cv_image_t, cv2.COLOR_BGR2GRAY)
		ret, mask = cv2.threshold(img2gray, 10, 255, cv2.THRESH_BINARY)
		mask_inv = cv2.bitwise_not(mask)
		img2_fg = cv2.bitwise_and(cv_image_t, cv_image_t, mask=mask)
		#img1_bg = cv2.bitwise_and(background_dst, background_dst, mask=mask_inv)
		dst = cv2.add(background_dst, img2_fg)

		self.point_y = int(120 + 100 * self.dist)
		if self.point_y > 320:
			self.point_y = 320

		screen_dist = "%.1fm" % float(self.dist)
		cv2.putText(dst, screen_dist, (572,365), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255, 255, 255), 1,cv2.CV_AA)
		if self.dist > 1.5 :
			screen_zone = "Status: Outside Zone"
			cv2.circle(dst, (self.point_x,self.point_y), 8, (100, 255, 100), -1,cv2.CV_AA)  #green light
		elif self.dist > 0.5:
			screen_zone = "Status: ROI Zone"
			cv2.circle(dst, (self.point_x,self.point_y), 8, (100, 255, 255 ), -1,cv2.CV_AA)  #yellow
		else :
			screen_zone = "Status: Target Zone"
			cv2.circle(dst, (self.point_x,self.point_y), 8, (100, 100, 255), -1,cv2.CV_AA)  #red
		cv2.circle(dst, (self.point_x,self.point_y), 8, (0, 150, 0), 1,cv2.CV_AA)  # FOR DISTANCE
		cv2.putText(dst, screen_zone, (170,25), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255, 255, 140), 1,cv2.CV_AA)
		# for gripper
		if self.l_coll == False and self.r_coll == False:
			cv2.circle(dst, (560,18) , 10 , (30,255,30), -1, cv2.CV_AA)
			cv2.circle(dst, (600,18) , 10 , (30,255,30), -1, cv2.CV_AA)
		elif self.l_coll == True and self.r_coll == True:
			cv2.circle(dst, (560,18) , 10 , (30,30,255), -1, cv2.CV_AA)
			cv2.circle(dst, (600,18) , 10 , (30,30,255), -1, cv2.CV_AA)
		else :
			if self.l_coll == True:
				cv2.circle(dst, (560,18) , 10 , (30,255,255), -1, cv2.CV_AA)
			if self.r_coll == True:
				cv2.circle(dst, (600,18) , 10 , (30,255,255), -1, cv2.CV_AA)
		cv2.circle(dst, (560,18) , 10 , (0,0,0), 1, cv2.CV_AA)
		cv2.circle(dst, (600,18) , 10 , (0,0,0), 1, cv2.CV_AA)

		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(dst, "bgr8"))
		except CvBridgeError as e:
			print(e)
		#fgmask_save = cv2.addWeighted(background_dst,1.0,cv_image_t,1.0,0)
		# try:
		# 	self.image_pub.publish(self.bridge.cv2_to_imgmsg(fgmask_save, "bgr8"))
		# except CvBridgeError as e:
		# 	print(e)

def main(args):
	rospy.init_node('image_projection_transformator', anonymous=True)
	ipt = image_projection_transformator()
	rospy.spin()

if __name__ == '__main__':
	main(sys.argv)
