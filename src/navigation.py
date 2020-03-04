#!/usr/bin/env python
import rospy, message_filters, tf, math, numpy as np, random
from PID import PointPID, RobotConfig
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from utils import *

PI = 3.1415926535897

class PotentialFieldsAlgorithm():

	def __init__(self, laser_range=5.0):
		self._msg = Twist()
		self._goal = None
		self._min_distance = laser_range - .5 

	def atractionForce(self, k, pos):
		return -k*(pos - self._goal)

	def repulsionForce(self, k, dist, pos, obstaclePos):
		minDist = self._min_distance
		if dist <= minDist:
			return k*(1./dist - 1./minDist)*(1./(dist**2)) * (pos-obstaclePos)/dist
		return PointPID()

	def control(self, msg, linear, angular):
		ang = angular.z
		v = linear.x*math.cos(ang) + linear.y*math.sin(ang)
		w = math.atan2(linear.y, linear.x) - ang
		msg.linear.x = v
		msg.angular.z = w

	def getObjectDetectedPosition(self, data, i, fov=360, sz_sp=720):
		fov = PI/180*fov
		a = i*(fov/sz_sp) - fov/2
		return PointPID(x=math.cos(a)*data, y=math.sin(a)*data)

	def calculateRepulsionSum(self, k, laserData, position, orientation):
		repAcum = PointPID()
		sample_size = len(laserData)
		for i,d in enumerate( laserData ):
			obj_pos = self.getObjectDetectedPosition(d, i, 360, sample_size)
			sAng = math.sin(orientation.z)
			cAng = math.cos(orientation.z)
			g = np.array([[cAng, -sAng, 0], [sAng, cAng, 0], [0, 0 , 1]], dtype=float)
			p = np.transpose(np.array([obj_pos.x, obj_pos.y, obj_pos.z], dtype=float))
			r = np.dot(g, p)
			positionObj = PointPID(x=r[0], y=r[1], z=r[2]) + position
			repAcum += self.repulsionForce(k, d, position, positionObj)
		return repAcum

	def response(self, laserData, odometryData):
		p = PointPID()
		p.copy(odometryData.pose.pose.position)
		o = Utils.quaternionToEucliean(odometryData.pose.pose.orientation)
		fat = self.atractionForce(30, p)
		frep = self.calculateRepulsionSum(10, laserData.ranges, p, o)
		fres = fat + frep
		self.control(self._msg, fres, o)
