#!/usr/bin/env python
import rospy, math
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

PI = 3.1415926535897

class PointPID(Point):
	def copy(self,pt):
		self.x = pt.x
		self.y = pt.y
		self.z = pt.z

	def __add__(self, other):
		c = PointPID()
		c.x = self.x + other.x
		c.y = self.y + other.y
		c.z = self.z + other.z
		return c

	def __sub__(self, other):
		c = PointPID()
		c.x = self.x - other.x
		c.y = self.y - other.y
		c.z = self.z - other.z
		return c

	def __mul__(self, other):
		c = PointPID()
		c.x = self.x * other.x
		c.y = self.y * other.y
		c.z = self.z * other.z
		return c

	def __div__(self, other):
		c = PointPID()
		if type(other) != PointPID:
			return self.__rdiv__(other)
		c.x = self.x / other.x
		c.y = self.y / other.y
		c.z = self.z / other.z
		return c

	def __rmul__(self, other):
		c = PointPID()
		c.x = self.x * other
		c.y = self.y * other
		c.z = self.z * other
		return c

	def __rdiv__(self, other):
		c = PointPID()
		c.x = self.x / other
		c.y = self.y / other
		c.z = self.z / other
		return c


class RobotConfig:
	def __init__(self):
		self.position = PointPID()
		self.orientation = PointPID()
