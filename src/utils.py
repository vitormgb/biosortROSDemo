#!/usr/bin/env python
import tf, math
from PID import PointPID

PI = 3.1415926535897

class Utils:

	@staticmethod
	def degreeToRadians(angle):
		return (angle*2*PI)/360

	@staticmethod
	def radiansToDegree(angle):
		return angle*360/(2*PI)

	@staticmethod
	def positionToList(position):
		return [position.x, position.y, position.z, position.w]

	@staticmethod
	def euclideanDist(p1, p2):
		return math.sqrt( (p1.x-p2.x)**2 + (p1.y-p2.y)**2 )

	@staticmethod
	def orientation(p1, p2):
		return math.atan2( (p2.y-p1.y), (p2.x-p1.x) )

	@staticmethod
	def quaternionToEucliean(quat):
		orientacao_atual = tf.transformations.euler_from_quaternion(Utils.positionToList(quat))
		return PointPID(x=orientacao_atual[0], y=orientacao_atual[1], z=orientacao_atual[2])

	@staticmethod
	def bresenham(start, end):
		# Setup initial conditions
		x1, y1 = start
		x2, y2 = end
		dx = x2 - x1
		dy = y2 - y1
		# Determine how steep the line is
		is_steep = abs(dy) > abs(dx)
			# Rotate line
		if is_steep:
			x1, y1 = y1, x1
			x2, y2 = y2, x2
		# Swap start and end points if necessary and store swap state
		swapped = False
		if x1 > x2:
			x1, x2 = x2, x1
			y1, y2 = y2, y1
			swapped = True
		# Recalculate differentials
		dx = x2 - x1
		dy = y2 - y1
		# Calculate error
		error = int(dx / 2.0)
		ystep = 1 if y1 < y2 else -1
		# Iterate over bounding box generating points between start and end
		y = y1
		points = []
		for x in range(x1, x2 + 1):
			coord = (y, x) if is_steep else (x, y)
			points.append(coord)
			error -= abs(dy)
			if error < 0:
				y += ystep
				error += dx
		# Reverse the list if the coordinates were swapped
		if swapped:
			points.reverse()
		return points
