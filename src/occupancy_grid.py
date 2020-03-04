#!/usr/bin/env python
############################
# Occupancy grid algorithm #
############################
import rospy, message_filters, tf, math, numpy as np, sys
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from utils import Utils

class OccupancyGridAlgorithm(object):
	def __init__(self, laser_range=5.0):
		self._l0 = self.logOdds(.4)
		self._z_max = laser_range
		self._map = self.create_map()

	def create_map(self):
		test_map = OccupancyGrid()
		#Default config
		test_map.info.resolution = .1	#m/cell
		test_map.info.width = 200		#cells
		test_map.info.height = 200		#cells
		test_map.data = []
		#Initialize
		p0 = self.calculateLogOddsProbability(self._l0)*100
		for i in xrange(0, test_map.info.width*test_map.info.height):
			test_map.data.append( p0 )
		return test_map

	def localToMap(self, posicao):
		res = self._map.info.resolution
		xP = int(math.floor(posicao.x / res))
		yP = int(math.floor(posicao.y / res))
		return (xP, yP)

	def getObjectDetectedPosition(self, data, i, fov=180, sz_sp=180):
		fov = math.pi/180*fov
		a = i*(fov/sz_sp) - fov/2
		return (math.cos(a)*data, math.sin(a)*data)

	def matrixToArray(self, x, y, width):
		return int(x*width + y)

	def posCenterGrid(self, x,y,res):
		return (x*res+res/2, y*res+res/2)

	def inverseSensorModel(self, robotXY, robot_theta, pointXY, point_angle, laser_data, map_res):
		global z_max
		r = math.sqrt( (pointXY[0]-robotXY[0])**2 + (pointXY[1]-robotXY[1])**2 )
		if laser_data < self._z_max and abs(r-laser_data)<map_res*.6:
		 	return .9
		elif r < laser_data:
		 	return .3
		return .4

	def calculateLogOddsProbability(self, lOdds):
		return 1 - (1/(1+math.exp(lOdds)))

	def logOdds(self, p):
		if p >= 1:
			p = 0.999
		elif p <= 0:
			p = 0.001
		return math.log(p/(1-p))

	def response(self, laserData, odometryData):
		posicaoAtual = odometryData.pose.pose.position
		xI,yI = self.localToMap(posicaoAtual)
		orientacaoAtual = Utils.quaternionToEucliean(odometryData.pose.pose.orientation)
		sample_size = len(laserData.ranges)
		sAng = math.sin(orientacaoAtual.z)
		cAng = math.cos(orientacaoAtual.z)
		g = np.array([[cAng, -sAng, 0], [sAng, cAng, 0], [0, 0 , 1]], dtype=float)
		for i,d in enumerate( laserData.ranges ):
			obj_pos = self.getObjectDetectedPosition(d, i, 360,sample_size)
			p = np.transpose(np.array([obj_pos[0], obj_pos[1], 0], dtype=float))
			r = np.dot(g, p)
			positionObj = Point(x=r[0]+posicaoAtual.x, y=r[1]+posicaoAtual.y, z=r[2]+posicaoAtual.z)
			xF,yF = self.localToMap(positionObj)
			listPoints = Utils.bresenham( (xI,yI), (xF,yF) )
			l_angle = i*(math.pi/180) - math.pi/2
			for l in listPoints:
				pos = self.matrixToArray(l[0], l[1], self._map.info.width)
				if pos > self._map.info.width * self._map.info.height or pos < 0:
					continue
				p = self._map.data[pos]/100
				ism = self.inverseSensorModel( self.posCenterGrid(xI,yI,self._map.info.resolution), orientacaoAtual.z, self.posCenterGrid(l[0],l[1],self._map.info.resolution), l_angle, d, self._map.info.resolution)
				lt = self.logOdds(p) + self.logOdds(ism) - self._l0
				self._map.data[pos] =  self.calculateLogOddsProbability(lt) * 100
