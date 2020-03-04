#!/usr/bin/env python
import rospy, message_filters, tf, math, numpy as np, sys, random
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from utils import Utils
from occupancy_grid import OccupancyGridAlgorithm
from PID import PointPID

if __name__ == '__main__':
	occMap = OccupancyGridAlgorithm(3.)
	try:
		rospy.init_node('robo_mapeamento', anonymous=True)
		mapPub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
		laserMsg = message_filters.Subscriber('base_scan', LaserScan)
		odomMsg = message_filters.Subscriber('base_pose_ground_truth', Odometry)
		ts = message_filters.TimeSynchronizer([laserMsg, odomMsg], 10)
		ts.registerCallback(occMap.response)
		rate = rospy.Rate(10) # 10hz
		while not rospy.is_shutdown():
			mapPub.publish(occMap._map)
			rate.sleep()

	except rospy.ROSInterruptException:
		pass
