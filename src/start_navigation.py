#!/usr/bin/env python
import rospy, message_filters, tf, math, numpy as np, sys, random
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from utils import Utils
from navigation import PotentialFieldsAlgorithm
from PID import PointPID

if __name__ == '__main__':
	nav = PotentialFieldsAlgorithm(3.)
	try:
		rospy.init_node('robo_navegacao', anonymous=True)
		odomPub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		laserMsg = message_filters.Subscriber('base_scan', LaserScan)
		odomMsg = message_filters.Subscriber('base_pose_ground_truth', Odometry)
		ts = message_filters.TimeSynchronizer([laserMsg, odomMsg], 10)
		ts.registerCallback(nav.response)
		rate = rospy.Rate(10) # 10hz
		it = 0
		while not rospy.is_shutdown():
			if it%100 == 0:
				pt = (random.randint(1,19), random.randint(1,19))
				nav._goal = PointPID(x=pt[0], y=pt[1])
			odomPub.publish(nav._msg)
			rate.sleep()
			it += 1

	except rospy.ROSInterruptException:
		pass
