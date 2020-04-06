#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
import matplotlib as mpl
from matplotlib.patches import Polygon, Rectangle, Circle
import numpy as np
import math
import time
from py_coverage.grid_map import GridMap
from py_coverage.grid_based_sweep_coverage_path_planner import planning
from py_coverage.tools import define_polygon, polygon_contains_point
from std_msgs.msg import Float32MultiArray
from minimization.msg import WaypointList
from geometry_msgs.msg import Point


MAX_ACCELERATION = 1.0
MAX_VELOCITY = 0.45

def define_flight_area(initial_pose):
	plt.grid()
	while True:
		try:
			num_pts = int( input('Enter number of polygonal vertixes: ') )
			break
		except:
			print('\nPlease, enter an integer number.')
	while True:
		flight_area_vertices = define_polygon(num_pts)
		if polygon_contains_point(initial_pose, flight_area_vertices):
			break
		plt.clf()
		plt.grid()
		print('The robot is not inside the flight area. Define again.')
	return flight_area_vertices


class Params:
	def __init__(self):
		self.numiters = 1000
		self.animate = 1
		self.dt = 0.1
		self.goal_tol = 0.25
		self.max_vel = 0.5 # m/s
		self.min_vel = 0.1 # m/s
		self.sensor_range_m = 0.25 # m
		self.time_to_switch_goal = 5.0 # sec #inactive for now
		self.sweep_resolution = 0.4 # m


def py_coverage_path_generation():
	# Code block taken from main.py
	obstacles = [
		# np.array([[0.7, -0.9], [1.3, -0.9], [1.3, -0.8], [0.7, -0.8]]) + np.array([-1.0, 0.5]),
		# np.array([[0.7, -0.9], [1.3, -0.9], [1.3, -0.8], [0.7, -0.8]]) + np.array([-1.0, 1.0]),
		# np.array([[0.7, -0.9], [0.8, -0.9], [0.8, -0.3], [0.7, -0.3]]) + np.array([-1.5, 1.0]),        
	
		np.array([[-0.3, -0.4], [0.3, -0.4], [0.3, 0.1], [-0.3, 0.1]]) * 0.5
	]
	# initial state = [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
	state = np.array([0, 0.2, np.pi/2, 0.0, 0.0])
	traj = state[:2]
	params = Params()
	# plt.figure(figsize=(10,10))

	flight_area_vertices = define_flight_area(state[:2])
	posset=[]
	# flight_area_vertices = np.array([[-1, -1], [-0.3, -1], [-0.3, -0.4], [0.3, -0.4], [0.3, -1], [1,-1], [1,1], [-1,1]])
	gridmap = GridMap(flight_area_vertices, state[:2])
	gridmap.add_obstacles_to_grid_map(obstacles)
	#obstacle x, y coordinates
	ox = flight_area_vertices[:,0].tolist() + [flight_area_vertices[0,0]]
	oy = flight_area_vertices[:,1].tolist() + [flight_area_vertices[0,1]]
	reso = params.sweep_resolution

	goal_x, goal_y, gmap, covmap = planning(ox, oy, reso)
	# covmap.plot_grid_map(axes[0])

	# goal = [x, y], m
	goali = 0
	goal = [goal_x[goali], goal_y[goali]]
	t_prev_goal = time.time()
	gridmap.draw_map(obstacles)
	iter = 0
	### End code block taken from main.py

	return goal_x, goal_y


def py_coverage_min_cb():
	# Create Publisher
	pub = rospy.Publisher('/py_coverage/trajectory', WaypointList, queue_size=10)
	rospy.init_node('trajectory_publisher', anonymous=True)
	rate = rospy.Rate(10) # 10hz
	
	while not rospy.is_shutdown():
		msg = WaypointList()

		# This function does the py_coverage side
		goal_x, goal_y = py_coverage_path_generation()

		# Now we have the waypoints --> put in msg
		msg.list_size = len(goal_x)
		msg.maxAcceleration = MAX_ACCELERATION
		msg.maxVelocity = MAX_VELOCITY


		
		for i in range(len(goal_x)):
			wp = Point()
			wp.x = goal_x[i]
			wp.y = goal_y[i]
			msg.wp_list.append(wp)


		# Publish
		pub.publish(msg)
		print('publishing')
		rate.sleep()




if __name__ == '__main__':
	try:
		py_coverage_min_cb()
	except rospy.ROSInterruptException:
		pass