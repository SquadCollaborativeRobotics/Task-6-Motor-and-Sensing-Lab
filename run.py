#!/usr/bin/env python

from argparse import ArgumentParser
from os import system
import time

if __name__ == '__main__':

	parser = ArgumentParser( description = """For launching SCR Software.""")
	parser.add_argument('-m', '--mode', help='Mode to run robot in, default is sim. [options: "real", "sim"]')
	parser.add_argument('-H', '--hardware', help='Hardware to run, default is none. [options: "p3", "custom", "none"]')
	parser.add_argument('-g', '--global_planner', help='global_planner to run, default is none. [options: "none", "naive", "closest_robot", "closest_waypoint"]')
	parser.add_argument('-Nc', '--Number_collector', help='number of collector robots to launch, default is 0.  This or names must be set.')
	parser.add_argument('-Nb', '--Number_bin', help='number of bin robots to launch, default is 0.  This or names must be set.')
	parser.add_argument('-e', '--environment', help='Gazebo Folder to get launch under. options are ["cubicle", "springdemo"]')
	parser.add_argument('-o', '--no_environment', action='store_true', dest='no_environment', help="don't launch environment (map server for real, gazebo env for sim)")
	parser.add_argument('-rc', '--robot_controller', action='store_true', dest='rc', help="Dictates whether to start robot controllers on the robots")
	parser.add_argument('-x', '--init_x', help="X positions for AMCL initialization")
	parser.add_argument('-y', '--init_y', help="Y positions for AMCL initialization")

	args = parser.parse_args()

	pkg = 'collectorbot_gazebo'
	gp_pkg = 'global_planner'
	rc_filename = 'robot_controller.launch'
	hardware = None
	glaunch = None
	hw_params = ''

	if args.environment != None:
		env = args.environment
	else:
		env = "cubicle"

	if args.init_x != None:
		xi = args.init_x.split()
	if args.init_y != None:
		yi = args.init_y.split()


	if args.mode != None and args.mode in 'real':
		pkg = 'scr_proto'
	  	filename = 'demo.launch'
	  	base_frame = 'base_link'
	  	cam_mode = "1"

	  	if args.hardware != None:
			if args.hardware in 'custom':
				hw_filename = 'custom'

			elif args.hardware in 'p3':
				hw_filename = 'p3'

			elif args.hardware in 'none':
				pass
			else:
				print("Unrecognized Hardware type.  Options are: [p3, custom, none]")

	else:
	  	filename = env + '_env.launch'
	  	hw_filename = 'ns_simbot'
	  	base_frame = 'robot_center'
	  	cam_mode = "0"

	if args.global_planner != None:
	  	glaunch = args.global_planner

	  	if args.global_planner in 'naive':
	  		glaunch = args.global_planner

	  	elif args.global_planner in 'closest_robot':
	  		glaunch = args.global_planner

	  	elif args.global_planner in 'closest_waypoint':
	  		glaunch = args.global_planner

	  	elif args.global_planner in 'none':
	  		pass

	  	else:
	  		print("Unrecognized global planner.  Options are: [naive, robot, waypoint, none]")

	# Launch Environment (Map Server for Real, and Gazebo with Environment for sim)
	if not args.no_environment:
		system('xterm -hold -e roslaunch ' + pkg + ' ' + filename + ' &')
		if args.mode != None and args.mode in 'sim':
			time.sleep(5)
		else:
			time.sleep(2)

	# Launch Robots
	names_c = []
	names_b = []
	if args.Number_collector != None or args.Number_bin != None:
	  	if args.Number_collector != None:
	  		names_c = ['robot' + str(i) for i in range(1, int(args.Number_collector) + 1)]
	  	if args.Number_bin != None:
	  		names_b = ['robot' + str(i) for i in range(1 + int(args.Number_collector), int(args.Number_collector) + int(args.Number_bin) + 1)]
	  	#if args.Number_bin != None and args.Number_collector != None:
	  	names = names_c + names_b
	  	if len(xi) < len(names) or len(yi) < len(names):
	  		print("Not enough initial positions given, make sure that #x arg = #y arg = #Nc + #Nb")
	  		exit(0)
	  	elif len(xi) > len(names) or len(yi) > len(names):
	  		print("Too many initial positions given, make sure that #x arg = #y arg = #Nc + #Nb")
	  		print("Continuing to try and launch with first initial positions")

	  	i = 0
	  	for name in names:
	  		# Launch Hardware
	  		hw_params = 'init_pose_x:=' + xi[i] + ' ' + 'init_pose_y:=' + yi[i]
	  		system('xterm -hold -e roslaunch ' + pkg + ' ' + hw_filename + '.launch' +  ' robot:=' + name + ' ' + hw_params + ' &')
		  	# Launch April Tags
	  		system('xterm -hold -e roslaunch ' + gp_pkg + ' ' + 'april_tags.launch' + ' robot:=' + name + ' mode:=' + cam_mode + ' &')
	  		i = i + 1
			time.sleep(0.5)

	  	# Launch Nav
	  	i = 0
	  	for name in names:
	  		hw_params = 'init_pose_x:=' + xi[i] + ' ' + 'init_pose_y:=' + yi[i]
	  		system('xterm -hold -e roslaunch ' + pkg + ' ' + hw_filename + '_nav.launch' + ' robot:=' + name + ' ' + hw_params + ' &')
	  		i = i + 1
			time.sleep(0.5)

	  	# Launch Robot Controllers
	  	if args.rc:
			for name in names_c:
				su = 0
				sc = 3
				type = 'collector'
				rid = name[-1]
		  		system('xterm -hold -e roslaunch ' + gp_pkg + ' ' + rc_filename + ' robot:=' + name + ' base_frame:=' + base_frame + ' robot_id:=' + rid + ' type:=' + type + ' storage_used:=' + str(su) + ' storage_capacity:=' + str(sc) + ' &')
				time.sleep(0.5)

		  	for name in names_b:
				su = 0
				sc = 10
				type = 'bin'
				rid = name[-1]
		  		system('xterm -hold -e roslaunch ' + gp_pkg + ' ' + rc_filename + ' robot:=' + name + ' base_frame:=' + base_frame + ' robot_id:=' + rid + ' type:=' + type + ' storage_used:=' + str(su) + ' storage_capacity:=' + str(sc) + ' &')
				time.sleep(0.5)

	  # Launch Global Planner
	if glaunch != None:
		if env == "cubicle":
		  	system('xterm -hold -e roslaunch ' + gp_pkg + ' global_planner_script.launch' + ' planner:=' + glaunch +' waypoint_file:=cubicle.points &')
		else:
		  	system('xterm -hold -e roslaunch ' + gp_pkg + ' global_planner_script.launch' + ' planner:=' + glaunch +' waypoint_file:=falldemo.points &')
