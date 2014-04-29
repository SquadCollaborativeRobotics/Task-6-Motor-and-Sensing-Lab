#!/usr/bin/env python

from argparse import ArgumentParser
from os import system, environ
import subprocess
import time

if __name__ == '__main__':

	parser = ArgumentParser( description = """For launching SCR Software.""")
	parser.add_argument('-m', '--mode', help='Mode to run robot in, default is sim. [options: "real", "sim"]')
	parser.add_argument('-a', '--april_tags', action='store_false', dest='a', help='Boolean to set april tag processor launch.  Default is true.')
	parser.add_argument('-H', '--hardware', help='Hardware to run, default is none. [options: "p3", "custom", "none"]')
	parser.add_argument('-g', '--global_planner', help='global_planner to run, default is none. [options: "none", "naive", "closest_robot", "closest_waypoint"]')
	parser.add_argument('-Nc', '--Number_collector', help='number of collector robots to launch, default is 0.  This or names must be set.')
	parser.add_argument('-Nb', '--Number_bin', help='number of bin robots to launch, default is 0.  This or names must be set.')
	parser.add_argument('-N', '--Names', help='names of the robots to use.  Deault is robot[n] so if one robot -> robot1')
	parser.add_argument('-e', '--environment', help='Gazebo Folder to get launch under. options are ["cubicle", "springdemo"]')
	parser.add_argument('-o', '--no_environment', action='store_true', dest='no_environment', help="don't launch environment (map server for real, gazebo env for sim)")
	parser.add_argument('-rc', '--robot_controller', action='store_true', dest='rc', help="Dictates whether to start robot controllers on the robots")
	parser.add_argument('-rid', '--robot_id', help="specifies an rid index to start from while launching robots")
	parser.add_argument('-u', '--uri', help="specifies who the ROS_MASTER is (assumed to not change if not given)")
	parser.add_argument('-x', '--init_x', help="X positions for AMCL initialization")
	parser.add_argument('-y', '--init_y', help="Y positions for AMCL initialization")

	args = parser.parse_args()

	pkg = 'collectorbot_gazebo'
	gp_pkg = 'global_planner'
	rc_filename = 'robot_controller.launch'
	hardware = None
	glaunch = None
	hw_params = ''
	delay_flag = 0
	kinect = '0'

	system('sudo service chrony restart')
	proc = subprocess.Popen(['rospack find global_planner'], stdout=subprocess.PIPE, shell=True)
	(out, err) = proc.communicate()
	print ("output = "+out)
	out = out + "/logger.config"
	environ['ROSCONSOLE_CONFIG_FILE'] = out
	print("set log config file to: "+out)

	# Parse Arguments and instantiate to proper default value
	# Number Collectors and Number Bins

	if args.uri:
		if args.uri in ['sam']:
			master = 'sam-UX31E'
		elif args.uri in ['alex']:
			master = 'TheBroPro'
		elif args.uri in 'shawn':
			master = 'shawn-ubuntu'
		elif args.uri in 'aaron':
			master = 'LinuxBox'

		master_uri = 'http://' + master + ':11311'
		environ['ROS_MASTER_URI'] = master_uri

	if args.Number_collector != None:
		nc = int(args.Number_collector)
	else:
		nc = 0
	if args.Number_bin != None:
		nb = int(args.Number_bin)
	else:
		nb = 0

	# Initial X and Y Coords
	if args.init_x != None:
		xi = args.init_x.split()
	else:
		xi = []
	if args.init_y != None:
		yi = args.init_y.split()
	else:
		yi = []

	# Environment
	if args.environment != None:
		env = args.environment
	else:
		env = "cubicle"

	arg_names = []
	if args.Names != None:
		arg_names = args.Names.split()


	# Pull proper package and hardware launch files
	if args.mode != None and args.mode in 'real':
		pkg = 'scr_proto'
	  	filename = 'demo.launch'
	  	base_frame = 'base_link'
	  	cam_mode = "1"

	  	if args.hardware != None:
			if args.hardware in 'custom':
				hw_filename = 'custom'
				kinect = "0"

			elif args.hardware in 'p3':
				hw_filename = 'p3'
				kinect = "0"

			elif args.hardware in 'none':
				pass
			else:
				print("Unrecognized Hardware type.  Options are: [p3, custom, none]")

	else:
	  	filename = env + '_env.launch'
	  	hw_filename = 'ns_simbot'
	  	base_frame = 'robot_center'
	  	cam_mode = "0"

	# Global Planner launch string parameters
	if args.global_planner != None:

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
		# First Core coming up, give time to start properly
		time.sleep(5)

	else:
		# First Core Doesn't come up here, needs to come up later
		delay_flag = 1

	# Launch Robots
	names_c = []
	names_b = []
	rids = []
	if args.Number_collector != None or args.Number_bin != None:

	  	if args.Number_collector != None:
	  		if args.Names == None:
	  			names_c = ['robot' + str(i) for i in range(1, nc + 1)]
	  		else:
	  			names_c = arg_names[:nc]
	  		rids.extend(range(len(names_c)))

	  	if args.Number_bin != None:
	  		if args.Names == None:
	  			names_b = ['robot' + str(i) for i in range(1 + nc, nc + nb + 1)]
	  		else:
	  			names_b = arg_names[nc:]
	  		rids.extend(range(len(names_c), len(names_b)))

	  	#if args.Number_bin != None and args.Number_collector != None:
	  	names = names_c + names_b
	  	if len(names) != (nb + nc):
			print("Right number of names not given! You gave [or default was]" + str(len(names)) + " but expecting " + str(nc+nb) + " based on -Nc and -Nb.")
			exit(0)

	  	if len(xi) < len(names) or len(yi) < len(names):
	  		print("Not enough initial positions given, make sure that #x arg = #y arg = #Nc + #Nb")
	  		print("Initial x's given are " + str(len(xi)) + ", expected " + str(len(names)))
	  		print("Initial y's given are " + str(len(yi)) + ", expected " + str(len(names)))
	  		exit(0)
	  	elif len(xi) > len(names) or len(yi) > len(names):
	  		print("Too many initial positions given, make sure that #x arg = #y arg = #Nc + #Nb")
	  		print("Continuing to try and launch with first initial positions")

	  	i = 0
	  	for name in names:
	  		# Launch Hardware
	  		hw_params = 'init_pose_x:=' + xi[i] + ' ' + 'init_pose_y:=' + yi[i]
	  		system('xterm -hold -e roslaunch ' + pkg + ' ' + hw_filename + '.launch' +  ' robot:=' + name + ' ' + hw_params + ' &')
	  		if delay_flag:
	  			time.sleep(2)
		  	# Launch April Tags
		  	if args.a:
	  			system('xterm -hold -e roslaunch ' + gp_pkg + ' ' + 'april_tags.launch' + ' robot:=' + name + ' mode:=' + cam_mode + ' kinect:=' + kinect + ' &')
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
	  		if args.robot_id != None:
	  			j = int(args.robot_id)
	  		else:
	  			j = 1
			for name in names_c:
				su = 0
				sc = 3
				type = 'collector'
				rid = str(j)
		  		system('xterm -hold -e roslaunch ' + gp_pkg + ' ' + rc_filename + ' robot:=' + name + ' base_frame:=' + base_frame + ' robot_id:=' + rid + ' type:=' + type + ' storage_used:=' + str(su) + ' storage_capacity:=' + str(sc) + ' &')
				time.sleep(0.5)
				j = j + 1

		  	for name in names_b:
				su = 0
				sc = 10
				type = 'bin'
				rid = str(j)
		  		system('xterm -hold -e roslaunch ' + gp_pkg + ' ' + rc_filename + ' robot:=' + name + ' base_frame:=' + base_frame + ' robot_id:=' + rid + ' type:=' + type + ' storage_used:=' + str(su) + ' storage_capacity:=' + str(sc) + ' &')
				time.sleep(0.5)
				j = j + 1

	  # Launch Global Planner
	if glaunch != None:
		if env == "cubicle":
		  	system('xterm -hold -e roslaunch ' + gp_pkg + ' global_planner_script.launch' + ' planner:=' + glaunch +' waypoint_file:=cubicle.points &')
		else:
		  	system('xterm -hold -e roslaunch ' + gp_pkg + ' global_planner_script.launch' + ' planner:=' + glaunch +' waypoint_file:=springdemo_search_waypoints.points &')
