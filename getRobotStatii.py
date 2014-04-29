#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('global_planner')
from global_planner.msg import RobotStatus
from math import acos, pi
import os

robots = dict()

state_names = {
0: "NONE",
1: "WAITING",
2: "WAITING_TAG_SPOTTED",
3: "WAITING_TAG_FINISHED",
9: "WAITING_FINISHED",
10: "NAVIGATING",
12: "NAVIGATING_TAG_SPOTTED",
13: "NAVIGATING_TAG_FINISHED",
19: "NAVIGATING_FINISHED",
20: "DUMPING",
29: "DUMPING_FINISHED",
39: "COLLECTING_FINISHED",
99: "UNINITIALIZED",
}

type_names = {
    0: "Any",
    1: "Collector",
    2: "Bin"
}

def callback(data):
    # rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.data)
    angle = 2*acos(data.pose.orientation.w) # radians
    angle_deg = int(angle * 180 / pi)

    robots[data.id] = {'pos': [data.pose.position.x, data.pose.position.y],
                       'rot': angle_deg,
                       'name': data.name,
                       'state': data.state,
                       'storage_used': data.storage_used,
                       'storage_capacity': data.storage_capacity,
                       'type': data.type,
                       'taskID': data.taskID}
    os.system('clear')
    for robot_id in robots:
        print "Robot(%d | %s):" % (robot_id, type_names[ robots[robot_id]['type'] ]), robots[robot_id]['name']
        print "Position: (%.2f, %.2f)" % (robots[robot_id]['pos'][0], robots[robot_id]['pos'][1]), "Orientation:", robots[robot_id]['rot']
        print "State: ", state_names[ robots[robot_id]['state'] ]
        print "Storage: ", robots[robot_id]['storage_used'],"/", robots[robot_id]['storage_capacity']
        print "Task ID: ", robots[robot_id]['taskID']
    
def listener():

    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The 
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaenously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/robot_status", RobotStatus, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
        
if __name__ == '__main__':
    listener()