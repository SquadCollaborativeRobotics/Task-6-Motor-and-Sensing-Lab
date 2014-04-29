#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

def callback(data):
    # rospy.loginfo(rospy.get_caller_id()+"I heard %s",data.data)
    print data.pose.position.x, data.pose.position.y, data.pose.orientation.z, data.pose.orientation.w

    
def listener():

    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The 
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaenously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/move_base_simple/goal", PoseStamped, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
        
if __name__ == '__main__':
    listener()