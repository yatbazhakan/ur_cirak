#!/usr/bin/env python
import sys
import rospy
import random
import tf
import math
import copy
from moveit_commander import RobotCommander,MoveGroupCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import *
from gazebo_msgs.msg import *
def callback(states):
        scene = PlanningSceneInterface()
        robot = RobotCommander()
        arm =  MoveGroupCommander("manipulator")
        arm.set_planner_id("RRTConnectkConfigDefault")
	arm.set_num_planning_attempts(20)
	arm.allow_replanning(True)
	pose =copy.deepcopy(states.pose[-1])
	temp = tf.transformations.euler_from_quaternion((pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w))
        quaternion = tf.transformations.quaternion_from_euler(math.pi, temp[1], temp[2])   
        pose.position.z += 0.22
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        print  pose
        arm.set_pose_target(pose)
        move_plan = arm.plan()
        i = 0
        while(not move_plan.joint_trajectory.joint_names):
	   move_plan = arm.plan()
           i+=1
           if(i==5):break	
        state = arm.execute(move_plan)
if __name__=='__main__':
    #Init
    libraries = {1 : "RRTConnect" , 2 : "RRTStar"}
    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_py_demo', anonymous=True)
    rospy.Subscriber("/gazebo/model_states",ModelStates,callback)
    rospy.sleep(3.0)
    rospy.spin()
    
roscpp_shutdown()
