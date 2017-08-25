#!/usr/bin/env python
import sys
import rospy
import random
import math
import copy
from moveit_commander import RobotCommander,MoveGroupCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped

"""
GLobal lists for target objects.
Target_poses : Pose list of created target objects
Picked       : List of part indexes to ensure to picked non-picked ones
"""
target_poses = []
picked = []
"""
----------------Function Name:  set_mid_state----------------
Definition  : Motion planning function for defined mid state
Robot       : Robot Commander Object
Scene       : Planning Scene Interface Object (current scene)
""" 
def set_mid_state(group,robot):
	pos = PoseStamped()
	pos.header.frame_id = robot.get_planning_frame()
        pos.pose.position.x =  	0.229537
        pos.pose.position.y =  0.261656
        pos.pose.position.z =  0.753969
	pos.pose.orientation.y = 1
	group.set_pose_target(pos)
    	pl = group.plan()
    	state =group.execute(pl)
	#while(not state):
	state =group.execute(pl)
"""
----------------Function Name: check_targets----------------
Definition  : Creates given number of objects within the distance provided. It prevents collision object overlapping.
	      It returns the pose list of the objects
Scene       : Planning Scene Interface Object (current scene)
Robot       : Robot Commander Object
Number      : Number of collision objects required to spawn
Distance    : Required minimum distance between each collision objects
"""
def check_targets(robot,scene,number,distance):
    print "Start Check"
    def al(type):
	if type == 'x':
	    return random.uniform(0.35,0.65)
	return random.uniform(-0.7,0.7)
    hemme = []
    cnt = 0
    while len(hemme) !=number :
        if cnt == 200:
            hemme = []
            cnt = 0
        cnt = cnt + 1
        box = PoseStamped()
        box.header.frame_id = robot.get_planning_frame()
        box.pose.position.z = 0.05
        box.pose.position.x = al('x')
        box.pose.position.y = al('y')
        flag = 1     
        for i in hemme:
            if abs(i.pose.position.x - box.pose.position.x) < distance  or  abs(i.pose.position.y - box.pose.position.y) < distance:
                flag = 0     
        if flag == 0:
            continue
        hemme.append(box)
    cnt = 0
    names = []
    for i in hemme:
        now = "part" + str(cnt)
        cnt = cnt + 1
        names.append(now)
        add_object(name=now,
		   x = i.pose.position.x,
		   y = i.pose.position.y,
		   z = i.pose.position.z,
		   d1 = 0.1,
		   d2 = 0.1,
		   d3 = 0.1,
		   robot = robot,
		   scene = scene,
		   typ =0)
    print "End Check!"	
    return hemme
	
"""
----------------Function Name: clean_scene----------------
Definition: Clears all non-attached objects from planning scene
Scene: PlanningSceneInterface Object (current scene)
"""	
def clean_scene(scene):
    scene.remove_world_object()	
def default_state_gripper(grp):
    joint_vals = grp.get_current_joint_values()
    if(len(joint_vals) >= 1):
	    joint_vals[0] = 0.0
	    grp.set_joint_value_target(joint_vals)
	    init_plan = grp.plan()
	    return grp.execute(init_plan)
"""
----------------Function Name: closed_state_gripper----------------
Definition   : Function that opens gripper and detachs the gripped object
Robot        : Robot Commander Object
Obj          : Name of the Object that is needed to detach
"""
def closed_state_gripper(robot, obj):
    def convert(width):
        return 0.77 - width / 0.15316 * 0.77
    width = scene.get_objects([obj])[obj].primitives[0].dimensions[0]
    width = convert(width)
    now = robot.endeffector.get_current_joint_values()[0]
    cnt = 0
    while abs(now - width) > 0.0001:
        now = robot.endeffector.get_current_joint_values()[0]
        cnt = cnt + 1
        tmp = width - abs(now-width) / 2.0
        robot.endeffector.set_joint_value_target('finger_joint', tmp)
        robot.endeffector.go()
	rospy.sleep(0.05)
        if cnt == 7:
            break
    rospy.sleep(1.0)   
    ret = robot.manipulator.attach_object(obj)
    return ret

"""
----------------Function Name: create_pose----------------
Description   : To create a pose object easily. Can be used to divide add_object function if needed.
(x,y,z)       : Position Attributes of Pose Message
(xo,yo,zo,wo) : Orientation Attributes of Pose Message (Default Value: 0)
"""
def create_pose(x,y,z,xo=0.0,yo=0.0,zo=0.0,wo=0.0):
	pos = PoseStamped()
	pos.header.frame_id = robot.get_planning_frame()
        pos.pose.position.x = x
        pos.pose.position.y = y
        pos.pose.position.z = z
	pos.pose.orientation.x = xo
	pos.pose.orientation.y = yo
	pos.pose.orientation.z = zo
	pos.pose.orientation.w = wo
	return pos
"""
----------------Function Name: create_dimensions----------------
Definition    : To create a dimension tuple easily. Can be used to divide add_object function if needed.
(d1,d2,d3)    : Dimensions of the object  (Default Value: 0.1)
"""
def create_dimensions(d1=0.1,d2=0.1,d3=0.1):
	return (d1,d2,d3)
"""     
----------------Function Name: add_object----------------
Description   : Adds an defined object to the given pose with given dimensions
Name          : Object Name
Pose	      : Pose of the Object (x,y,z,xo,yo,zo,wo)
Dimension     : Dimensions of the Obhect (Tuple) (d1,d2,d3)
Type          : Box(0),Sphere(1)
Robot         : Robot Commander Object
Scene	      : Planning Scene Interface Object(current scene)

!!!!!!!	d1 is radius for sphere i.e typ==1  !!!!!!!!!
"""
def add_object(name,x,y,z,robot,scene,xo=0.0,yo=0.0,zo=0.0,wo=0.0,d1=0.1,d2=0.1,d3=0.1,typ=0):
	pos = PoseStamped()
	pos.header.frame_id = robot.get_planning_frame()
        pos.pose.position.x = x
        pos.pose.position.y = y
        pos.pose.position.z = z
	pos.pose.orientation.x = xo
	pos.pose.orientation.y = yo
	pos.pose.orientation.z = zo
	pos.pose.orientation.w = wo	
	if(typ==0):
    		scene.add_box(name, pos, (d1,d2,d3))
	elif(typ==1):
		scene.add_sphere(name,pos,d1)
	else:
		print "ERROR in Type"
	return pos
"""
----------------Function Name:  pick_object----------------
Definition  : It moves the given group i.e robot to the collision object whose index is given and picks that object.
Group       : MoveGroupCommander Object (Motion Planning group)
Part_index  : Index of the target object to obtain pose
Robot       : Robot Commander Object
Scene       : Planning Scene Interface Object (current scene)
"""
def pick_object(group,part_index,robot,scene):

    pos = copy.deepcopy(target_poses[part_index])
    pos.pose.position.z += 0.12
    pos.pose.orientation.y = 1
    group.set_pose_target(pos)
    state =group.go()
    print "Picking execution for Part_"+str(part_index)+":" + str(state)
    if(state):
	#Without gripper version :
	robot.manipulator.attach_object("part"+str(part_index))
        #closed_state_gripper(robot,"part"+str(part_index))
	rospy.sleep(1.0)
	place_object(group,part_index,robot,scene)
	return
    else:
	print "Failed to execute picking trajectory"
	return
"""
----------------Function Name:  pick_object----------------
Definition  : It places the gripped object to the target location
Group       : MoveGroupCommander Object (Motion Planning group)
Part_index  : Index of the target object to obtain pose
Robot       : Robot Commander Object
Scene       : Planning Scene Interface Object (current scene)
"""    
def place_object(group,part_index,robot,scene):
	pos = PoseStamped()
	pos.header.frame_id = robot.get_planning_frame()
        pos.pose.position.x = -0.565859
        pos.pose.position.y = -0.379874
        pos.pose.position.z = 0.26
	pos.pose.orientation.y =1.0
	group.set_pose_target(pos)
	state = group.go()
	if(state):
		detached= group.detach_object("part"+str(part_index))
		rospy.sleep(1.5)
		if(detached):
			picked.append(part_index)
			scene.remove_world_object("part"+str(part_index))
	else:
		group.detach_object("part"+str(part_index))
		
"""
----------------Function Name:  create_environment----------------
Definition  : Creates the simulation environment with non-target collision objects
Robot       : Robot Commander Object
Scene       : Planning Scene Interface Object (current scene)
""" 	
def create_environment(scene,robot):	
    
    add_object(name="wall",x=0.0,y=0.8,z=0.5,robot=robot,scene=scene,d1=0.01,d2=0.35,d3=1,typ=0)
    add_object(name="wall_2",x=0.0,y=-0.8,z=0.5,robot=robot,scene=scene,d1=0.01,d2=0.35,d3=1,typ=0)
    add_object(name="table",x=0.0,y=0.0,z=-0.05,robot=robot,scene=scene,d1=2,d2=2,d3=0.0001,typ=0)
    #add_object(name="table2",x=0.0,y=0.0,z=0.8,robot=robot,scene=scene,d1=2,d2=2,d3=0.0001,typ=0)
     
if __name__=='__main__':
    #Init
    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_py_demo', anonymous=True)
    scene = PlanningSceneInterface()
    robot = RobotCommander()
    arm =  MoveGroupCommander("manipulator")
    #eef = MoveGroupCommander("endeffector")
    rospy.sleep(1)

    #Reset State of the Gripper 
    #default_state_gripper(eef) 
    #Reset the position of the Arm
    # Will be implemented if needed
    #Clean the scene
    clean_scene(scene)
    #Create environment
    create_environment(scene,robot)
    #Create target objects
    target_poses = check_targets(robot,scene,3,0.10)
    rospy.sleep(1)
    # Try 35 times to plan motion for each object -- to ensure it cannot be planned, number can be increased--
    for j in xrange(35):
	print  "Try : " +str(j+1)
	#Pick and place every object
    	for i in xrange(len(target_poses)):
            if i not in picked:
	    	pick_object(group=arm, part_index = i,robot = robot,scene=scene)
    if(j < 35):
        rospy.spin()
roscpp_shutdown()
