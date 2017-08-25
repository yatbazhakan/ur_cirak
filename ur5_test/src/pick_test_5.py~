#!/usr/bin/env python
import sys
import rospy
import random
from moveit_commander import RobotCommander,MoveGroupCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped
joint_names =  ['finger_joint', 'left_inner_knuckle_joint', 'left_inner_finger_joint', 'right_outer_knuckle_joint', 'right_inner_knuckle_joint', 'right_inner_finger_joint', 'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
positions =  [0.2928262968941126, -0.2928262968941126, 0.2928262968941126, -0.2928262968941126, -0.2928262968941126, 0.2928262968941126, 0.44286313484323736, -1.1781581860491328, 1.7121511388933512, 1.0374444266562388, 1.5704395720709645, -2.698212672994671]
#def default_grip(arm):
    

if __name__=='__main__':

    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_py_demo', anonymous=True)
    
    scene = PlanningSceneInterface()
    robot = RobotCommander()
    arm =  MoveGroupCommander("manipulator")
    eef = MoveGroupCommander("endeffector")
    rospy.sleep(1)
    #Start State Gripper 
    group_variable_values = eef.get_current_joint_values()
    group_variable_values[0] = 0.0
    eef.set_joint_value_target(group_variable_values)
    plan2 = eef.plan()
    arm.execute(plan2)  
    # clean the scene
    scene.remove_world_object("pole")
    scene.remove_world_object("table")
    scene.remove_world_object("part")

    # publish a demo scene
    k = PoseStamped()
    k.header.frame_id = robot.get_planning_frame()
    k.pose.position.x = 0.0
    k.pose.position.y = 0.0
    k.pose.position.z = -0.05
    scene.add_box("table", k, (2,2, 0.0001))
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 0.35
    p.pose.position.y = 0.35		
    p.pose.position.z = 0.05
    scene.add_box("part", p, (0.09, 0.09, 0.09))

    rospy.sleep(1)
#Planning Part 1: Move to correct xy coorinate
    temPose = p
    print temPose
    temPose.pose.position.z += 0.20
    temPose.pose.orientation.y = 1
    arm.set_pose_target(temPose)
    pl = arm.plan()
    state =arm.execute(pl)
    print state
    if(state):
        group_variable_values = eef.get_current_joint_values()
        group_variable_values[0] = 0.185
        eef.set_joint_value_target(group_variable_values)
        plan2 = eef.plan()
        eef.execute(plan2)

        #Planning Part 3: Grip
        x= eef.attach_object("part","robotiq_arg2f_base_link")
        if(x):
            place_pose = PoseStamped()
            place_pose.header.frame_id = robot.get_planning_frame()
            place_pose.pose.position.x = 0.324157
            place_pose.pose.position.y = 0.354343
            place_pose.pose.position.z = 0.575525  
            place_pose.pose.orientation.x = 0.00014 
            place_pose.pose.orientation.y = 0.884293 
            place_pose.pose.orientation.z = 0.466933
     
            arm.set_pose_target(place_pose)
            pl = arm.plan()
            state =arm.execute(pl)
            if(state):
                place_pose = PoseStamped()
                place_pose.header.frame_id = robot.get_planning_frame()
                place_pose.pose.position.x = -0.5
                place_pose.pose.position.y = -0.6
                place_pose.pose.position.z = 0.35
                place_pose.pose.orientation.y = 1
                arm.set_pose_target(place_pose)
                pl = arm.plan()
                state =arm.execute(pl)
                if(state):
                    group_variable_values = eef.get_current_joint_values()
                    group_variable_values[0] = 0.0
                    eef.detach_object("part")
                    place_pose = PoseStamped()
                    place_pose.header.frame_id = robot.get_planning_frame()
                    place_pose.pose.position.x = -0.495562
                    place_pose.pose.position.y = -0.607059
                    place_pose.pose.position.z = 0.417763
                    place_pose.pose.orientation.y = 1
                    arm.set_pose_target(place_pose)
                    pl = arm.plan()
                    state =arm.execute(pl)
                    if(state):
                        place_pose = PoseStamped()
                        place_pose.header.frame_id = robot.get_planning_frame()
                        place_pose.pose.position.x = -0.5
                        place_pose.pose.position.y = -0.6
                        place_pose.pose.position.z = 0.575525
                        place_pose.pose.orientation.y = 1.0
                        arm.set_pose_target(place_pose)
                        pl = arm.plan()
                        state =arm.execute(pl)
                   
                     
                
            
    
        #robot.manipulator.pick('part')

    rospy.spin()
roscpp_shutdown()
