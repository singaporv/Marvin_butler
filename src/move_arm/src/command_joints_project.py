#!/usr/bin/env python

import sys
import numpy as np
import rospy
import math
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from dynamixel_workbench_msgs.msg import DynamixelStateList as DSL

#need josh's message type here

ROSTOPIC_SET_ARM_JOINT = '/goal_dynamixel_position'

rospy.init_node("control_arm_node")

drink_angle = 0
#set up publisher
global drink_delivered_pub
drink_delivered_pub = rospy.Publisher('drink_delivered', Bool, queue_size=1)
drink_delivered = Bool()

pub = rospy.Publisher(ROSTOPIC_SET_ARM_JOINT,JointState, queue_size=1)

#set global variable so know when arm done lifting
global arm_lifted 
arm_lifted = 0
global twisty_prev 
twisty_prev= 0
global lifted_position
lifted_position = 0
global changing_position
changing_position = 0


def home_arm(pub):
    rospy.loginfo('Going to arm home pose')
    set_arm_joint(pub, np.zeros(5))
    rospy.sleep(5)


def set_arm_joint(pub, joint_target):
    joint_state = JointState()
    joint_state.position = tuple(joint_target)
    pub.publish(joint_state)


def goal_achieved_callback(msg):
    #if goal achieved lift arm and rotate to specified drink number
    drink_num = msg.data
    print('hellloooooo')


    ''' for drink 2 drink angle is 0
    	for drink 1 drink angle is -math.pi/3
    	for drink 6 drink angle is -2*math.pi/3
    	for drink 5 drink angle is 3*math,pi/3
    	for drink 3 drink angle is math.pi/3
    	for drink 4 drink angle is 2*math.pi/3'''

    if (drink_num ==1):
    	goal = 2724
    elif (drink_num ==2):
    	goal = 2042
    elif (drink_num ==0):
	goal = 2042
    elif (drink_num ==3):
    	goal = 1368
    elif (drink_num ==4):
    	goal = 685
    elif (drink_num ==5):
    	goal = 4

    if (drink_num == 6):
    	drink_angle = -2*math.pi/3
    	goal = 3406
    elif (drink_num ==0):
	drink_angle = 0
    else:
    	drink_angle = (drink_num-2)*math.pi/3

    #lift arm
    target_joints = [
	#[-1.1, 0, -0.1, -1.6, 0],
    #[0,    0, -1.4, -0.2, 0],
    [0,    0, -1.4, -0.2, drink_angle]
    ]

    for joint in target_joints:
    	set_arm_joint(pub, joint)
    	rospy.sleep(5)

    global lifted_position
    global changing_position
    lifted_position = changing_position
    print('lifted ',lifted_position )

    while (abs(goal-lifted_position)>10):
    	lifted_position = changing_position
    	print('lifted ',lifted_position )
    
    global arm_lifted
    arm_lifted = 1
    	#lifted_position = changing_position


def state_position_callback(msg):
	twisty = msg.dynamixel_state[5].present_position
	global changing_position
	changing_position = twisty
	print ('change ',changing_position)
	thresh = 1
	global arm_lifted
	global twisty_prev
	if (arm_lifted == 1):
		print('arm lifted')
		if (abs(lifted_position-twisty)>thresh):
    		#drink has been taken
			print('arm going down')
    		#lower arm
			target_joints = [
			#[0,    0, -1.4, -0.2, 0],
			[-1.1, 0,-0.1, -1.6, 0]
			#[-1.1, -0.75, .7, -1.6, 0]
			]
			for joint in target_joints:
				set_arm_joint(pub, joint)
				rospy.sleep(8)

			#publish to josh to look for next spot
			global drink_delivered
			arm_lifted =0
			drink_delivered.data = True
			drink_delivered_pub.publish(drink_delivered)

	print(twisty)
	print(lifted_position)
	#print(twisty)
	#twisty_prev = twisty
    

    
	'''pos = []
 	vel = []
    for i in range(0,8):
        pos.append(msg.dynamixel_state[i].present_position)
        vel.append(msg.dynamixel_state[i].present_velocity)
    position.append(pos)
    velocity.append(vel)

    #pos2 = msg.dynamixel_state[2].present_position
    #pos3 = msg.dynamixel_state[3].present_position
    #pos4 = msg.dynamixel_state[4].present_position





    print('pos 2', msg.dynamixel_state[2].present_position)
    print('pos 3', msg.dynamixel_state[3].present_position)
    print('pos 4', msg.dynamixel_state[4].present_position)
    print('pos 5', msg.dynamixel_state[5].present_position)'''


    


'''def drink_taken_callback(msg):
    #if goal achieved lift arm and rotate to specified drink number
    if msg = True:
        #fully extended, change last number to rotation corresponding to drink number
        target_joints = [0, 0, 0, -1.6, 0] #while wheeling around position
        set_arm_joint(pub, target_joint) #need to define publisher'''





if __name__ == "__main__":
    rospy.init_node("control_arm_node")
    #set arm
    rospy.sleep(10)
    target_joints = [[-1.1, 0,-0.1, -1.6, 0]]
    for joint in target_joints:
   	 set_arm_joint(pub, joint)
         rospy.sleep(8)

    #http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
    #I don't know what this does yet and if this is correct
    #rospy.init_node('get april tag goal from josh', anonymous = True)
    #rospy.init_node('get achieved april tag from josh', anonymous = True)

    #subscribe to Josh's topics
    rospy.Subscriber("drink_number",Int32, goal_achieved_callback, queue_size = 1)
    rospy.Subscriber("dynamixel_state", DSL, state_position_callback, queue_size = 1)
    

    while not rospy.is_shutdown():
    	rospy.spin()

