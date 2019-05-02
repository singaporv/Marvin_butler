#!/usr/bin/env python
"""
Example for commanding joints
"""

import sys

import numpy as np
import math
import rospy
from sensor_msgs.msg import JointState
from dynamixel_workbench_msgs.msg import DynamixelStateList as DSL
import matplotlib.pyplot as plt

ROSTOPIC_SET_ARM_JOINT = '/goal_dynamixel_position'


def home_arm(pub):
    rospy.loginfo('Going to arm home pose')
    set_arm_joint(pub, np.zeros(5))
    rospy.sleep(5)


def set_arm_joint(pub, joint_target):
    joint_state = JointState()
    joint_state.position = tuple(joint_target)
    pub.publish(joint_state)


def interp(target_joints, num_steps):
    counter = 0
    target = np.array(target_joints)
    #rospy.loginfo(target)
    output_np = np.zeros(((target.shape[0]-1)*num_steps, target.shape[1]))
    for i in range(0,target.shape[0]-1):
        delta = (target[i+1,:]-target[i,:])/num_steps
        for j in range(0,num_steps):
            output_np[counter,:]=target[i,:]+delta*j
            counter+=1
    print(output_np)
    return output_np.tolist() 

position = []
velocity = []


def state_position_callback(msg):
    
    pos = []
    vel = []
    for i in range(0,8):
        pos.append(msg.dynamixel_state[i].present_position)
        vel.append(msg.dynamixel_state[i].present_velocity)
    position.append(pos)
    velocity.append(vel)

    #print(np.array(position).shape)






    print('base', msg.dynamixel_state[0].present_position)
    print('pos1', msg.dynamixel_state[1].present_position)
    print('pos2', msg.dynamixel_state[2].present_position)
    print('elbow', msg.dynamixel_state[3].present_position)
    print('wrist', msg.dynamixel_state[4].present_position)
    print('twisty', msg.dynamixel_state[5].present_position)
    




def main():
    rospy.init_node('control_arm', anonymous=True)

    rospy.Subscriber("dynamixel_state", DSL, state_position_callback)
    target_joints = [
    #[-1.1,-0.75,.7,-1.6,0], #dont need
	#[-1.1, 0, -0.1, -1.6, 0],
    [0, 0, -1.4, -0.2, 0],
    [0, 0, -1.4, -0.2, math.pi],
    [-1.1, 0,-0.1, -1.6, 0],
	#[-1.1, -0.75, .7, -1.6, 0]
    #[0,.6,-1,-1.4,0] #testing param
    #[0,0,-1.4,-0.2,0]#testing
    #[-1.1, 0, -0.1, -1.6, 0]
    ]
  
    #was .7

   

    #print(interp(target_joints,5))

    pub = rospy.Publisher(ROSTOPIC_SET_ARM_JOINT,
                          JointState, queue_size=1)
    rospy.sleep(2)
    #home_arm(pub)

    #for joint in interp(target_joints,20):
    for joint in target_joints:
      set_arm_joint(pub, joint)
      rospy.sleep(15)

    rospy.sleep(30)
    #home_arm(pub)
    print(np.array(position).shape)
    plt.subplot(2,1,1)
    plt.plot(np.array(position))
    plt.legend(['1','2','3','4','5'])
    plt.subplot(2,1,2)
    plt.plot(np.array(velocity))
    plt.show()



if __name__ == "__main__":
    main()
