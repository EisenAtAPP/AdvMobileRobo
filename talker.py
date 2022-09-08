#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from std_msgs.msg import String
from turtlesim.msg import Pose
import math
from geometry_msgs.msg import Twist

x=0
y=0
theta=0

def poseCallback(pose_message):
    global x
    global y, theta
    offset = 5.544444561004639
    x = pose_message.x - offset
    y = pose_message.y - offset
    theta = pose_message.theta
    
def moveByDistance(distance, lx, ly,vel_pub):
    global x, y
    
    x0 = x
    y0 = y

    vel_msg = Twist() 
    vel_msg.linear.y = ly
    vel_msg.linear.x = lx
    vel_msg.angular.z = 0.0 

    distanceTravelled = math.sqrt(((x-x0) ** 2) + ((y-y0) ** 2))
    rate = rospy.Rate(10) # 10hz
    while distanceTravelled < distance:
        vel_pub.publish(vel_msg)
        rate.sleep()
        distanceTravelled = math.sqrt(((x-x0) ** 2) + ((y-y0) ** 2))
    
    vel_msg.linear.y = 0.0
    vel_msg.linear.x = 0.0
    vel_msg.angular.z = 0.0
    vel_pub.publish(vel_msg)
    rate.sleep()

def triangle(size,speed):
    vel_pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
    pos_sub = rospy.Subscriber('turtle1/pose', Pose, poseCallback)

    moveByDistance(size,speed,speed,vel_pub)
    moveByDistance(size/math.sqrt(2),0.0,-1*speed,vel_pub)
    moveByDistance(size/math.sqrt(2),-1*speed,0.0,vel_pub)
       
def circle(speed):
    pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    vel_msg = Twist() 
    vel_msg.linear.x = speed
    vel_msg.angular.z = speed 

    while not rospy.is_shutdown():
        pub.publish(vel_msg)
        
        rate.sleep()

def talker(path_type):
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    if path_type == 'circle':
        circle(2.0)
    elif path_type == 'triangle':
        triangle(2.0,2.0)
    

if __name__ == '__main__':
    try:
        talker('circle')
    except rospy.ROSInterruptException:
        pass
