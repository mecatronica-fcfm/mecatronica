#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, division

# ROS
import rospy
from biped_interface.commander import BipedCommander
import math
 
def main():
  rospy.init_node('biped_commander_test')
  
  biped = BipedCommander()
  print(biped.joint_names)
  r = rospy.Rate(30)
  A = 0.3
  B = 0.2
  f = 2.0
  
  while not rospy.is_shutdown():
    
    t = rospy.get_rostime().to_sec()
    
    theta1 = math.sin(A*2*math.pi*f*t)
    theta2 = math.sin(B*2*math.pi*f*(t+0.3))

    alpha1 = math.sin(A*2*math.pi*f*t+math.pi/2)
    alpha2 = math.sin(B*2*math.pi*f*(t+0.3)+math.pi/2)

    biped.set('right_waist', theta1)
    biped.set('right_knee', theta2)

    biped.set('left_waist', alpha1)
    biped.set('left_knee', alpha2)

    r.sleep()

if __name__ == '__main__':
  main()

