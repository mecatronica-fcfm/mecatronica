#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function, division

__author__ = 'Rodrigo Muñoz'
__email__ = 'rorro.mr@gmail.com'

# ROS
import rospy
from sensor_msgs.msg import JointState
# Servo interface
# @TODO
import threading

 
class CommanderBase(object):
  """Base class for commander"""
  def __init__(self):
    # Joint states
    self._pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    self._joint_states = JointState()


class BipedCommander(CommanderBase):
  def __init__(self):
    super(BipedCommander,self).__init__()
    self.joint_names = ['right_waist', 'right_knee', 'right_foot_angle', 'left_waist', 'left_knee', 'left_foot_angle']
    self.dof = len(self.joint_names)
    self._joint_states.name = self.joint_names
    self._joint_states.position = [0.0] * self.dof
    threading.Thread(target=self._publish_state).start()

  def set(self, joint_name = 'right_waist', value = 0):
    #rospy.loginfo('BipedCommander set {0}'.format(joint_name))
    idx = self.joint_names.index(joint_name)
    self._joint_states.position[idx] = value

  def _publish_state(self):
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
      self._joint_states.header.stamp = rospy.Time.now()
      self._pub.publish(self._joint_states)
      r.sleep()

def main():
  import math
  rospy.init_node('biped_commander_test')
  
  biped = BipedCommander()
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

