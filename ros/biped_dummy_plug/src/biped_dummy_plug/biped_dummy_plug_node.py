#!/usr/bin/env python

#
# @file callback_ir.cpp
#
# Callback from simsensor to publish dummy messages as the encoder.
#
# @author <a href="mailto:knzo@live.cl">Kenzo</a>
# @created 2014-11-1
# @modified 2014-11-1
# @version 1.0
#

import rospy
from std_msgs.msg import String
from Tkinter import *
from c_msg.msg import ServoArray



def callback_left_hip(value):
	msg_servo_array.left_hip_angle = int(value)
	pub_servo_array.publish(msg_servo_array)
	rospy.loginfo("Dummy: Left Hip: %d !",int(value))
	return

def callback_left_knee(value):
	msg_servo_array.left_knee_angle = int(value)
	pub_servo_array.publish(msg_servo_array)
	rospy.loginfo("Dummy: Left Knee: %d !",int(value))
	return

def callback_left_ankle(value):
	msg_servo_array.left_ankle_angle = int(value)
	pub_servo_array.publish(msg_servo_array)
	rospy.loginfo("Dummy: Left Ankle: %d !", int(value))
	return

def callback_right_hip(value):
	msg_servo_array.right_hip_angle = int(value)
	pub_servo_array.publish(msg_servo_array)
	rospy.loginfo("Dummy: Right Hip: %d !", int(value))
	return

def callback_right_knee(value):
	msg_servo_array.right_knee_angle = int(value)
	pub_servo_array.publish(msg_servo_array)
	rospy.loginfo("Dummy: Right Knee: %d !", int(value))
	return

def callback_right_ankle(value):
	msg_servo_array.right_ankle_angle = int(value)
	pub_servo_array.publish(msg_servo_array)
	rospy.loginfo("Dummy: Right Ankle: %d !", int(value))
	return

def callback_enable(value):
	value = int(value)
	if value==1:
		value = True
	else:
		value = False
	msg_servo_array.enable = value
	pub_servo_array.publish(msg_servo_array)
	rospy.loginfo("Enable: " + str(value))
	return

#Initialize as node
rospy.init_node('biped_dummy_plug_node', anonymous=True)

#Create dummy msg objects
msg_servo_array = ServoArray()
msg_servo_array.left_hip_angle    = 128
msg_servo_array.left_knee_angle   = 128
msg_servo_array.left_ankle_angle  = 128
msg_servo_array.right_hip_angle   = 128
msg_servo_array.right_knee_angle  = 128
msg_servo_array.right_ankle_angle = 128
msg_servo_array.enable = True

def quit(event):
    rospy.loginfo("Exit!")
    rospy.signal_shutdown("")
    root.quit()

#Create & Configure Publishers
pub_servo_array = rospy.Publisher('biped_dummy_plug_node/t_servo_array', ServoArray, queue_size=10)

#Create main window
root = Tk()
root.bind('<Control-c>', quit)
root.minsize() #width=500, height=500
root.wm_title("Biped - EVA-01 Dummy Plug")

#Create Main Frame Widget
win = Frame(root)
win.grid(sticky=N+S+E+W)

#Create Frames
frame_left =  LabelFrame(win, text='Left Foot', padx=5, pady=5)
frame_right = LabelFrame(win, text='Right Foot', padx=5, pady=5)

frame_left.pack(side=LEFT, expand=True, padx=10, pady=10, fill=BOTH)
frame_right.pack(side=LEFT, expand=True, padx=10, pady=10, fill=BOTH)


#Add Labels

#Left Foot Labels
Label(win, text='Hip:').grid(in_=frame_left,row=0,column=0, sticky=W)
Label(win, text='Knee:').grid(in_=frame_left,row=1,column=0, sticky=W)
Label(win, text='Ankle:').grid(in_=frame_left,row=2,column=0, sticky=W)

#Right Foot  Labels
Label(win, text='Hip:').grid(in_=frame_right,row=0,column=0, sticky=W)
Label(win, text='Knee:').grid(in_=frame_right,row=1,column=0, sticky=W)
Label(win, text='Ankle:').grid(in_=frame_right,row=2,column=0, sticky=W)
Label(win, text='Enable:').grid(in_=frame_right,row=3,column=0, sticky=W)

#Add Slide Bars

left_hip_bar = Scale(win, from_=10, to=170, orient=HORIZONTAL, command=callback_left_hip)
left_hip_bar.grid(in_=frame_left, row=0, column=100, sticky=E)
left_hip_bar.set(106)

left_knee_bar = Scale(win, from_=10, to=170, orient=HORIZONTAL, command=callback_left_knee)
left_knee_bar.grid(in_=frame_left, row=1, column=100, sticky=E)
left_knee_bar.set(112)

left_ankle_bar = Scale(win, from_=10, to=170, orient=HORIZONTAL, command=callback_left_ankle)
left_ankle_bar.grid(in_=frame_left, row=2, column=100, sticky=E)
left_ankle_bar.set(93)

right_hip_bar = Scale(win, from_=10, to=170, orient=HORIZONTAL, command=callback_right_hip)
right_hip_bar.grid(in_=frame_right, row=0, column=100, sticky=E)
right_hip_bar.set(97)

right_knee_bar = Scale(win, from_=10, to=170, orient=HORIZONTAL, command=callback_right_knee)
right_knee_bar.grid(in_=frame_right, row=1, column=100, sticky=E)
right_knee_bar.set(91)

right_ankle_bar = Scale(win, from_=10, to=170, orient=HORIZONTAL, command=callback_right_ankle)
right_ankle_bar.grid(in_=frame_right, row=2, column=100, sticky=E)
right_ankle_bar.set(97)

enable_bar = Scale(win, from_=0, to=1, orient=HORIZONTAL, command=callback_enable)
enable_bar.grid(in_=frame_right, row=3, column=100, sticky=E)
enable_bar.set(0)

win.mainloop()

