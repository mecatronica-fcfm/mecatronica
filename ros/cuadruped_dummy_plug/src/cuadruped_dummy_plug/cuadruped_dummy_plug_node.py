#!/usr/bin/env python

#
# @file callback_ir.cpp
#
# Callback from simsensor to publish dummy messages as the encoder.
#
# @author <a href="mailto:nicolas.marticorena.vidal@gmail.com">nmarticorena</a>
# @created 2014-11-1
# @modified 2017-5-10
# @version 1.1
#

import rospy
from std_msgs.msg import String
from Tkinter import *
from c_msg.msg import ServoArray



def callback_left_front_hip(value):
	msg_servo_array.left_front_hip_angle = int(value)
	pub_servo_array.publish(msg_servo_array)
	rospy.loginfo("Dummy: Left Front Hip: %d !",int(value))
	return

def callback_left_front_knee(value):
	msg_servo_array.left_front_knee_angle = int(value)
	pub_servo_array.publish(msg_servo_array)
	rospy.loginfo("Dummy: Left Front Knee: %d !",int(value))
	return

def callback_left_front_ankle(value):
	msg_servo_array.left_front_ankle_angle = int(value)
	pub_servo_array.publish(msg_servo_array)
	rospy.loginfo("Dummy: Left Front Ankle: %d !", int(value))
	return

def callback_right_front_hip(value):
	msg_servo_array.right_front_hip_angle = int(value)
	pub_servo_array.publish(msg_servo_array)
	rospy.loginfo("Dummy: Right Front Hip: %d !", int(value))
	return

def callback_right_front_knee(value):
	msg_servo_array.right_front_knee_angle = int(value)
	pub_servo_array.publish(msg_servo_array)
	rospy.loginfo("Dummy: Right Front Knee: %d !", int(value))
	return

def callback_right_front_ankle(value):
	msg_servo_array.right_ankle_angle = int(value)
	pub_servo_array.publish(msg_servo_array)
	rospy.loginfo("Dummy: Right Front Ankle: %d !", int(value))
	return

def callback_left_back_hip(value):
	msg_servo_array.left_back_hip_angle = int(value)
	pub_servo_array.publish(msg_servo_array)
	rospy.loginfo("Dummy: Left Back Hip: %d !",int(value))
	return

def callback_left_back_knee(value):
	msg_servo_array.left_back_knee_angle = int(value)
	pub_servo_array.publish(msg_servo_array)
	rospy.loginfo("Dummy: Left Back Knee: %d !",int(value))
	return

def callback_left_back_ankle(value):
	msg_servo_array.left_back_ankle_angle = int(value)
	pub_servo_array.publish(msg_servo_array)
	rospy.loginfo("Dummy: Left Back Ankle: %d !", int(value))
	return

def callback_right_back_hip(value):
	msg_servo_array.right_back_hip_angle = int(value)
	pub_servo_array.publish(msg_servo_array)
	rospy.loginfo("Dummy: Right Back Hip: %d !", int(value))
	return

def callback_right_back_knee(value):
	msg_servo_array.right_back_knee_angle = int(value)
	pub_servo_array.publish(msg_servo_array)
	rospy.loginfo("Dummy: Right Back Knee: %d !", int(value))
	return

def callback_right_back_ankle(value):
	msg_servo_array.right_back_ankle_angle = int(value)
	pub_servo_array.publish(msg_servo_array)
	rospy.loginfo("Dummy: Right Back Ankle: %d !", int(value))
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
msg_servo_array.left_front_hip_angle    = 128
msg_servo_array.left_front_knee_angle   = 128
msg_servo_array.left_front_ankle_angle  = 128
msg_servo_array.right_front_hip_angle   = 128
msg_servo_array.right_front_knee_angle  = 128
msg_servo_array.right_front_ankle_angle = 128
msg_servo_array.left_back_hip_angle    = 128
msg_servo_array.left_back_knee_angle   = 128
msg_servo_array.left_back_ankle_angle  = 128
msg_servo_array.right_back_hip_angle   = 128
msg_servo_array.right_back_knee_angle  = 128
msg_servo_array.right_back_ankle_angle = 128
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
frame_left_up =  LabelFrame(win, text='Left Front Foot', padx=5, pady=5)
frame_right_up = LabelFrame(win, text='Right Front Foot', padx=5, pady=5)
frame_left_down =  LabelFrame(win, text='Left Back Foot', padx=5, pady=5)
frame_right_down = LabelFrame(win, text='Right Back Foot', padx=5, pady=5)


frame_left.pack(side=LEFT, expand=True, padx=10, pady=10, fill=BOTH)
frame_right.pack(side=LEFT, expand=True, padx=10, pady=10, fill=BOTH)


#Add Labels

#Left Front Foot Labels
Label(win, text='Hip:').grid(in_=frame_left_up,row=0,column=0, sticky=W)
Label(win, text='Knee:').grid(in_=frame_left_up,row=1,column=0, sticky=W)
Label(win, text='Ankle:').grid(in_=frame_left_up,row=2,column=0, sticky=W)

#Right Front Foot  Labels
Label(win, text='Hip:').grid(in_=frame_right_up,row=0,column=0, sticky=W)
Label(win, text='Knee:').grid(in_=frame_right_up,row=1,column=0, sticky=W)
Label(win, text='Ankle:').grid(in_=frame_right_up,row=2,column=0, sticky=W)
Label(win, text='Enable:').grid(in_=frame_right_up,row=3,column=0, sticky=W)

#Right Back Foot  Labels
Label(win, text='Hip:').grid(in_=frame_right_down,row=0,column=0, sticky=W)
Label(win, text='Knee:').grid(in_=frame_right_down,row=1,column=0, sticky=W)
Label(win, text='Ankle:').grid(in_=frame_right_down,row=2,column=0, sticky=W)
Label(win, text='Enable:').grid(in_=frame_right_down,row=3,column=0, sticky=W)

#Left Back Foot Labels
Label(win, text='Hip:').grid(in_=frame_left_down,row=0,column=0, sticky=W)
Label(win, text='Knee:').grid(in_=frame_left_down,row=1,column=0, sticky=W)
Label(win, text='Ankle:').grid(in_=frame_left_down,row=2,column=0, sticky=W)

#Add Slide Bars

left_front_hip_bar = Scale(win, from_=10, to=170, orient=HORIZONTAL, command=callback_left_front_hip)
left_front_hip_bar.grid(in_=frame_left_up, row=0, column=100, sticky=E)
left_front_hip_bar.set(106)

left_front_knee_bar = Scale(win, from_=10, to=170, orient=HORIZONTAL, command=callback_left_front_knee)
left_front_knee_bar.grid(in_=frame_left_up, row=1, column=100, sticky=E)
left_front_knee_bar.set(112)

left_front_ankle_bar = Scale(win, from_=10, to=170, orient=HORIZONTAL, command=callback_left_front_ankle)
left_front_ankle_bar.grid(in_=frame_left_up, row=2, column=100, sticky=E)
left_front_ankle_bar.set(93)

right_front_hip_bar = Scale(win, from_=10, to=170, orient=HORIZONTAL, command=callback_right_front_hip)
right_front_hip_bar.grid(in_=frame_right_up, row=0, column=100, sticky=E)
right_front_hip_bar.set(97)

right_front_knee_bar = Scale(win, from_=10, to=170, orient=HORIZONTAL, command=callback_right_front_knee)
right_front_knee_bar.grid(in_=frame_right_up, row=1, column=100, sticky=E)
right_front_knee_bar.set(91)

right_front_ankle_bar = Scale(win, from_=10, to=170, orient=HORIZONTAL, command=callback_right_front_ankle)
right_front_ankle_bar.grid(in_=frame_right_up, row=2, column=100, sticky=E)
right_front_ankle_bar.set(97)


left_back_hip_bar = Scale(win, from_=10, to=170, orient=HORIZONTAL, command=callback_left_back_hip)
left_back_hip_bar.grid(in_=frame_left_down, row=0, column=100, sticky=E)
left_back_hip_bar.set(106)

left_back_knee_bar = Scale(win, from_=10, to=170, orient=HORIZONTAL, command=callback_left_back_knee)
left_back_knee_bar.grid(in_=frame_left_down, row=1, column=100, sticky=E)
left_back_knee_bar.set(112)

left_back_ankle_bar = Scale(win, from_=10, to=170, orient=HORIZONTAL, command=callback_left_back_ankle)
left_back_ankle_bar.grid(in_=frame_left_down, row=2, column=100, sticky=E)
left_back_ankle_bar.set(93)

right_back_hip_bar = Scale(win, from_=10, to=170, orient=HORIZONTAL, command=callback_right_back_hip)
right_back_hip_bar.grid(in_=frame_right_down, row=0, column=100, sticky=E)
right_back_hip_bar.set(97)

right_back_knee_bar = Scale(win, from_=10, to=170, orient=HORIZONTAL, command=callback_right_back_knee)
right_back_knee_bar.grid(in_=frame_right_down, row=1, column=100, sticky=E)
right_back_knee_bar.set(91)

right_back_ankle_bar = Scale(win, from_=10, to=170, orient=HORIZONTAL, command=callback_right_back_ankle)
right_back_ankle_bar.grid(in_=frame_right_down, row=2, column=100, sticky=E)
right_back_ankle_bar.set(97)

enable_bar = Scale(win, from_=0, to=1, orient=HORIZONTAL, command=callback_enable)
enable_bar.grid(in_=frame_right_up, row=3, column=100, sticky=E)
enable_bar.set(0)

win.mainloop()
