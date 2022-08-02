#! /usr/bin/env python

import os
import fcntl
import subprocess
import psutil

import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
import sys, select, termios, tty

if __name__=="__main__":
	settings = termios.tcgetattr(sys.stdin)
	
	rospy.init_node('aruco_block_pose_node')
	pub_pose = rospy.Publisher('block_pose',Pose,queue_size=10)
	pub_grab = rospy.Publisher('grab',Bool,queue_size=10)
	rate = rospy.Rate(1)
	p = Pose()
	g = Bool()

	out = subprocess.PIPE
	err = subprocess.STDOUT
	status_proc = psutil.Popen(['rosrun','tf','tf_echo','/wlkata_base_link','/aruco_marker_frame'],stdout = out, stderr = err)
	file = status_proc.stdout
	fl = fcntl.fcntl(file.fileno(), fcntl.F_GETFL)
	fcntl.fcntl(file.fileno(),fcntl.F_SETFL, fl | os.O_NONBLOCK)
	
	while True:
		try:
			s = file.read()
		except:
			continue
		if not s:
			break
		
		index_Translation = s.find('Translation')
		if index_Translation == -1:			
			continue
		index_Translation = index_Translation +14
		
		index_Rotation = s.find('Rotation')
		if index_Rotation == -1:
			continue
		index_Rotation = index_Rotation + 25
	
		index_End = s.find('in RPY')
		if index_End == -1:
			continue
		
		Translation = s[index_Translation:index_Rotation - 29]
		Rotation = s[index_Rotation: index_End - 14]
		Translation = Translation.split(',')
		Rotation = Rotation.split(',')
		p.position.x = float(Translation[0]) + 0.05
		p.position.y = float(Translation[1]) + 0.015
		p.position.z = float(Translation[2])
		p.orientation.x = float(Rotation[0])
		p.orientation.y = float(Rotation[1])
		p.orientation.z = float(Rotation[2])
		p.orientation.w = float(Rotation[3])
		pub_pose.publish(p)
		rate.sleep()



