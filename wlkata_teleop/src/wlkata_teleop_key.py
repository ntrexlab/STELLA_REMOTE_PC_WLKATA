#! /usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
import sys, select, termios, tty
import time

msg = """
Control Your WLKATA!
--------------------------------------
Wait until WLKATA Homing finished, 
and control the WLKATA!

Moving around:
        w         e
   a         d         h
        x         c         ./ 

w/x : increase/decrease X postion
a/d : increase/decrease Y position
e/c : increase/decrease Z position(height)

h   : Homing

.,/ : open/close the gripper
 
CTRL-C to quit
"""

e = """
Error!!!
"""


def get_key():
	tty.setraw(sys.stdin.fileno())
	rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
	if rlist:
		key = sys.stdin.read(1)
	else:
		key = ''

	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key



if __name__=="__main__":
	settings = termios.tcgetattr(sys.stdin)

	rospy.init_node('pose_node')
	pub_pose = rospy.Publisher('pose',Pose,queue_size=10)
	pub_gripper = rospy.Publisher('gripper',Bool,queue_size=10)
	pub_homing = rospy.Publisher('homing',Bool,queue_size=10)

	rate = rospy.Rate(10)
	p = Pose()
	p.position.x = 198.6
	p.position.y = 0.0
	p.position.z = 230.4
	g = Bool()
	h = Bool()

	try:
		rospy.loginfo(msg)
		time.sleep(1)
		h.data = True
		pub_homing.publish(h)		
		rate.sleep()
		while(1):
			key = get_key()
			if key == 'w' :
				p.position.x = p.position.x + 5.0
				pub_pose.publish(p)
			elif key == "x" :
				p.position.x = p.position.x - 5.0
				pub_pose.publish(p)
			elif key == "a" :
				p.position.y = p.position.y + 5.0
				pub_pose.publish(p)
			elif key == "d" :
				p.position.y = p.position.y - 5.0
				pub_pose.publish(p)
			elif key == "e" :
				p.position.z = p.position.z + 5.0
				pub_pose.publish(p)
			elif key == "c" :
				p.position.z = p.position.z - 5.0
				pub_pose.publish(p)
			elif key == "h" :
				h.data = True
				pub_homing.publish(h)
			elif key == '.':
				g.data = True
				pub_gripper.publish(g)
			elif key == '/':
				g.data = False
				pub_gripper.publish(g)

			else:
				if key == '\x03':
					break
		
			rate.sleep()
	except:
		rospy.loginfo(msg)
