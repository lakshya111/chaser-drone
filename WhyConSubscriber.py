#!/usr/bin/env python
import time
import rospy
import geometry_msgs.msg
from std_msgs.msg import String

class PosePrint:

    def GetData(self,data):
	self.data = data

    def __init__(self):
	rospy.init_node('A')
	self.data = None
       	rospy.Subscriber("/whycon/poses",geometry_msgs.msg.PoseArray,self.GetData)

    def PrintPosition(self):
	print(time.time())
	for x in range(0,5):
		print("Marker %d - %0.15f %0.15f %0.15f" % (x+1,self.data.poses[x].position.x,self.data.poses[x].position.y,self.data.poses[x].position.z))	

	
if __name__ == '__main__':
	track = PosePrint()
	rospy.sleep(1)
	r=rospy.Rate(10)
	while not rospy.is_shutdown():
		track.PrintPosition()
		r.sleep()
	rospy.spin()

