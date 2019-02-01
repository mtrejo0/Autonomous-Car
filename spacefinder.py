import time
import numpy as np
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from ar_track_alvar_msgs.msg import AlvarMarkers

class Wall_Follow():
    def __init__(self):
        # subscribe to Ackermann
        rospy.Subscriber("ackermann_cmd_mux/output",AckermannDriveStamped,self.ackermann_cmd_input_callback)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        
        self.cmd_pub = rospy.Publisher('/ackermann_cmd_mux/input/default', AckermannDriveStamped,queue_size = 10)

        self.Kp = 1
        self.Kd = .1
        self.Ki = .01
        self.prev_error = 0
        self.control = 0
        self.time = 0
        self.avg = 0
 	self.goal = .7
	self.front = 10
    
    

    def laser_callback(self,msg):
       
        ranges = msg.ranges[300:781]
       	area = 0
	for i in ranges:
		area+= i
	cumu = 0
	angle = 0
	for i in range(len(ranges)):
		cumu+=ranges[i]
		if(cumu > area/2):
			angle = i
			break
	print(angle)
	
	#then normalize this index to a steering angle GOAL
	#then do PID to make the steering angle equal to GOAL
	
	
	

    def ackermann_cmd_input_callback(self, msg):
        msg.drive.speed = 0.1
        msg.drive.steering_angle = -1
        msg.drive.steering_angle_velocity = .6
        self.cmd_pub.publish(msg)
                    
 
if __name__ == "__main__":
    rospy.init_node("Wall_Follow")
    node = Wall_Follow()
    rospy.spin()
