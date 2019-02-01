import time
import numpy as np
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from ar_track_alvar_msgs.msg import AlvarMarkers

mode = ''
class Wall_Follow():
    def __init__(self):
        # subscribe to Ackermann
        rospy.Subscriber("ackermann_cmd_mux/output",AckermannDriveStamped,self.ackermann_cmd_input_callback)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        rospy.Subscriber('ar_pose_marker', AlvarMarkers, self.ar_callback, queue_size = 1)
        self.cmd_pub = rospy.Publisher('/ackermann_cmd_mux/input/default', AckermannDriveStamped,queue_size = 10)

        self.Kp = 1
        self.Kd = .1
        self.Ki = .01
        self.prev_error = 0
        self.control = 0
        self.time = 0
        self.avg = 0
        self.goal = 0.7
        self.mode = mode
        self.front = 10
    
    

    def laser_callback(self,msg):
        #Reads ranges
        ranges = msg.ranges
       
	#depending on task use certain values
	if(mode == 'right'):
	    self.avg = np.mean(ranges[100 :250 ])
        elif(mode == 'left'):
	    self.avg = np.mean(ranges[956 :981 ])
	self.front = np.mean(ranges[len(ranges)//2-50 :len(ranges)//2+50])
	
	#update time
	self.time+=1
	
	#calculate error
	error = self.goal- self.avg
	
	#PID

	#P
	proportional_gain = self.Kp*error
	#D
        derivative_gain = self.Kd * (error - self.prev_error) * self.time
	#I
        integral_gain = self.Ki * error /self.time
        

        #update control value
        self.control = proportional_gain+ derivative_gain+ integral_gain

	
	#reflect if left instead of right
	if(mode == 'left'):
	    self.control*=-1

	#save previous values
        self.prev_error = error
	
        
    def ar_callback(self, ar_markers):
        pass


    def ackermann_cmd_input_callback(self, msg):
	#speed
        msg.drive.speed = 0.4
        msg.drive.steering_angle = self.control
        msg.drive.steering_angle_velocity = .6
        self.cmd_pub.publish(msg)
                    
 
if __name__ == "__main__":
    print('Start')
    print('What mode would you like? (left,right)')
    mode = input()
    rospy.init_node("Wall_Follow")
    node = Wall_Follow()
    rospy.spin()
