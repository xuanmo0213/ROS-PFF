#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('pff_diff_robot')
from math import sin, cos, pi, sqrt

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from tf.broadcaster import TransformBroadcaster
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16
from std_msgs.msg import Header

#---------------#
class DiffRobot:
#---------------#

    #-----------------#
    def __init__(self):
    #-----------------#
        rospy.init_node("diff_robot")
        self.nodename = rospy.get_name()
        rospy.loginfo("--INIT ROBOT: %s started --" % self.nodename)
        
        #-- parameters --#
	self.mode = rospy.get_param('mode',"keyboard")
	rospy.loginfo("--INIT ROBOT: %s mode selected --" % self.mode)
	self.size = rospy.get_param('size',1.0)
	rospy.loginfo("--INIT ROBOT: size %s selected --" % self.size)
        self.rate = rospy.get_param('rate',1.0)  # the rate for publishing the transform
	self.t_delta = rospy.Duration(1.0/self.rate)
        self.t_next = rospy.Time.now() + self.t_delta
        self.x = 0
        self.y = 0
        self.th = 0
	self.th_diff = 0
	self.angle = pi/16
        self.then = rospy.Time.now()
	self.count = 0
        self.orientation = Quaternion()

        # subscriptions

	rospy.Subscriber("/cmd_vel", Twist, self.cmdVelCallback)
        self.odomPub = rospy.Publisher("odom", Odometry, queue_size=10)
	self.joinPub = rospy.Publisher('joint_states', JointState, queue_size=10)
	self.markerPub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        self.odomBroadcaster = TransformBroadcaster()
        
    #--------------#
    def spin(self): 
    #--------------#
	#keep robot active
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()

    #--------------------------------#
    def seq(self,start, stop, step=1):
    #--------------------------------#
	#create coordinates for square mode
    	n = int(round((stop - start)/float(step)))
    	if n > 1:
        	return([start + step*i for i in range(n+1)])
    	else:
        	return([])
  
    #----------------#
    def update(self):
    #----------------#
	#update position and orientation
        now = rospy.Time.now()
        if now > self.t_next:
            
            self.then = now

            if self.mode == "circle":
		self.x = cos( self.th ) * self.size
        	self.y = sin( self.th ) * self.size
		self.th += self.angle
		if self.th >= 2 * pi:
			self.th = 0
		self.th_diff = self.angle
		
	    if self.mode == "square":

		x_label = self.seq(0, self.size, self.size/10) +\
			 [self.size]*8+self.seq(self.size, 0, 0-self.size/10) + [ 0 ]*8
		y_label = [ 0] *10+ self.seq(self.size/10, self.size, self.size/10) +\
			 [self.size]*8 + self.seq(self.size, self.size/10, (0 - self.size/10))
	        th_label = [ 0] * 10 + [ pi/2 ] * 9 + [0 ]* 10+ [pi/2] * 7

		if self.count >= 36:
		    self.count = 0
		self.x = x_label[self.count]
		self.y = y_label[self.count]
		self.th = th_label[self.count]
		self.count += 1
		
		
            self.odomBroadcaster.sendTransform(
                (self.x, self.y, 0),
                (0, 0, sin( self.th/2),cos( self.th/2)), 
                rospy.Time.now(),
                "base_link",
                "map"
                )
	    #----------------#
            odom = Odometry()

            odom.header.stamp = now
            odom.header.frame_id = "base_link"
	    odom.child_frame_id = "map"

            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = 0

            odom.pose.pose.orientation = [0, 0, sin( self.th/2),cos( self.th / 2 )] 

            odom.twist.twist.linear.x = 0
            odom.twist.twist.linear.y = 0
            odom.twist.twist.angular.z = 0

            self.odomPub.publish(odom)

	    #----------------#
	    join = JointState()

	    join.header.frame_id = "map"
	    join.header.stamp = now
	    join.name = ["joint_wheel_left","joint_wheel_right"]

	    if self.mode == "keyboard":
	        join.position = [sin( self.th_diff / 2 ) -0.005,sin( self.th_diff / 2 ) + 0.005]

	    elif self.mode == "circle":
		join.position = [-0.05,0.05]

	    elif self.mode == "square":
		join.position = [0, 0]

            self.joinPub.publish(join)

	    #----------------#
	    marker = Marker()

	    marker.type = Marker.POINTS
	    marker.header.stamp = now
	    marker.header.frame_id = "map"
	    marker.ns = "path"
	    marker.id = 0;

	    marker.pose.position.x = self.x
	    marker.pose.position.y = self.y
	    marker.pose.position.z = 0

	    marker.pose.orientation.x = 0.0
	    marker.pose.orientation.y = 0.0
	    marker.pose.orientation.z = sin( self.th / 2 )
	    marker.pose.orientation.w = cos( self.th / 2 )

	    marker.scale.x = 1;
	    marker.scale.y = 0.1
	    marker.scale.z = 0.1

	    marker.color.a = 1.0
	    marker.color.r = 0.0
	    marker.color.g = 1.0
	    marker.color.b = 0.0

	    self.markerPub.publish(marker)

            
    #----------------------------#
    def cmdVelCallback(self, msg):
    #----------------------------#
	#catch keyboard value and update
	if self.mode == "keyboard":
        	self.x += cos( self.th ) * msg.linear.x - sin( self.th ) * msg.linear.y
        	self.y += sin( self.th ) * msg.linear.x + cos( self.th ) * msg.linear.y
		self.th += msg.angular.z
		self.th_diff = msg.angular.z


    

#############################################################################

if __name__ == '__main__':
    """ main """
    diffTf = DiffRobot()
    diffTf.spin()
