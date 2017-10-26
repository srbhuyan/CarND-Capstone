#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane
from std_msgs.msg import Int32
import numpy as np
import sys
import math
import numpy as np
import tf

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
MAX_SPEED = 10 # The actrual speed is 2x MAX_SPEED. I don't know why. Please seet this value < 10 or the controller cannot follow and the I controller would not work properly..

DEBUG = True

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.twist_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.waypoints = None
        self.waypoints_count = 0
        self.curr_pose = None
        self.curr_twist = None
        self.position = None
        self.orientation = None
        self.theta = None

        self.stop_idx = -1

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement

        self.curr_pose = msg.pose
        self.position = self.curr_pose.position
        self.orientation = self.curr_pose.orientation
        # transform Quaterion coordinate to (roll, pitch and yaw)
        euler = tf.transformations.euler_from_quaternion([
            self.orientation.x,
            self.orientation.y,
            self.orientation.z,
            self.orientation.w])
        self.theta = euler[2]   # steering angle

        final_wps = self.generate_final_waypoints()
        # publish final waypoints to /final_waypoints topic
        self.final_waypoints_pub.publish(final_wps)
    
    def twist_cb(self, msg):
        # TODO: Implement
        self.curr_twist = msg

    def generate_final_waypoints(self):
        select_wps = []

        start_idx = -1
        end_idx = -1

        # waypoints from base waypoints
        if self.waypoints:
            # waypoint closest to car
            start_idx = self.get_closest_waypoint(self.curr_pose)

            # slice LOOKAHEAD_WPS number of waypoints from base waypoints
            end_idx = start_idx + LOOKAHEAD_WPS
            if end_idx > self.waypoints_count:
                select_wps = self.waypoints[start_idx:] + self.waypoints[0:end_idx % self.waypoints_count]
            else:
                select_wps = self.waypoints[start_idx:end_idx]

                # rospy.logwarn('final waypoints range: (' + str(start_idx) + ',' + str(end_idx % self.waypoints_count) + ')')

        final_wps = Lane()
        final_wps.waypoints = select_wps

        # if upcoming red light -> Decelerate (to full stop)
        # else                  -> Accelerate (to max velocity)

        # Deceleration
        if self.stop_idx != -1:

            for i in range(len(select_wps)):
                self.set_waypoint_velocity(select_wps, i, 0.0)

            stop_idx_in_select_wps = self.stop_idx - start_idx
            # Using sudden stop mechnism. It is just a step responce and performs quite well

#            for i in range(stop_idx_in_select_wps-np.int(MAX_SPEED*1.5)):
#                self.set_waypoint_velocity(select_wps, i, MAX_SPEED)
            
            for i in range(stop_idx_in_select_wps-np.int(MAX_SPEED*0.5)):
                self.set_waypoint_velocity(select_wps, i, MAX_SPEED/10)
            
            for i in range(stop_idx_in_select_wps-np.int(MAX_SPEED*1.0)):
                self.set_waypoint_velocity(select_wps, i, MAX_SPEED/2)
            
            for i in range(stop_idx_in_select_wps-np.int(MAX_SPEED*3.0)):
                self.set_waypoint_velocity(select_wps, i, MAX_SPEED)
        

        # Acceleration
        else:
            # Acceleration should be gradual. Although throttle=1.0 works OK in the simulator
            # it would be rough for Carla
            # We can use increments in steps as used for deceleration
            for i in range(len(select_wps)):

                self.set_waypoint_velocity(select_wps, i, MAX_SPEED) # This should be smaller than 10

        if DEBUG:
            v = []
            for w in select_wps:
                v.append(w.twist.twist.linear.x)

            rospy.logwarn('Velocities of selected waypoints:')
            rospy.logwarn('{}'.format(v))

        return final_wps

    def euclidean_distance_3D(self, p1, p2):
        return math.sqrt((p1.x-p2.x)**2 + (p1.y-p2.y)**2  + (p1.z-p2.z)**2)

    def get_closest_waypoint(self, pose):

        min_dist = sys.maxint
        min_dist_idx = 0

        for i in range(0, self.waypoints_count):
            dist = self.euclidean_distance_3D(self.waypoints[i].pose.pose.position, pose.position)
            if min_dist > dist:
                min_dist = dist
                min_dist_idx = i

        x = self.waypoints[min_dist_idx].pose.pose.position.x
        y = self.waypoints[min_dist_idx].pose.pose.position.y
        heading = np.arctan2((y-pose.position.y), (x-pose.position.x))
        angle = np.abs(heading - self.theta)
        if angle > np.pi/4:
            min_dist_idx += 1
            if min_dist_idx > len(self.waypoints):
                min_dist_idx = 0
        return min_dist_idx

    def euclidean_distance(self, p1, p2):
        # Can we ignore z ?
        return math.sqrt((p1.x-p2.x)**2 + (p1.y-p2.y)**2  + (p1.z-p2.z)**2)

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.waypoints = waypoints.waypoints
        self.waypoints_count = len(self.waypoints)
        rospy.logwarn('Received base waypoints. Total base waypoints = ' + str(self.waypoints_count))

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement

        self.stop_idx = msg.data

        #rospy.logwarn('waypoint_updater: Traffic stop line waypoint index = {}'.format(msg.data) )

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
