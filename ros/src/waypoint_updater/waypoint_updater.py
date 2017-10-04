#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import sys
import math

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


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_wps = None
        self.base_wps_count = 0
        self.curr_pose = None

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        self.curr_pose = msg

        final_wps = self.generate_final_waypoints()

        # publish final waypoints to /final_waypoints topic
        self.final_waypoints_pub.publish(final_wps)
    
    def generate_final_waypoints(self):
        select_wps = []

        # waypoints from base waypoints
        if self.base_wps:
            # waypoint closest to car
            start_idx = self.closest_wp(self.base_wps, self.curr_pose)

            # slice LOOKAHEAD_WPS number of waypoints from base waypoints 
            end_idx = start_idx + LOOKAHEAD_WPS
            if end_idx > self.base_wps_count:
                select_wps = self.base_wps[start_idx:] + self.base_wps[0:end_idx % self.base_wps_count]
            else:
                select_wps = self.base_wps[start_idx:end_idx]

            #rospy.logwarn('final waypoints range: (' + str(start_idx) + ',' + str(end_idx % self.base_wps_count) + ')')

        final_wps = Lane()
        final_wps.waypoints = select_wps

        # velocities are not set for the final waypoints
        # need to set velocities depending on obstacles

        return final_wps
     
    def closest_wp(self, waypoints, p):
        # This closest point returned might not be infront of the car
        # Is that problematic?

        min_dist = sys.maxint
        min_dist_idx = 0

        for i in range(0, self.base_wps_count):
            dist = self.euclidean_distance(self.base_wps[i].pose.pose.position, p.pose.position)
            if min_dist > dist:
                min_dist = dist
                min_dist_idx = i

        return min_dist_idx

    def euclidean_distance(self, p1, p2):
        # Can we ignore z ?
        return math.sqrt((p1.x-p2.x)**2 + (p1.y-p2.y)**2  + (p1.z-p2.z)**2)

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_wps = waypoints.waypoints
        self.base_wps_count = len(self.base_wps)
        rospy.logwarn('Base waypoints count = %d' % (self.base_wps_count))

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

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
