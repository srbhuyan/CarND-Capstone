#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

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
        self.curr_pose = None

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement

        final_wps = self.generate_final_waypoints(msg)

        # publish final waypoints to /final_waypoints topic
        self.final_waypoints_pub.publish(final_wps)
    
    def generate_final_waypoints(self, curr_pose):
        gen_wps = []

        # waypoints from base waypoints
        if(self.base_wps is None):
            rospy.logerr('ganerate_final_waypoints : No base waypoints received yet!!!')
        else:
            # first waypoint from base waypoints
            start_index = 0
            for i in range(0, len(self.base_wps)-1):
                w = self.base_wps[i]
                if w.pose.pose.position.x > curr_pose.pose.position.x:
                    #rospy.logwarn('Found first waypoint ahead of car')
                    #rospy.logwarn(w.pose.pose.position)
                    start_index = i
                    break

            # LOOKAHEAD_WPS number of waypoints in front of the car
            for i in range(start_index, start_index + LOOKAHEAD_WPS):
                gen_wps.append(self.base_wps[i])

        final_wps = Lane()
        final_wps.waypoints = gen_wps

        return final_wps
         
    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_wps = waypoints.waypoints
        rospy.logwarn('base waypoints size = ' + str(len(self.base_wps)))

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
