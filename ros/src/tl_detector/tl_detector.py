#!/usr/bin/env python

import sys
import math
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
import numpy as np
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import csv

from light_classification.simple_detector import simple_detector, simple_detector_ROSdebug

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.waypoints_count = 0
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        # Hack light state to run tests independent of the classifier
        self.hack_light_state = False

        # Capture test data
        self.capture_test_data = False
        self.car_stop_line_gap_threshold = 120 # max gap between light and car
                                               # for the light to be captured for training
        self.img_dir = './data'
        self.img_base_name = 'img'
        self.img_counter = 1
        self.img_total_count_to_capture = 600
        self.img_csv_file_name = './data/img_test_data.csv'
        self.img_csv_touple = []

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints
        self.waypoints_count = len(self.waypoints)
        rospy.logwarn('Received base waypoints. Total base waypoints = ' + str(self.waypoints_count))

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        # capture test data 
        if self.capture_test_data and self.has_image and self.pose:
            '''
            Captures traffic lights which are within 'self.car_stop_line_gap_threshold' waypoints ahead of the car.
            Writes the image file name, light state (as received in the /vehicle/traffic_lights topic) and 
            the lights 3D coordinates to self.img_csv_file_name
            '''

            # closest waypoint to car position
            # closest waypoint to next light stop line
            # light_3D_position
            # state
            
            wp_closest_to_car_idx = self.get_closest_waypoint(self.pose.pose)
            next_stop_line_idx, next_stop_light_pose = self.get_next_stop_line()
            wp_closest_to_next_stop_line_idx = self.get_closest_waypoint(next_stop_light_pose.pose) 
            next_light_3D = self.lights[next_stop_line_idx]
            next_light_state = next_light_3D.state

            car_stop_line_gap = wp_closest_to_next_stop_line_idx - wp_closest_to_car_idx
           
            # capture image within the gap threshold
            if (self.img_counter <= self.img_total_count_to_capture) and car_stop_line_gap > -1 and car_stop_line_gap <= self.car_stop_line_gap_threshold:

                cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
                img_name = '%s_%s_%s.png' % (self.img_base_name, car_stop_line_gap, self.img_counter)
                rospy.logwarn('captured image ' + img_name)

                cv2.imwrite(self.img_dir + '/' + img_name, cv_image)
                self.img_counter += 1

                # csv touple
                self.img_csv_touple.append((img_name, next_light_state, next_light_3D.pose.pose.position.x, next_light_3D.pose.pose.position.y, next_light_3D.pose.pose.position.z))

            # csv_entry
            if self.img_counter == self.img_total_count_to_capture:
                rospy.logwarn('Test images captured.  Writing to csv file ' + self.img_csv_file_name)

                self.capture_test_data = False

                with open(self.img_csv_file_name, 'wb') as csvfile:
                    csv_writer = csv.writer(csvfile, delimiter=',')

                    for t in self.img_csv_touple:
                        csv_writer.writerow(t)

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_next_stop_line(self):
        """
        The closest stop line this method returns might be behind the car.
        We might need a better logic
        """

        stop_line_positions = self.config['stop_line_positions']
       
        min_dist = sys.maxint
        min_dist_idx = 0

        for i in range(0, len(stop_line_positions)):
            p1 = stop_line_positions[i]
            p2 = self.pose.pose.position
            dist = math.sqrt((p1[0]-p2.x)**2 + (p1[1]-p2.y)**2)
            if min_dist > dist:
                min_dist = dist
                min_dist_idx = i

        stop_line_xy = stop_line_positions[min_dist_idx]

        p = PoseStamped()
        p.pose.position.x = stop_line_xy[0]
        p.pose.position.y = stop_line_xy[1]

        return min_dist_idx, p

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement

        min_dist = sys.maxint
        min_dist_idx = 0

        for i in range(0, self.waypoints_count):
            dist = self.euclidean_distance_3D(self.waypoints[i].pose.pose.position, pose.position)
            if min_dist > dist:
                min_dist = dist
                min_dist_idx = i 

        return min_dist_idx

    def euclidean_distance_2D(self, p1, p2):
        return math.sqrt((p1.x-p2.x)**2 + (p1.y-p2.y)**2)

    def euclidean_distance_3D(self, p1, p2):
        return math.sqrt((p1.x-p2.x)**2 + (p1.y-p2.y)**2  + (p1.z-p2.z)**2)

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        raw_image = cv2.cvtColor(cv_image,cv2.COLOR_BGR2RGB)

        #Get classification
        return self.light_classifier.get_classification(raw_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        light = -1
        state = -1

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)

        #TODO find the closest visible traffic light (if one exists)

        wp_closest_to_car_idx = self.get_closest_waypoint(self.pose.pose)
        next_stop_line_idx, next_stop_light_pose = self.get_next_stop_line()
        wp_closest_to_next_stop_line_idx = self.get_closest_waypoint(next_stop_light_pose.pose) 
        next_light_3D = self.lights[next_stop_line_idx]
        next_light_state = next_light_3D.state

        car_stop_line_gap = wp_closest_to_next_stop_line_idx - wp_closest_to_car_idx
         
        # flag classifier if upcoming stop line is within the threshold
        if car_stop_line_gap > -1 and car_stop_line_gap <= self.car_stop_line_gap_threshold:
            light = next_stop_line_idx
            state = next_light_state

        if light > -1:
            
            # use hack to run tests independent of the classifier
            if self.hack_light_state:
                state = next_light_statie
            else:
                state = self.get_light_state(light)

                state_str = 'Unknown'

                if state == 0:
                    state_str = 'Red'
                elif state == 1:
                    state_str = 'Yellow'
                elif state == 2:
                    state_str = 'Green'

                rospy.logwarn('The state of the traffic light is: {}'.format(state_str))
            
            # waypoint closest to next stop line
            light_wp = wp_closest_to_next_stop_line_idx

            return light_wp, state

        #self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
