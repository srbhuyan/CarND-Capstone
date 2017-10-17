#!/usr/bin/env python

import sys
import math
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import csv

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None # self.pose<-(sub1,self.pose_cb)/current_pose
        self.waypoints = None # self.waypoints<-(sub2,self.waypoints_cb)/base_waypoints
        self.waypoints_count = 0
        self.camera_image = None # self.camera_image<-(sub3,self.traffic_cb)/vehicle/traffic_lights
        self.lights = [] # # self.lights<-(sub4,self.image_cb)/image_color

        # Add subscribers to four useful topics
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

        # Load the position of traffic light stop line and the size of the captured image
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        # Add a publisher to the topic indicating the index of the way point of the stop line 
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge() # used to converge the image msg to cv::Mat
        self.light_classifier = TLClassifier() # classifier for detecting the color of the traffic light
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN # the color of the traffic light
        self.last_state = TrafficLight.UNKNOWN # the color of the previous traffic light
        self.last_wp = -1 # previous way point
        self.state_count = 0

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
    	'''
    	Get the pose of the car
    	'''
        self.pose = msg

    def waypoints_cb(self, waypoints):
    	'''
    	Get all waypoints and the length of the waypoints
    	'''
        self.waypoints = waypoints.waypoints
        self.waypoints_count = len(self.waypoints)
        rospy.logwarn('Received base waypoints. Total base waypoints = ' + str(self.waypoints_count))

    def traffic_cb(self, msg):
    	'''
    	Get info. regarding all traffic lights (3D position and state)
    	'''
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
            
            wp_closest_to_car_idx = self.get_closest_waypoint(self.pose.pose.position)
            next_stop_line_idx, next_stop_light_pose = self.get_next_stop_line()
            wp_closest_to_next_stop_line_idx = self.get_closest_waypoint(next_stop_light_pose.pose.position) 
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
        We might need a better logic (comments by Zhi: I think we should use way point 
        index rather than real distance.)
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

    def get_closest_waypoint(self, position):
        """Identifies the closest path waypoint in 'self.waypoints' to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        min_dist = sys.maxint # the maximum number that can be represented by in in this system
        min_dist_idx = 0 # the index in self.waypoints that points to the minimum distance

        for i in range(0, self.waypoints_count):
            dist = self.euclidean_distance_3D(self.waypoints[i].pose.pose.position, self.pose.position)
            if min_dist > dist:
                min_dist = dist
                min_dist_idx = i 

        return min_dist_idx

    def project_to_image_plane(self, point_in_world):
        # This function has problems!
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """

        fx = self.config['camera_info']['focal_length_x']
        fy = self.config['camera_info']['focal_length_y']
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']
        cx = image_width/2
        cy = image_height/2

        # get transform between pose of camera and world frame
        trans = None
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link",
                  "/world", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link",
                  "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")

        #TODO Use tranform and rotation to calculate 2D position of light in image
        # xZ = fx*X + cx*Z;
        # yZ = fy*Y + cy*Z;
        # self.lights[i].pose.pose.position(and orientation)
        # self.pose.pose.position(and orientation)
        
        # NO CAMERA POSE INFOMATION IS PROVIDED. THIS FUNCTION CANNOT WORK CORRECTLLY!
        position_camera = self.pose.pose.position
        orientation_camera = self.pose.pose.orientation
        # orientation = [(x,y,x)*sin(theta/2),cos(theta/2)]
        # theta = 2*math.acos(orientation_camera[3])
        sin_theta = orientation_camera[2]
        cos_theta = orientation_camera[3]
        dX = point_in_world[0] - position_camera[0]
        dY = point_in_world[1] - position_camera[1]
        dZ = point_in_world[2] - position_camera[2]
        X_camera_coord = dX*cos_theta + dY*sin_theta
        Y_camera_coord = dZ
        Z_camera_coord = -(-dX*sin_theta + dY*cos_theta)
        
        x=(fx*X_camera_coord + cx*Z_camera_coord)/Z_camera_coord
        y=-(fy*Y_camera_coord + cy*Z_camera_coord)/Z_camera_coord
        return (x, y)
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

        # We should get the 3D position of the traffic light
        # The following function cannot work properly, since no camera pose info. is provided!
        # x, y = self.project_to_image_plane(light.pose.pose.position)

        #TODO use light location to zoom in on traffic light in image
        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # Light color detection
        # self.camera_image = sensor_msgs/Image
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        # config.yaml->self.config;(camera_info,stop_line_positions)
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose): 
            # Get index of the closest way point to the car
            # PoseStamped: (header;pose(position;orientation))  ;'self.pose = msg'
            car_way_point_index = self.get_closest_waypoint(self.pose.pose.position) 

        #TODO find the closest visible stop line (if one exists)
        # First, find out the nearest waypoints to each stop line.
        stop_line_positions = self.config['stop_line_positions']
        line_way_point_indices = []
        for i in range(0, len(stop_line_positions)):
            line_way_point_indices.append(self.get_closest_waypoint(stop_line_positions[i]))
            
        # Second, find out the index of the way point of the nearest line on path
        min_dist = sys.maxint
        min_dist_idx = -1
        for i in range(0, len(line_way_point_indices)):
            dist = i - car_way_point_index
            if dist>0 and min_dist > dist:
                min_dist = dist
                min_dist_idx = i    
        nearesr_line_way_point_index = line_way_point_indices[min_dist_idx]
        
        # Third check wheter the nearest light is visable
        # Check whether the position is visable
        X,Y = stop_line_positions[min_dist_idx]
        Z = 0
        position_visable = False
        x,y = self.project_to_image_plane([X,Y,Z])
        if x>0 and x<self.config['camera_info']['image_width']:
            position_visable = True

        if position_visable:
            state = self.get_light_state(light)
            return nearesr_line_way_point_index, state

        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
