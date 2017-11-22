#!/usr/bin/env python
'''
This node takes in data from the /image_color, /current_pose, and /base_waypoints topics
and publishes the locations to stop for red traffic lights to the /traffic_waypoint topic.

The /current_pose topic provides the vehicle's current position, and /base_waypoints
provides a complete list of waypoints the car will be following.
Traffic light detection takes place within tl_detector.py, whereas traffic light 
classification takes place within ../tl_detector/light_classification_model/tl_classfier.py.
'''
import os
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
import math

LOG_RATE = 4

STATE_COUNT_THRESHOLD = 2
classes= ['Red', 'Yellow', 'Green','Null']
traffic_light_classifier = '/traffic_light_classifier/cnn'

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.has_image = False
        self.lights = []

        #sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        #self.sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size = 1)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size = 1)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb, queue_size = 1)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size = 1)
        sub7 = rospy.Subscriber('/image_raw', Image, self.image_cb, queue_size=1)
        '''
        The permanent (x, y) coordinates for each traffic light's stop line are provided
        by the config dictionary, which is imported from the traffic_light_config file:
        '''
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)
        
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
        self.bridge = CvBridge()
        #Calling the class for classifying the color of the traffic light
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()
        self.state = TrafficLight.UNKNOWN

        #self.last_state = TrafficLight.UNKNOWN
        #self.last_wp = -1
        #self.state_count = 0
        #self.image_index = 0
        #rospy.spin()

        self.red_state_count = 0
        self.loop()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        self.sub2.unregister()
    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.image_index += 1
        if self.image_index % 5 != 1:
            print('--- ignoring image')
            return
        self.has_image = True
        self.camera_image = msg
    #loop through to capture all states
    #def loop(self):

        #rate = rospy.Rate(10) # in Hz
        #while not rospy.is_shutdown():
            #rate.sleep()
    def loop(self):
        rate = rospy.Rate(10) #hz
        while not rospy.is_shutdown():
            if(not self.has_image):
                rospy.logwarn_throttle(LOG_RATE, "[TLD] Images aren't seen")
                continue
            light_wp, state = self.process_traffic_lights()

            if self.state != state:
                rospy.logwarn_throttle(LOG_RATE, "[TLD] State update from {} to {}".format(self.state, state))
            self.state = state
                
            if state == None:
                rospy.logwarn_throttle(LOG_RATE, "[TLD] The state is None")
                rate.sleep()
                continue

            if light_wp <= 0:
                rospy.logwarn_throttle(LOG_RATE, "[TLD] Light wp index is 0")
                rate.sleep()
                continue

            
            '''
            Publish upcoming red lights at camera frequency.
            Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
            of times till we start using it. Otherwise the previous stable state is
            used.
            '''
            if state != TrafficLight.RED:
                self.red_state_count = 0
            else:
                self.red_state_count += 1
            
            if self.red_state_count >= STATE_COUNT_THRESHOLD:
                rospy.logwarn_throttle(LOG_RATE, "[TLD] Publishing detected lights at wp {}".format(light_wp))
                self.upcoming_red_light_pub.publish(Int32(light_wp))
            else:
                rospy.logwarn_throttle(LOG_RATE, "[TLD] Publishing red light not detected")
                self.upcoming_red_light_pub.publish(Int32(-1))
            
            rate.sleep()
#Utility functions
#############################################################################
    def pose_distance(self, pose_from, pose_to):
        """Find the Euclidian distance between two coordinates  
        
        Args:
                        pose_from (Pose): position from calculation
            pose_to (Pose): position to calculation
        Returns:
            Euclidean distance (double): Distance between two coordinates in meters
        """
        x1= pose_from.position.x
        y1= pose_from.position.y
        x2= pose_to.position.x
        y2= pose_to.position.y
        x = x2 - x1
        y = y2 - y1
        return math.sqrt((x*x) + (y*y))
    def get_orientations(self,quat):
        quaternion = [quat.x, quat.y, quat.z, quat.w]
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)
        return roll,pitch,yaw

    def get_vector(self, quat):
        roll, pitch, yaw = self.get_orientations(quat)
        x = math.cos(yaw) * math.cos(pitch)
        y = math.sin(yaw) * math.cos(pitch)
        z = math.sin(pitch)
        return x, y, z
#############################################################################
    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        if(self.waypoints and self.waypoints.waypoints):
            min_index = 0
            #minimum distance to traffic light
            min_distance = 1e9
            for index,waypoint in enumerate(self.waypoints.waypoints):
                #convert into eucledian
                distance = self.pose_distance(waypoint.pose.pose,pose)
                if(distance < min_distance):
                    min_distance = distance
                    min_index = index
            rospy.logwarn_throttle(LOG_RATE, "[TLD] Closest waypoint {}\n at distance  {}\n".format(min_index, min_distance))
            return min_index
        return 0
    def get_closest_traffic_light(self, pose, light_positions):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to
            light_position: the location of the traffic light
        Returns:
            int: index of the closest traffic light in self.lights

        """
        #TODO implement
        if(light_positions):
            min_light = None
            #minimum distance to traffic light
            min_distance = 1e9
            for index,light in enumerate(light_positions):
                #convert into eucledian
                traffic_light = TrafficLight()
                traffic_light.header.stamp=rospy.Time(0)

                traffic_light.pose.pose.position.x = light[0]
                traffic_light.pose.pose.position.y = light[1]

                distance = self.pose_distance(traffic_light.pose.pose,pose)
                if(distance < min_distance and self.is_waypoint_in_front_of_vehicle(traffic_light.pose.pose,pose)):
                    min_distance = distance
                    min_light = traffic_light

            return min_light,min_distance
        return None
    def is_waypoint_in_front_of_vehicle(self,waypoint,pose):
        """Check if the waypoint is infront of the car's orientation (pose).  
        Args:
 
        Returns:
            boolean 

        """
        # orientation
        x_vec, y_vec,z_vec = self.get_vector(pose.orientation)
        waypoint_dist = self.pose_distance(pose, waypoint)
        vec_dist =  math.sqrt((waypoint.position.x-pose.position.x - x_vec*0.1)**2 + (waypoint.position.y-pose.position.y - y_vec*0.1)**2)

        return vec_dist < waypoint_dist

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image or self.light_classifier == None):
            return TrafficLight.UNKNOWN

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")
        
        ##Sanity check ans resize
        if cv_image.size != (800,600,3):
             cv_image = resized_image = cv2.resize(cv_image, (800, 600)) 
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        #Get classification
        ########################################################
        ############ Under construction ########################
        ########################################################
        classification = self.light_classifier.get_classification(cv_image)
        rospy.logwarn_throttle(LOG_RATE, "[TLD] classifying traffic light on frame as: {}".format(classification))
        return classification

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
             light, light_distance = self.get_closest_traffic_light(self.pose.pose, stop_line_positions)
        #TODO find the closest visible traffic light (if one exists)

        if light != None and light_distance < 200.0:
            light_index = self.get_closest_waypoint(light.pose.pose)
            state = self.get_light_state(light)
            rospy.logwarn_throttle(LOG_RATE, "[TLD] Closest Light at {}m is Red? {}\n".format(light_distance, (state==TrafficLight.RED)))
            return light_index, state
        self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
