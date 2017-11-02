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


STATE_COUNT_THRESHOLD = 3
classes= ['Red', 'Yellow', 'Green','Null']
Show_img = False
traffic_light_classifier = '/traffic_light_classifier/cnn'

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.has_image = False
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
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb, queue_size = 1)
        '''
        The permanent (x, y) coordinates for each traffic light's stop line are provided
        by the config dictionary, which is imported from the traffic_light_config file:
        '''
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        #Calling the class for classifying the color of the traffic light
        self.light_classifier = TLClassifier(os.path.dirname(__file__) + traffic_light_classifier)
        self.listener = tf.TransformListener()
        self.loop()
        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

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
    #loop through to capture all states
    #def loop(self):

        #rate = rospy.Rate(10) # in Hz
        #while not rospy.is_shutdown():
            #rate.sleep()
    def loop(self):
        rate = rospy.Rate(10) #hz
        while not rospy.is_shutdown():
            if(not self.has_image):
                rospy.loginfo("Images aren't seen")
                continue
            light_wp, state = self.process_traffic_lights()
            if state == None:
                rospy.loginfo("The state is None")
                continue

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        light_distance = 0
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
############################################################################
            if(self.light_wp != -1):
                #check if there are upcoming traffic lights, see the explination id readme for more details
                light_distance = self.pose_distance(self.pose.pose,self.waypoints.waypoints[self.last_wp].pose.pose)    
                rospy.loginfo("Distance to traffic light 0.3f\n", light_distance)
            else:
                rospy.loginfo("No upcoming lights")
############################################################################
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            if(self.light_wp != -1):
                #check if there are upcoming traffic lights, see the explination id readme for more details
                light_distance = self.pose_distance(self.pose.pose,self.waypoints.waypoints[self.last_wp].pose.pose)    
                rospy.loginfo("Distance to traffic light 0.3f\n", light_distance)
            else:
                rospy.loginfo("No upcoming lights")

            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1
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
            rospy.loginfo("Closest waypoint %s\n at distance  %s\n", index, min_distance)
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
            rospy.loginfo("Closest traffic light %s\n at distance  %s\n", traffic_light, min_distance)
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
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        
        ##Sanity check ans resize
        if cv_image.size != (800,600,3):
             cv_image = resized_image = cv2.resize(cv_image, (800, 600)) 
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        #Get classification
        ########################################################
        ############ Under construction ########################
        ########################################################
        if self.light_classifier == None:
            classification = TrafficLight.UNKNOWN
        else:



        ########################################################
            return self.light_classifier.get_classification(cv_image)

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
            #car_position = self.get_closest_waypoint(self.pose.pose)
             light, light_distance = self.get_closest_traffic_light(self.pose.pose, stop_line_positions)
        #TODO find the closest visible traffic light (if one exists)

        if light:
            #state = self.get_light_state(light)
            #return light_wp, state
            light_index = self.get_closest_waypoint(light.pose.pose)
            state = self.get_light_state(light)
            rospy.loginfo("Closest Light at %0.2fm is Red? %s\n", light_distance, (state==TrafficLight.RED))
            return light_index, state
        self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
