#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLight
from std_msgs.msg import Int32
from geometry_msgs.msg import TwistStamped

import math, sys
from itertools import islice, cycle
import numpy as np
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

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. 
MAX_SPEED_METERS_PER_SEC = 10*0.447
SLOWDOWN_WPS = 50 # Number of waypoints before traffic light to start slowing down

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint 
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.waypoints = None # all waypoints
        self.waypoints_size = None
        self.current_pose = None     # 
        self.final_waypoints = None
        self.current_velocity = None
        self.red_light_wp = -1
        self.wrn_rot_licht = False
        self.last_wp_id = None
        self.next_light_state = None
        self.next_light_wp = None
        self.max_velocity = 1 # meter/sec
        self.lookahead_wps = 0
        self.limit_traffic_ahead = 1 # when closer than these many meters is the light limit it

        rospy.spin()

    def send_waypoints(self):

        for ii in range(self.lookahead_wps):
            # initialize to max velocity 
            velocity = self.max_velocity
            if self.wrn_rot_licht:
                point_von_licht = self.traffic_point - self.pos_point
                chk_licht_pt = (ii < point_von_licht) & (point_von_licht > 1) & (point_von_licht < self.lookahead_wps)
                if chk_licht_pt:
                    dist_tr_licht = self.distance(self.final_waypoints, ii, point_von_licht + 1)
                    if (dist_tr_licht < self.limit_traffic_ahead):
                        velocity = 0.0

            self.set_waypoint_velocity(self.final_waypoints, ii, velocity)

        self.Publish()

    def Publish(self):
        l = Lane()
        l.header = self.waypoints.header
        l.waypoints = self.final_waypoints
        self.final_waypoints_pub.publish(l)

    #Position updater topic
    def pose_cb(self, msg):
        ''' 
          lets return closest points '''
        dist_func = lambda a, b: math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2)
        dist = []  # lets find distance of waypoints from current waypoint
        if self.waypoints:
            for waypoint in self.waypoints.waypoints:
                dist.append(dist_func(waypoint.pose.pose.position, msg.pose.position))
            self.pos_point = np.argmin(dist)

            if self.pos_point + self.lookahead_wps + 1 > self.waypoints_size:
                list_end = self.pos_point + self.lookahead_wps + 1 - self.waypoints_size 
                self.final_waypoints = self.waypoints.waypoints[self.pos_point:] + self.waypoints.waypoints[:list_end]
            else:
                self.final_waypoints = self.waypoints.waypoints[self.pos_point : self.pos_point + self.lookahead_wps + 1]

            self.send_waypoints()

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        self.waypoints_size = np.shape(waypoints.waypoints)[0]
        self.lookahead_wps = min(LOOKAHEAD_WPS, self.waypoints_size//2)
        rospy.logwarn("# of waypoints ".format(self.waypoints_size))
        self.max_velocity = self.get_waypoint_velocity(waypoints.waypoints[0])
        self.limit_traffic_ahead = self.max_velocity

    def traffic_cb(self, msg):
        # Read data from msg and mark as traffic points
        if (msg.data >= 0):
            self.traffic_point = msg.data
            self.wrn_rot_licht = True
        else:
            self.traffic_point = None
            self.wrn_rot_licht = False


    def obstacle_cb(self, msg):
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
