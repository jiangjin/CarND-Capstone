#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math

from copy import deepcopy

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
        self.pose=None
        self.given_waypoints=None

        #to be able to store the old/last position
        self.old_pose=None

        #to be decided if they are going to be used
        self.new_pose=None
        self.old_wp=None
        self.new_wp=None

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement

        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        distance=9999
        index=None

        self.pose=msg.pose.position

        #to prevent situations where the simulator gives the /current_pose before providing the /base_points
        if self.given_waypoints is None:
            return

        else:
            if self.old_pose==msg.pose:
                self.new_wp=self.old_wp
            else:
                subset_waypoints=Lane()
                subset_end_index=None

                for i in range(len(self.given_waypoints)):
                    point_distance=dl(self.given_waypoints[i].pose.pose.position,self.pose)
                    if point_distance<distance:
                        distance=point_distance
                        index=i
                self.new_wp=index
                self.old_wp=self.new_wp
                self.old_pose=msg.pose

                #to fix the case when the last waypoint is outside of the waypoints list
                subset_end_index=self.new_wp+LOOKAHEAD_WPS
                if subset_end_index>len(self.given_waypoints)-1:
                    subset_end_index=len(self.given_waypoints)-1

                for i in range(self.new_wp,subset_end_index):
                    subset_waypoints.waypoints.append(deepcopy(self.given_waypoints[i]))

                 # set velocity for each waypoint
                for i in range(len(subset_waypoints.waypoints)):
                    self.set_waypoint_velocity(subset_waypoints.waypoints, i, 1)

                self.final_waypoints_pub.publish(subset_waypoints)


    def waypoints_cb(self, waypoints):
        # TODO: Implement

        #since /base_waypoints is only published once, the given_waypoints is only changed once as well.
        #I think we only need the .waypoints, so I've discarded the .header
        if self.given_waypoints is None:
            self.given_waypoints=waypoints.waypoints

        pass





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
