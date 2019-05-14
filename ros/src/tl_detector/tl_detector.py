#!/usr/bin/env python
import sys
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
from scipy.spatial import KDTree
import tf
import cv2
import yaml
import time
import json
import copy
from datetime import datetime

import os
dir = os.path.dirname(__file__)

STATE_COUNT_THRESHOLD = 3
TEST_MODE_ENABLED = False
DEBUG_MODE_ENABLED = True
DEBUG_MODE_DISABLED = False

class TLDetector(object):

    def now(self):
        return str(datetime.now().strftime('%I:%M:%S.%f'))

    def log(self, msg):
        filename = os.path.join(dir, '../../../master.log')
        f = open(filename,"a+")
        f.write('{} [tl_detector]: {}\n'.format(self.now(), msg))
        f.close()

    def __init__(self):
        rospy.init_node('tl_detector')
        self.debug_mode = DEBUG_MODE_ENABLED
        
        try:
            self.pose = None
            self.waypoints = None
            self.camera_image = None
            self.lights = []
            self.waypoints_2d = []
            self.ready = True

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
            self.light_classifier = TLClassifier(rospy.get_param('~model_file'))
            self.listener = tf.TransformListener()

            self.state = TrafficLight.UNKNOWN
            self.last_state = TrafficLight.UNKNOWN
            self.last_wp = -1
            self.state_count = 0

            rospy.spin()
        except Exception as e:
            print(e, sys.stderr)

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x,waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        if self.ready:
            start = time.time();
            self.ready = False

            self.has_image = True
            self.camera_image = msg
            light_wp, state = self.process_traffic_lights()

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
                self.log('Publish:: /traffic_waypoint index: {}'.format(self.last_wp))
                self.upcoming_red_light_pub.publish(Int32(light_wp))
            else:
                self.log('Publish:: /traffic_waypoint index: {}'.format(self.last_wp))
                self.upcoming_red_light_pub.publish(Int32(self.last_wp))
            self.state_count += 1

            self.ready = True
            end = time.time();
            duration = (end - start) * 1000
            self.log('Frame processing duration: {} ms'.format(duration))

    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        closest_idx = self.waypoint_tree.query([x,y],1)[1]
        return closest_idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #return light.state
    
    
        # if(not self.has_image):
        #     self.prev_light_loc = None
        #     return False

        # cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        # #Get classification
        # return self.light_classifier.get_classification(cv_image)
        # For test mode, just return the light state
        if TEST_MODE_ENABLED:
            classification = light.state
        else:
            cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
            # cv2.imwrite(filename, cv_image)

            # Get classification
            classification = self.light_classifier.get_classification(cv_image)

            # Save image (throttled)
            #if SAVE_IMAGES and (self.process_count % LOGGING_THROTTLE_FACTOR == 0):
                #save_file = "../../../imgs/{}-{:.0f}.jpeg".format(self.to_string(classification), (time.time() * 100))
                #cv2.imwrite(save_file, cv_image)

        return classification

    def write_image(self, data):
        if self.debug_mode:
            filename = data["filename"]
            cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
            dt = data["time"]["colon"]
            light_color = data["lights"]["final"]["color"]
            average = data["lights"]["final"]["average"]

            for box in data["boxes"]:
                xmin = box["xmin"]
                ymin = box["ymin"]
                xmax = box["xmax"]
                ymax = box["ymax"]
                score = box["score"]

                bounding_box_color = (255, 255, 255)
                if light_color == 'RED':
                    bounding_box_color = (0, 0, 255)
                elif light_color == 'GREEN':
                    bounding_box_color = (0, 255, 0)
                cv2.rectangle(cv_image, (xmin, ymin), (xmax, ymax), bounding_box_color, 3)

                confidence = '{}'.format(score)
                cv2.putText(cv_image, confidence, (xmin+10, ymin+10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1, cv2.LINE_AA)

            text = '{} / {} ({})'.format(dt, light_color, average)
            cv2.putText(cv_image, text, (20, 20), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2, cv2.LINE_AA)
            cv2.imwrite(filename, cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        closest_light = None
        line_wp_idx = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)
        else:
            return -1, TrafficLight.UNKNOWN

        #TODO find the closest visible traffic light (if one exists)
        diff = len(self.waypoints.waypoints)

        for i,light in enumerate(self.lights):
            line = stop_line_positions[i]
            temp_wp_idx = self.get_closest_waypoint(line[0],line[1])
            d = temp_wp_idx - car_wp_idx
            if d >= 0 and d < diff:
                diff = d
                closest_light = light
                line_wp_idx = temp_wp_idx

        self.log('Traffic light waypoint: {}, Current waypoint: {}'.format(line_wp_idx, car_wp_idx))
        if closest_light:
            data = self.get_light_state(closest_light)
            data["waypoints"] = {
                "traffic_light": line_wp_idx,
                "current": car_wp_idx
            }
            self.log(json.dumps(data))
            self.log("***** {} *****".format(data["lights"]["final"]["color"]))
            self.write_image(data)
            return line_wp_idx, data["lights"]["final"]["state"]

        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
