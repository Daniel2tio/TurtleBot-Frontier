#!/usr/bin/env python3
from random import uniform
from math import sqrt, pi as PI, isnan, cos, sin, radians

import rospy
import numpy as np
from std_msgs.msg import Int32
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, PoseStamped, Pose, Quaternion, Point
from nav_msgs.msg import Path, Odometry
import tf
import cv2
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from cv_bridge import CvBridge, CvBridgeError
from tf import transformations
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
from tf.transformations import euler_from_quaternion

# Define some constants
CLOSE_DISTANCE_THRESHOLD = 20
FORWARD_SPEED = 0.2
TURN_SPEED = ((2 * PI) / 360) * 25  # Angular speed (25 degrees per second)
WALK_DISTANCE = 3.0
MIN_DISTANCE = 0.38
LOW_GREEN = np.array([50, 255, 0])  # Lower bound of green color in HSV
HIGH_GREEN = np.array([100, 255, 255])  # Upper bound of green color in HSV

LOW_BLUE = np.array([115,255,0])
HIGH_BLUE = np.array([120,255,255])

LOW_RED = np.array([0,255,0])
HIGH_RED = np.array([5,255,255])\

MARKER_ID = 0

# Define a class for the Turtlebot follower
class Follower():
    '''
    Will beacon a Turtlebot3 Waffle toward green objects or wander around if not.
    '''

    def __init__(self):
        rospy.init_node('follower', anonymous=True)
        rospy.on_shutdown(self.shutdown)
        self.bridge = CvBridge()
        self.depth_image = []
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback) 
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.twist = Twist()
        self.closest_object = None  # Store information about the closest green object

        # transform buffer for robot pose
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # transfer a message
        self.msg_pub = rospy.Publisher('flag_msg',Int32,queue_size=1)

        # For path plotting
        self.path_publisher = rospy.Publisher("/path", Path, queue_size=10)
        self.path = Path()
        #self.plot_subscriber = rospy.Subscriber("odom", Odometry, self.plot_trajectory)

        # For obstacle avoidance
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.obstacle = True
        self.direction = 0
        self.direction_set = False

        self.blueTileMove = False
        self.FoundBlueTile = False
        self.green_object_counter = 0
        self.red_object_counter = 0
        
        # For markers
        self.g_flag = True
        self.r_flag = True
        self.marker_array = MarkerArray()
        self.mark_pub = rospy.Publisher("/marker_array", MarkerArray, queue_size= 1)


        # For odometry and position tracking
        self.curr_pose = {"theta": 0.0, "x": 0.0, "y": 0.0}
        self.last_pose = {"theta": 0.0, "x": 0.0, "y": 0.0}
        self.turning = False
        self.travel_angle = 0.0
        self.object_detected = False
        self.objects = 0
        rospy.loginfo("No object detected")


    def drive_turn_left(self):
        '''
        Turns left
        '''
        turn_left = Twist()
        turn_left.linear.x = 0
        turn_left.angular.z = TURN_SPEED
        #self.vel_pub.publish(turn_left)

    def drive_turn_right(self):
        '''
        Turns right
        '''
        turn_right = Twist()
        turn_right.linear.x = 0
        turn_right.angular.z = -TURN_SPEED
        #self.vel_pub.publish(turn_right)

    def drive_forward(self):
        '''
        Drives forwards
        '''
        forward = Twist()
        forward.linear.x = FORWARD_SPEED
        forward.angular.z = 0
        #self.vel_pub.publish(forward)

    def drive_stop(self):
        '''
        Stops driving
        '''
        self.msg_pub.publish(0)
        stop = Twist()
        stop.linear.x = 0
        stop.angular.z = -TURN_SPEED
        #self.vel_pub.publish(stop)


    def depth_callback(self, data):

        depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        (h,w) = depth_image.shape[:2]
        depth_image = cv2.resize(depth_image, (w//4, h//4))
        self.depth_image = depth_image

    def image_callback(self, data):
        if len(self.depth_image) == 0:
            return
        try:
            #Cashes the depth image
            depth_image = self.depth_image
            pose = self.tfBuffer.lookup_transform('map', 'base_footprint', rospy.Time())
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            cv_image_resized = cv2.resize(cv_image, (cv_image.shape[1] // 4, cv_image.shape[0] // 4))
            image_hsv = cv2.cvtColor(cv_image_resized, cv2.COLOR_BGR2HSV)

            b_mask = cv2.inRange(image_hsv, LOW_BLUE, HIGH_BLUE)
            # b_object = self.get_blue_centroids(b_mask, cv_image_resized) 
            # self.is_robot_close_to_blue_object(b_object, image_hsv, depth_image, pose)


            g_mask = cv2.inRange(image_hsv, LOW_GREEN, HIGH_GREEN)
            g_objects = self.get_centroids(g_mask)

            # Count green objects when the robot is close
            self.is_robot_close_to_green_object(g_objects, image_hsv, depth_image, pose)

            r_mask = cv2.inRange(image_hsv, LOW_RED, HIGH_RED)
            #r_objects = self.get_centroids(r_mask)

            # Count red objects when the robot is close
            #self.is_robot_close_to_red_object(r_objects, image_hsv, depth_image, pose)

            #cv2.imshow('hsv', image_hsv)
            immask =  cv2.bitwise_and(image_hsv,image_hsv, mask=g_mask)
            cv2.imshow('mask', immask)

            self.closest_object = self.find_closest_object(g_objects, cv_image_resized.shape[1])
            self.drive(cv_image_resized.shape[1])
            #self.closest_object = self.find_closest_object(r_objects, cv_image_resized.shape[1])

            #self.draw_centroids(b_objects, cv_image_resized)
            self.draw_centroids(g_objects, cv_image_resized)
            #self.draw_centroids(r_objects, cv_image_resized)
            
            #Marking Object
            self.mark_pub.publish(self.marker_array)
            

            cv2.waitKey(3)

        except CvBridgeError as error:
            print(error)

    def is_robot_close_to_green_object(self, green_objects, image, depth_image, pose):
        """
        Check if the robot is close to a green object based on some criteria.
        Adjust the criteria based on your robot's behavior and environment.
        """
        image_h,image_w,d = image.shape

        for object_x, object_y in green_objects:
            depth = depth_image[object_y, object_x]
            distance = sqrt((object_x - image_w / 2)**2 + (object_y - image_h / 2)**2)
            if not (isnan(depth)):
                if (distance < CLOSE_DISTANCE_THRESHOLD):
                    self.drive_stop()
                    rospy.loginfo('Found the green object.')
                    #Adjusts the angle of object x based on the camera fov
                    theta = euler_from_quaternion([pose.transform.rotation.x,pose.transform.rotation.y, pose.transform.rotation.z, pose.transform.rotation.w])[2]
                    theta -= radians(-34.5 + ((object_x * 69)/self.depth_image.shape[1])) # nice
                    r_object_pose = (pose.transform.translation.x + (depth*cos(theta)), pose.transform.translation.y + (depth*sin(theta)))
                    object_unique = True
                    for marker in self.marker_array.markers:
                        marker_pose = marker.pose.position
                        if (marker_pose.x - r_object_pose[0])**2 + (marker_pose.y - r_object_pose[1])**2 < (1.425 **2):
                            object_unique = False
                    if object_unique:
                        self.marker_array.markers.append(self.convert_marker(r_object_pose, r=0.0, g=1.0,sx=0.6,sy=0.6,sz=0.6))
                        rospy.loginfo(r_object_pose)



    def is_robot_close_to_red_object(self, red_objects, image, depth_image, pose):
        """
        Check if the robot is close to a green object based on some criteria.
        Adjust the criteria based on your robot's behavior and environment.
        """
        image_h,image_w,d = image.shape

        for object_x, object_y in red_objects:
            depth = depth_image[object_y, object_x]
            if not (isnan(depth)):
                if (depth < CLOSE_DISTANCE_THRESHOLD):
                    rospy.loginfo('Found the red object. Total number ')
                    #Adjusts the angle of object x based on the camera fov
                    theta = euler_from_quaternion([pose.transform.rotation.x,pose.transform.rotation.y, pose.transform.rotation.z, pose.transform.rotation.w])[2]
                    theta -= radians(-34.5 + ((object_x * 69)/self.depth_image.shape[1]))
                    r_object_pose = (pose.transform.translation.x + (depth*cos(theta)), pose.transform.translation.y + (depth*sin(theta)))
                    object_unique = True
                    for marker in self.marker_array.markers:
                        marker_pose = marker.pose.position
                        if (marker_pose.x - r_object_pose[0])**2 + (marker_pose.y - r_object_pose[1])**2 < (1.425 **2):
                            object_unique = False
                    if object_unique:
                        self.marker_array.markers.append(self.convert_marker(r_object_pose,sx=0.6,sy=0.6,sz=0.6))
                        rospy.loginfo(r_object_pose)

    
    def is_robot_close_to_blue_object(self, red_objects, image, depth_image, pose):
        """
        Check if the robot is close to a green object based on some criteria.
        Adjust the criteria based on your robot's behavior and environment.
        """
        for object_x, object_y in red_objects:
            depth = depth_image[object_y, object_x]
            if not (isnan(depth)):
                if (depth < CLOSE_DISTANCE_THRESHOLD):
                    rospy.loginfo('Found the blue tile')
                    #Adjusts the angle of object x based on the camera fov
                    theta = euler_from_quaternion([pose.transform.rotation.x,pose.transform.rotation.y, pose.transform.rotation.z, pose.transform.rotation.w])[2]
                    theta -= radians(-34.5 + ((object_x * 69)/self.depth_image.shape[1]))
                    b_object_pose = (pose.transform.translation.x + (depth*cos(theta)), pose.transform.translation.y + (depth*sin(theta)))
                    b_object_x = pose.transform.translation.x + (depth*cos(theta))
                    b_object_y = pose.transform.translation.y + (depth*sin(theta))

        
                    
                    

    def avoid_blueTile(self):
        # Move backward for a short duration
        backward_duration = rospy.Duration.from_sec(0.5)
        backward_start_time = rospy.Time.now()
        
        backward_twist = Twist()
        backward_twist.linear.x = -FORWARD_SPEED
        #self.vel_pub.publish(backward_twist)

        while rospy.Time.now() - backward_start_time < backward_duration:
            pass  # Wait for the specified duration

        # Stop the robot
        stop_twist = Twist()
        #self.vel_pub.publish(stop_twist)

        # Turn away from the blue tile
        turn_twist = Twist()
        turn_twist.angular.z = TURN_SPEED
        #self.vel_pub.publish(turn_twist)

        # Wait for a short duration for the turn
        rospy.sleep(0.5)

        # Stop the robot after turning
        self.drive_stop()
        self.FoundBlueTile = False

    
    def find_closest_object(self, objects, image_width):
        #print(objects)
        closest_object = None
        min_distance = float('inf')
        for object_x, object_y in objects:
            distance = abs(object_x - image_width / 2)
            if sqrt((object_y**2)+(distance**2))<min_distance:
                min_distance = sqrt((object_y**2)+(distance**2))
                closest_object = (object_x, min_distance)

        return closest_object
    
    def drive(self, image_width):
        if not self.obstacle:
            if self.closest_object:
                self.drive_beacon(image_width)
            # else:
            #     self.drive_random_walk()

    def drive_beacon(self, image_width):
        # need to judge the blue tile
        if self.closest_object and self.FoundBlueTile != True:

            err = self.closest_object[0] - image_width / 2
            # Set maximum linear speed
            self.twist.linear.x = FORWARD_SPEED
            self.twist.angular.z = -float(err) / 100
            rospy.loginfo('****************************************')
            self.vel_pub.publish(self.twist)
        else:
            self.avoid_blueTile()

    # def drive_random_walk(self):
    #     '''
    #     Will move forward a predefined distance and then rotate by a random angle
    #     '''
    #     if self.turning:
    #         if (self.travel_angle < 0 and self.curr_pose["theta"] > self.travel_angle) \
    #         or (self.travel_angle >= 0 and self.curr_pose["theta"] < self.travel_angle):
    #             if self.travel_angle > 0:
    #                 self.drive_turn_left()
    #             else:
    #                 self.drive_turn_right()
    #         else:
    #             rospy.loginfo("Driving without objects")
    #             self.drive_stop()
    #     else:
    #         progress = sqrt((self.last_pose["x"] - self.curr_pose["x"]) ** 2 \
    #         + (self.last_pose["y"] - self.curr_pose["y"]) ** 2)
    #         if progress <= WALK_DISTANCE:
    #             self.drive_forward()
    #             self.turning = False
    #         else:
    #             rospy.loginfo("Turning")
    #             self.drive_stop()
    #             self.travel_angle = uniform(-PI, PI)
    #             self.set_location_ref()
    #             self.set_angle()
    #             self.turning = True

    def scan_callback(self, scan):
        '''
        Handles the scanner data and determines the correct action
        '''
        min_val_angle = 0
        min_val = 50
        for i in range(320, 359):
            if scan.ranges[i] < min_val:
                min_val = scan.ranges[i]
                min_val_angle = i
        for i in range(0, 45):
            if scan.ranges[i] < min_val:
                min_val = scan.ranges[i]
                min_val_angle = i

        if min_val < MIN_DISTANCE:
            if not self.obstacle:
                rospy.loginfo("Obstacle detected")
                self.drive_stop()
                self.obstacle = True
                self.direction_set = False
            self.obstacle_avoidance(min_val_angle)
        else:
            if self.obstacle:
                rospy.loginfo("No obstacle detected")
                self.obstacle = False
                self.direction = 0
                self.direction_set = False
                self.drive_stop()
        

    def obstacle_avoidance(self, min_val_angle):
        '''
        Will identify the obstacle in the path of the Waffle and attempt to avoid it.
        '''
        if not self.direction_set:
            #rospy.loginfo("Obstacle avoidance")
            if min_val_angle < 90:
                self.direction = 2
                self.direction_set = True
            else:
                self.direction = 1
                self.direction_set = True
        else:
            if self.direction == 1:
                self.drive_turn_left()
            else:
                self.drive_turn_right()

    def draw_centroids(self, objects, image):
        '''
        Draws a magenta circle on the select ed object and a red circle on the other objects.
        Assumes the first object in the array is the target.
        '''
        for i, coordinates in enumerate(objects):
            if i == 0:
                cv2.circle(image, coordinates, 5, (255, 0, 255), 2)  # Magenta circle for the target
            else:
                cv2.circle(image, coordinates, 5, (0, 0, 255), 2)  # Red circle for other objects
        cv2.imshow("targets", image)
        

    def get_centroids(self, mask):
        '''
        Returns an array of centroids based on the number of masked objects
        '''
        # contours, _ = cv2.findContours(mask.copy(),
        #                                 cv2.RETR_TREE,
        #                                 cv2.CHAIN_APPROX_SIMPLE,
        #                                 offset=(0, 0))  # Finds the outlines of each object
        centroids = []
        #for contour in contours:  # Creates an array of the centers of each object
        moments = cv2.moments(mask)
        if moments["m00"] != 0:
            center_x = int(moments['m10'] / moments['m00'])
            center_y = int(moments['m01'] / moments['m00'])
            centroids.append((center_x, center_y))
            self.msg_pub.publish(1)
        # if len(contours) != self.objects:
        #     self.objects = len(contours)
        #     rospy.loginfo("Objects: " + str(self.objects))

        return centroids
    
    def get_blue_centroids(self,mask,image):
        """
        only detect the image of one meter in front of the robot
        """
        h,w,d = image.shape
        search_top = 3*h/4
        search_right = w/2+10
        search_left = w/2-10
        search_bot = search_top + 20
        mask[0:int(search_top), 0:w] = 0
        mask[int(search_bot):h, 0:w] = 0
        # mask[0:h,0:int(search_left)] = 0
        # mask[0:h,int(search_right):w] = 0
       
        centroids = []
        #for contour in contours:  # Creates an array of the centers of each object
        moments = cv2.moments(mask)
        if moments["m00"] > 0:
            cx = int(moments['m10'] / moments['m00'])
            cy = int(moments['m01'] / moments['m00'])
            rospy.loginfo("found the blue tile")
            cv2.circle(image,(cx, cy), 10, (0,0,255), -1)
            self.FoundBlueTile = True
            #self.drive_stop()
            #self.avoid_blueTile()
            centroids.append((cx, cy))

        return centroids

    def set_angle(self):
        '''
        Sets a random angle
        '''
        self.travel_angle = uniform(-PI, PI)
        rospy.loginfo("Travel angle: %f", self.travel_angle)

    def set_location_ref(self):
        '''
        Records the current location x, y, and theta as the last location
        '''
        rospy.loginfo("Recording last position")
        self.last_pose["theta"] = self.curr_pose["theta"]
        self.last_pose["x"] = self.curr_pose["x"]
        self.last_pose["y"] = self.curr_pose["y"]
        rospy.loginfo("Theta: %f, x: %f, y: %f",
                       self.last_pose["theta"],
                       self.last_pose["x"],
                       self.last_pose["y"])

    # def plot_trajectory(self, odom):
    #     '''
    #     Publishes the odometry data to /path so rviz can plot it.
    #     '''
    #     pose = PoseStamped()
    #     pose.header = odom.header
    #     pose.pose = odom.pose.pose

    #     self.path.header = odom.header
    #     self.path.poses.append(pose)
    #     self.path_publisher.publish(self.path)


        # quart = [odom.pose.pose.orientation.x,
        #          odom.pose.pose.orientation.y,
        #          odom.pose.pose.orientation.z,
        #          odom.pose.pose.orientation.w]

        # (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quart)

        # curr_pose = {}
        # curr_pose["theta"] = yaw
        # curr_pose["x"] = odom.pose.pose.position.x
        # curr_pose["y"] = odom.pose.pose.position.y
        # print(curr_pose["x"], curr_pose["y"])


    def convert_marker(self,points = None, 
                        w=1 , 
                        nx=0 , 
                        ny=0 , 
                        nz=0 , 
                        r=1.0, 
                        g=0.0, 
                        b=0.0, 
                        alpha=1.0, 
                        id=0, 
                        namespace='marker', 
                        type = 3, 
                        sx = 0.2, 
                        sy = 0.2, 
                        sz = 0.2,
                        z = 0 ) :
        """
            Creates marker object with entry information
        """        
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()

        # marker type = 8 should be a point, refer to the msg/Marker config for details.
        marker.ns = namespace
        marker.type = type
        marker.id = int(rospy.Time.now().nsecs)

        marker.pose.orientation.x = nx
        marker.pose.orientation.y = ny
        marker.pose.orientation.z = nz
        marker.pose.orientation.w = w

        marker.scale.x = sx
        marker.scale.y = sy
        marker.scale.z = sz
        # Set the color 
        marker.color.r = float(r)
        marker.color.g = float(g)
        marker.color.b = float(b)
        marker.color.a = float(alpha)

        if points == None :
            rospy.logerr('list of points need not be none in RviZ Publisher -> pub_point')
            return None

        if points is not None : 
            marker.pose = Pose(Point(points[0], points[1], 0.0), Quaternion(0,0,0,1))

        marker.lifetime = rospy.Duration(0)
        marker.frame_locked = True

        return marker

    def shutdown(self):
        '''
        Shutdown method in the event of ctrl + c.
        '''
        self.drive_stop()
        rospy.loginfo("Stopping")
        rospy.sleep(1)

def reset_position(new_x,new_y,new_yaw):
    rospy.wait_for_service('/gazebo/set_model_state')
    quar = transformations.quaternion_from_euler(0,0,new_yaw)

    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        model_state = ModelState()
        model_state.model_name = 'turtlebot3_waffle'
        model_state.pose = Pose(position = Point(x =new_x, y = new_y), orientation = Quaternion(*quar))
        model_state.twist = Twist()
        set_state(model_state)
    except rospy.ServiceException as e:
        rospy.logerr("")
def main():
    # Initialize ROS node and create a Follower instance
    # Initial_robot_pose()
    #rospy.init_node('follower', anonymous=True)
    x = -1.545471
    y = 4.2083094
    z = -1.57
    reset_position(x,y,z)
    Follower()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
