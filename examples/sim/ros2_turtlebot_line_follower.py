#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

import numpy as np
import cv2
import cv_bridge

# Create a bridge between ROS and OpenCV
bridge = cv_bridge.CvBridge()

## User-defined parameters: (Update these values to your liking)
# Minimum size for a contour to be considered anything
MIN_AREA = 500 

# Minimum size for a contour to be considered part of the track
MIN_AREA_TRACK = 5000

# Robot's speed when following the line
MAX_LINEAR_SPEED = 0.22

# Proportional constant to be applied on speed when turning 
# (Multiplied by the error value)
KP = 0.005

# Send messages every $TIMER_PERIOD seconds
TIMER_PERIOD = 0.06

# The maximum error value for which the robot is still in a straight line
MAX_ERROR = 30

class LineFollower(Node):
    def __init__(self) :
        super().__init__('line_follower')
        self.get_logger().info('Start line following')

        # BGR values to filter only the selected color range
        self.lower_bgr_values = np.array([0,  0,  0])
        self.upper_bgr_values = np.array([80, 80, 80])
        self.crop_h_start = 0
        self.crop_h_stop = 0
        self.crop_w_start = 0
        self.crop_w_stop = 0

        qos = QoSProfile(depth=10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', qos)
        subscription = self.create_subscription(Image, '/rgb',
                                                self.image_callback,
                                                qos)

        timer = self.create_timer(TIMER_PERIOD, self.timer_callback)

    def crop_size(self, height, width):
        """
        Get the measures to crop the image
        Output:
        (Height_upper_boundary, Height_lower_boundary,
        Width_left_boundary, Width_right_boundary)
        """
        ## Update these values to your liking.
        return (1*height//2, height, 0, width)


    def image_callback(self, msg):
        """
        Function to be called whenever a new Image message arrives.
        """
        self.image_input = bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        # self.get_logger().info('Received image')

    def get_contour_data(self, mask, out):
        """
        Return the centroid of the largest contour in
        the binary image 'mask' (the line) 
        and return the side in which the smaller contour is (the track mark) 
        (If there are any of these contours),
        and draw all contours on 'out' image
        """ 
        # get a list of contours
        print(type(mask))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]

        # mark = {}
        line = {}

        for i,contour in enumerate(contours):
            
            M = cv2.moments(contour)
            # Search more about Image Moments on Wikipedia :)

            if M['m00'] > MIN_AREA:
            # if countor.area > MIN_AREA:

                if (M['m00'] > MIN_AREA_TRACK):
                    # Contour is part of the track
                    line['x'] = self.crop_w_start + int(M["m10"]/M["m00"])
                    line['y'] = int(M["m01"]/M["m00"])
                    
                    # plot the area in green
                    cv2.drawContours(out, contour, -1, (0,255,0), 3) 
                    cv2.putText(out, str(M['m00']), (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"])),
                        cv2.FONT_HERSHEY_PLAIN, 2, (255,255,0), 2)                             

        return line

    def timer_callback(self):
        """
        Function to be called when the timer ticks.
        According to an image 'image_input', determine the speed of the robot
        so it can follow the contour
        """        
        error = 0

        # Wait for the first image to be received
        if type(self.image_input) != np.ndarray:
            self.get_logger().info('not received image')
            return

        height, width, _ = self.image_input.shape

        image = self.image_input.copy()

        self.crop_h_start, self.crop_h_stop, self.crop_w_start, self.crop_w_stop = self.crop_size(height, width)


        # get the bottom part of the image (matrix slicing)
        crop = image[self.crop_h_start:self.crop_h_stop, self.crop_w_start:self.crop_w_stop]

        # get a binary picture, where non-zero values represent the line.
        # (filter the color values so only the contour is seen)
        mask = cv2.inRange(crop, self.lower_bgr_values, self.upper_bgr_values)

        # get the centroid of the biggest contour in the picture,
        # and plot its detail on the cropped part of the output image
        output = image
        line = self.get_contour_data(mask, output[self.crop_h_start:self.crop_h_stop, self.crop_w_start:self.crop_w_stop])  
        # also get the side in which the track mark "is"
        
        message = Twist()
        
        if line:
        # if there even is a line in the image:
        # (as the camera could not be reading any lines)   
            x = line['x']

            # error:= The difference between the center of the image
            # and the center of the line
            error = x - width//2

            message.linear.x = MAX_LINEAR_SPEED

            # plot the line centroid on the image
            cv2.circle(output, (line['x'], self.crop_h_start + line['y']), 5, (0,255,0), 7)
        
        # Determine the speed to turn and get the line in the center of the camera.
        message.angular.z = float(error) * -KP
        # print("Error: {} | Angular Z: {}, ".format(error, message.angular.z))     

        # Plot the boundaries where the image was cropped
        cv2.rectangle(output, (self.crop_w_start, self.crop_h_start), (self.crop_w_stop, self.crop_h_stop), (0,0,255), 2)

        # Uncomment to show the binary picture
        cv2.imshow("mask", mask)

        # Show the output image to the user
        cv2.namedWindow('output', flags=cv2.WINDOW_NORMAL)
        cv2.imshow("output", output)
        # Print the image for 5milis, then resume execution
        cv2.waitKey(1)

        # # Publish the message to 'cmd_vel'
        self.publisher.publish(message)

def main(args=None):
    rclpy.init(args=args)

    line_follower = LineFollower()

    rclpy.spin(line_follower)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    empty_message = Twist()
    line_follower.publisher.publish(empty_message)
    line_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()