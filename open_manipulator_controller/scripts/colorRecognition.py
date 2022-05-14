#!/usr/bin/env python3

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist
import cv2
import rospy
try:
    from queue import Queue
except ImportError:
    from Queue import Queue
import threading
import numpy as np
from std_msgs.msg import String

class BufferQueue(Queue):
    """Slight modification of the standard Queue that discards the oldest item
    when adding an item and the queue is full.
    """
    def put(self, item, *args, **kwargs):
        # The base implementation, for reference:
        # https://github.com/python/cpython/blob/2.7/Lib/Queue.py#L107
        # https://github.com/python/cpython/blob/3.8/Lib/queue.py#L121
        with self.mutex:
            if self.maxsize > 0 and self._qsize() == self.maxsize:
                self._get()
            self._put(item)
            self.unfinished_tasks += 1
            self.not_empty.notify()

class cvThread(threading.Thread):
    """
    Thread that displays and processes the current image
    It is its own thread so that all display can be done
    in one thread to overcome imshow limitations and
    https://github.com/ros-perception/image_pipeline/issues/85
    """
    def __init__(self, queue):
        threading.Thread.__init__(self)
        self.queue = queue
        self.image = None


    def run(self):
        # Create a single OpenCV window
        cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("frame", 800,600)

        while True:
            self.image = self.queue.get()

            # Initialize published message
            coord_message = ""

            # Process the current image
            maskR, contourR, crosshairR, coordinatesR = self.processImage(self.image, "R")
            maskB, contourB, crosshairB, coordinatesB = self.processImage(self.image, "B")

            # Add processed images as small images on top of main image
            result = self.addSmallPictures(self.image, [maskR, contourR, crosshairR])
            cv2.imshow("frame", result)

            #publishing to node
            for coord in coordinatesR:
                coord_message += "R," + str(coord[0]) + "," + str(coord[1])
                #if index < len(coordinatesR)-1:
                coord_message += ";"

            for coord in coordinatesB:
                coord_message += "B," + str(coord[0]) + "," + str(coord[1])
                #if index < len(coordinatesB)-1:
                coord_message += ";"

            pub.publish(coord_message)

            # Check for 'q' key to exit
            k = cv2.waitKey(6) & 0xFF
            if k in [27, ord('q')]:
                rospy.signal_shutdown('Quit')

    def processImage(self, img, channel, treshold=10):

        rows,cols = img.shape[:2]

        R,G,B = self.convert2rgb(img)

        redMask = self.thresholdBinary(R, (255 - treshold, 255))
        greenMask = self.thresholdBinary(G, (255 - treshold, 255))
        blueMask = self.thresholdBinary(B, (255 - treshold, 255))


        if channel == "R":
            stackedMask = np.dstack((redMask, redMask, redMask))
            contourMask = stackedMask.copy()
            crosshairMask = stackedMask.copy()
            (contours,hierarchy) = cv2.findContours(redMask.copy(), 1, cv2.CHAIN_APPROX_NONE)
        elif channel == "G":
            stackedMask = np.dstack((greenMask, greenMask, greenMask))
            contourMask = stackedMask.copy()
            crosshairMask = stackedMask.copy()
            (contours,hierarchy) = cv2.findContours(greenMask.copy(), 1, cv2.CHAIN_APPROX_NONE)
        elif channel == "B":
            stackedMask = np.dstack((blueMask, blueMask, blueMask))
            contourMask = stackedMask.copy()
            crosshairMask = stackedMask.copy()
            (contours,hierarchy) = cv2.findContours(blueMask.copy(), 1, cv2.CHAIN_APPROX_NONE)
        else:
            binary = np.zeros_like(img)
            stackedMask = np.dstack((binary, binary, binary))
            contourMask = stackedMask.copy()
            crosshairMask = stackedMask.copy()
            (contours,hierarchy) = cv2.findContours(binary.copy(), 1, cv2.CHAIN_APPROX_NONE)
        

        # return value of findContours depends on OpenCV version
        # (_, contours,hierarchy) = cv2.findContours(redMask.copy(), 1, cv2.CHAIN_APPROX_NONE)

        coordinates = []
        if len(contours) > 0:
            for contour in contours:
                if cv2.contourArea(contour) > 50:
                    #c = max(contours, key=cv2.contourArea)
                    M = cv2.moments(contour)

                    # Make sure that "m00" won't cause ZeroDivisionError: float division by zero
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                    else:
                        cx, cy = 0, 0

                    # Show contour and centroid
                    cv2.drawContours(contourMask, contour, -1, (0,255,0), 10)
                    cv2.circle(contourMask, (cx, cy), 5, (0, 255, 0), -1)

                    if cx != 0 and cy != 0:
                        coordinates.append([cx, cy])
                    # Show crosshair and difference from middle point
                    #cv2.line(crosshairMask,(cx,0),(cx,rows),(0,0,255),10)
                    #cv2.line(crosshairMask,(0,cy),(cols,cy),(0,0,255),10)


        # Return processed frames
        return redMask, contourMask, crosshairMask, coordinates

    # Convert to RGB channels
    def convert2rgb(self, img):
        R = img[:, :, 2]
        G = img[:, :, 1]
        B = img[:, :, 0]

        return R, G, B

    # Apply threshold and result a binary image
    def thresholdBinary(self, img, thresh=(200, 255)):
        binary = np.zeros_like(img)
        binary[(img >= thresh[0]) & (img <= thresh[1])] = 1

        return binary*255

    # Add small images to the top row of the main image
    def addSmallPictures(self, img, small_images, size=(160, 120)):
        '''
        :param img: main image
        :param small_images: array of small images
        :param size: size of small images
        :return: overlayed image
        '''

        x_base_offset = 40
        y_base_offset = 10

        x_offset = x_base_offset
        y_offset = y_base_offset

        for small in small_images:
            small = cv2.resize(small, size)
            if len(small.shape) == 2:
                small = np.dstack((small, small, small))

            img[y_offset: y_offset + size[1], x_offset: x_offset + size[0]] = small

            x_offset += size[0] + x_base_offset

        return img

def queueMonocular(msg):
    try:
        # Convert your ROS Image message to OpenCV2
        cv2Img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except CvBridgeError as e:
        print(e)
    else:
        qMono.put(cv2Img)

print("OpenCV version: %s" % cv2.__version__)

queueSize = 1      
qMono = BufferQueue(queueSize)

cvThreadHandle = cvThread(qMono)
cvThreadHandle.setDaemon(True)
cvThreadHandle.start()

bridge = CvBridge()

rospy.init_node('color_recognition')
# Define your image topic
image_topic = "/head_camera/image_raw"
# Set up your subscriber and define its callback
rospy.Subscriber(image_topic, Image, queueMonocular)

pub = rospy.Publisher('/piece_coordinates', String, queue_size=1)
# Spin until Ctrl+C
rospy.spin()
