#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import Image
from engagement_detector import EngagementDetector
from time_serie_in_image import TimeSerieInImage

import threading
from client import VideoStreamClient
class ROSEngagementDetector():
    def __init__(self, plot_in_image=True, out_image_topic="/engagement_detector/out_image"):
        self.plot_in_image = plot_in_image
        self.bridge = CvBridge()
        self.detector = EngagementDetector()
        self.client = VideoStreamClient(host="127.0.0.1", port=3000)
        # Initialize the webcam (change index as necessary)
        # self.cap = cv2.VideoCapture(6)  # Change the index if necessary

        # # Check if the camera opened successfully
        # if not self.cap.isOpened():
        #     rospy.logerr("Error: Could not open video device.")
        #     rospy.signal_shutdown("Camera initialization failed")  # Shutdown ROS node if camera fails

        self.eng_pub = rospy.Publisher("/engagement_level", Float32, queue_size=1)

        if self.plot_in_image:
            self.image_plotter = TimeSerieInImage()
            self.outImg_pub = rospy.Publisher(out_image_topic, Image, queue_size=1)

        self.image_seq = []
        self.sequence_lock = threading.Lock()
        self.last_value = 0.0  # Initialize last_value here

        self.display_engagement = True  # By default, display engagement levels
        self.display_sub = rospy.Subscriber('/display_engagement', Bool, self.display_callback)

    def display_callback(self, msg):
        self.display_engagement = msg.data

    def capture_image(self):
        # ret, cv_image = self.cap.read()
        # if not ret:
        #     rospy.logwarn("Failed to capture image from webcam.")
        #     return None
        # return cv_image
        frame = self.client.receive_frame()
        if frame is None:
            rospy.logwarn("No frame received from FrameProvider.")
        return frame

    def spin(self, hz=10):
        rospy.loginfo("Starting engagement detection...")

        # Capture images in a loop
        rate = rospy.Rate(hz)
        while not rospy.is_shutdown():
            # Capture image from webcam
            cv_image = self.capture_image()
            if cv_image is None:
                rospy.logwarn("No image captured. Retrying...")
                continue

            last_img = np.asarray(cv_image)

            # Add the captured image to the sequence
            with self.sequence_lock:
                self.image_seq.append(last_img.copy())
                if len(self.image_seq) > 10:
                    self.image_seq.pop(0)

            # Process predictions if we have enough images
            if len(self.image_seq) >= 10:
                # Copy the sequence to avoid threading issues
                with self.sequence_lock:
                    tmp_image_seq = list(self.image_seq)

                prediction = self.detector.predict(tmp_image_seq)
                if prediction is None:
                    rospy.logwarn("Could not make a prediction, probably the frame sequence length is not 10.")
                    continue

                value = float(np.squeeze(prediction))
                self.eng_pub.publish(value)
                self.last_value = value  # Update last_value with the current prediction

                if not self.display_engagement:
                    rospy.loginfo("Engagement levels are hidden during conversation.")
                else:
                    rospy.loginfo(f"Predicted engagement score: {value}")  # Log the engagement score

            if self.plot_in_image and self.display_engagement:
                # Plot engagement value on the current frame
                out_img = self.image_plotter.step(last_img.copy(), self.last_value)
                try:
                    out_imgmsg = self.bridge.cv2_to_imgmsg(out_img, "bgr8")
                    self.outImg_pub.publish(out_imgmsg)
                except CvBridgeError as e:
                    rospy.logerr(e)

                # Display the image with the engagement plot
                # cv2.imshow("Engagement Plot", out_img)  # Display the image with the plot
                cv2.waitKey(1)  # Allow OpenCV to process the window events

            rate.sleep()

        # Release the webcam and close any OpenCV windows when shutting down
        # self.cap.release()
        self.client.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    rospy.init_node("engagement_detector")

    debug_image = rospy.get_param("~debug_image", True)
    out_image = rospy.get_param("~out_image", "/engagement_detector/out_image")

    red = ROSEngagementDetector(plot_in_image=debug_image, out_image_topic=out_image)

    try:
        red.spin()
    except rospy.ROSInterruptException:
        pass
    


