#!/usr/bin/env python3
import cv2
from deepface import DeepFace
from collections import Counter
import time
import rospy
from qt_robot_interface.srv import behavior_talk_text
from std_msgs.msg import String
from client import VideoStreamClient


class EmotionDetector:
    def __init__(self, host="127.0.0.1", port=3000):
        rospy.wait_for_service('/qt_robot/behavior/talkText')
        self.talkText = rospy.ServiceProxy('/qt_robot/behavior/talkText', behavior_talk_text)
        self.talkText("I will start the focus session soon! Good luck you got this!")
        time.sleep(20)
        self.client = VideoStreamClient(host, port)
        self.emotion_history = []
        self.last_detection_time = time.time()
        self.detection_interval_sec = 5  # Detect emotion every 5 seconds
        self.max_history_size = 12  # 1 minute of data at 5-second intervals
        self.first_frame = True

    def detect_emotion(self):
        frame = self.client.receive_frame()

        if frame is None:
            rospy.logerr("No frames received")
            return None, None
        
        # cv2.imshow("Frame", frame)

        # Limit emotion detection to every 5 seconds
        if time.time() - self.last_detection_time >= self.detection_interval_sec:
            try:
                result = DeepFace.analyze(frame, actions=['emotion'], enforce_detection=False)
                dominant_emotion = max(result[0]['emotion'], key=result[0]['emotion'].get)

                # Store the detected emotion and limit history size
                self.emotion_history.append(dominant_emotion)
                if len(self.emotion_history) > self.max_history_size:
                    self.emotion_history.pop(0)

                # Update last detection time
                self.last_detection_time = time.time()
                # self.display_frame(frame, dominant_emotion)
                # Return the frame and emotion for display purposes
                return frame, dominant_emotion

            except Exception as e:
                rospy.logerr(f"Emotion detection error: {e}")
                return frame, None
        else:
            return frame, None

    def get_mode_emotion(self):
        if not self.emotion_history:
            return None

        # Get the mode (most common emotion)
        most_common_emotion = Counter(self.emotion_history).most_common(1)[0][0]
        # Clear the emotion history for the next interval
        self.emotion_history.clear()
        return most_common_emotion

    def release_resources(self):
        self.client.close()

    def display_frame(self, frame, emotion=None):
        if frame is not None:
            if emotion:
                cv2.putText(frame, f"Emotion: {emotion}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow('Emotion Detection', frame)
            cv2.waitKey(1)
