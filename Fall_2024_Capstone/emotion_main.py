#!/usr/bin/env python3
import rospy
import time
from emotion_detection_array import EmotionDetector
from qt_robot_interface.srv import behavior_talk_text
from std_msgs.msg import String
import threading
import os
import fcntl
from working_prompt import start_conversation

EMOTION_CONV_LOCK = "/tmp/conversation.lock"

def attempt_emotion_lock():
    """
    Try to acquire the conversation lock for emotion detection.
    Returns True if successful; otherwise False.
    """
    # If lock file is already present, bail out
    if os.path.isfile(EMOTION_CONV_LOCK):
        return False

    try:
        # Create and mark it locked by emotion detection
        with open(EMOTION_CONV_LOCK, 'w') as lf:
            lf.write("locked by emotion detection.\n")
            lf.flush()

        lock_file = open(EMOTION_CONV_LOCK, 'r+')
        fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        # Release the handle right away (optional)
        fcntl.flock(lock_file, fcntl.LOCK_UN)
        lock_file.close()
        return True
    except IOError:
        return False
    except Exception as e:
        print(f"Error acquiring emotion lock: {e}")
        return False

def release_emotion_lock():
    """
    Immediately unlock and remove the emotion lock file.
    """
    if not os.path.isfile(EMOTION_CONV_LOCK):
        return
    try:
        lock_file = open(EMOTION_CONV_LOCK, 'r+')
        fcntl.flock(lock_file, fcntl.LOCK_UN)
        lock_file.close()
        os.remove(EMOTION_CONV_LOCK)
        print("Emotion lock released.")
    except Exception as e:
        print(f"Error releasing emotion lock: {e}")

def release_emotion_lock_with_delay(delay_seconds):
    """
    Writes a message to the lock file and then releases it after delay_seconds.
    """
    def _delay_unlock():
        try:
            with open(EMOTION_CONV_LOCK, 'a') as lf:
                lf.write("Releasing emotion lock in 2 minutes.")
                lf.flush()
            time.sleep(delay_seconds)
            release_emotion_lock()
        except Exception as e:
            print(f"Error in delayed emotion lock release: {e}")

    threading.Thread(target=_delay_unlock, daemon=True).start()
    
 
class EmotionMonitor:
    def __init__(self):
        print("Starting Emotion Monitor...")
        # rospy.init_node('emotion_monitor_node')
        self.emotion_detector = EmotionDetector()
        self.talkText = rospy.ServiceProxy('/qt_robot/behavior/talkText', behavior_talk_text)
        self.emotion_publisher = rospy.Publisher('/qt_robot/emotion/show', String, queue_size=10)

        self.is_conversation_active = False
        self.last_conversation_time = 0
        self.conversation_cooldown = 300

        self.analysis_interval = 60
        self.emotion_detector.max_history_size = int(self.analysis_interval / self.emotion_detector.detection_interval_sec)

        self.last_check_time = time.time()

    def run(self):
        try:
            while not rospy.is_shutdown():
                frame, emotion = self.emotion_detector.detect_emotion()
                if emotion is not None:
                    print("Detected Emotion: ", emotion)

                if time.time() - self.last_check_time >= self.analysis_interval:
                    self.check_emotion_threshold()
                    self.last_check_time = time.time()

                rospy.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            self.emotion_detector.release_resources()

    def is_negative_emotion(self, emotion):
        negative_emotions = ['angry', 'disgust', 'fear', 'sad', 'surprise']
        return emotion.lower() in negative_emotions

    def check_emotion_threshold(self):
        if not self.emotion_detector.emotion_history:
            return

        negative_count = sum(1 for emotion in self.emotion_detector.emotion_history if self.is_negative_emotion(emotion))
        total_count = len(self.emotion_detector.emotion_history)
        negative_percentage = (negative_count / total_count) * 100

        self.emotion_detector.emotion_history.clear()

        threshold = 80  
        if negative_percentage >= threshold and not self.is_conversation_active:
            self.handle_negative_emotion_detected()

    def handle_negative_emotion_detected(self, emotion=None):
        current_time = time.time()
        if self.is_conversation_active:
            return
        if current_time - self.last_conversation_time < self.conversation_cooldown:
            return

        self.is_conversation_active = True
        if not emotion:
            emotion = next((e for e in reversed(self.emotion_detector.emotion_history) if self.is_negative_emotion(e)), "sad")

        rospy.loginfo(f"Detected negative emotion: {emotion}")
        if not attempt_emotion_lock():
            print("Conversation locked by another script.")
        else:    
            user_feels_better = start_conversation(emotion)  # Uses working_prompt with input()
            if user_feels_better:
                rospy.loginfo("User feels better now.")
            else:
                rospy.loginfo("Conversation ended.")
            release_emotion_lock_with_delay(100)
            print("Lock will be released in 2 minutes.")
            self.last_conversation_time = current_time
        self.is_conversation_active = False

if __name__ == "__main__":
    monitor = EmotionMonitor()
    monitor.run()
