#!/usr/bin/env python3
# pomodoro_timer.py

import time
import threading
import rospy
from qt_robot_interface.srv import behavior_talk_text
from std_msgs.msg import String
import os
import fcntl
CONVERSATION_LOCK_PATH = "/tmp/qt_robot_conversation.lock"


class PomodoroTimer:
    def __init__(self):
        self.pomodoro_duration = 25 * 60  # Pomodoro duration in seconds
        self.short_break_duration = 5 * 60  # Short break duration in seconds
        self.long_break_duration = 2 * 60  # Long break duration in seconds
        self.remaining_time = self.pomodoro_duration
        self.timer_thread = None
        self.paused = False
        self.stop_event = threading.Event()
        self.state_pub = rospy.Publisher('/pomodoro_state', String, queue_size=10)
        rospy.wait_for_service('/qt_robot/behavior/talkText')
        self.talkText = rospy.ServiceProxy('/qt_robot/behavior/talkText', behavior_talk_text)
        self.start_engagement_pub = rospy.Publisher('start_engagement', String, queue_size=10)
        self.talking_pub = rospy.Publisher('stop_listening', String, queue_size=10)
        self.emotion_publisher = rospy.Publisher('/qt_robot/emotion/show', String, queue_size=10)
        self.session_count = 0  # To keep track of completed Pomodoro sessions
        self.total_sessions = 1  # Sessions before long break  # Changed from 4 to 1
        # Start a thread to monitor the lock file 
        self.lock_monitor_thread = threading.Thread(target=self.monitor_lock_file) 
        self.lock_monitor_thread.daemon = True 
        self.lock_monitor_thread.start() 
    
    def monitor_lock_file(self):
        """
        Monitor /tmp/conversation.lock once per second.
        If the file is locked by emotion detection (and no 'release' marker), pause the timer.
        If the file indicates it will be released soon, resume the timer if it was paused.
        """
        while not rospy.is_shutdown():
            locked, locked_by_emotion, releasing_soon = self.check_lock_status()

            if locked and locked_by_emotion:
                # If the lock is active and not releasing soon, we pause the Pomodoro.
                if not releasing_soon:
                    if not self.paused:
                        rospy.loginfo("Emotion lock detected. Pausing Pomodoro timer.")
                        self.pause()
                else:
                    # If the file indicates lock release is upcoming, resume if paused.
                    if self.paused:
                        rospy.loginfo("Lock release detected. Resuming Pomodoro timer.")
                        self.resume()
            else:
                # If there's no lock file or it's not emotion-based, resume if paused
                # (That way we don't stay stuck in 'paused' if the file is removed).
                if self.paused and locked_by_emotion is None:
                    rospy.loginfo("No emotion lock or file removed. Resuming Pomodoro timer.")
                    self.resume()

            time.sleep(1)

    def check_lock_status(self):
        """
        Returns a tuple: (locked: bool, locked_by_emotion: bool or None, releasing_soon: bool)
          locked: True if lock file exists at all
          locked_by_emotion: True if the file indicates it's locked by emotion detection;
                             False if locked by something else;
                             None if no lock file exists.
          releasing_soon: True if the file indicates lock release text is present
        """
        if not os.path.exists(CONVERSATION_LOCK_PATH):
            return (False, None, False)

        locked = True
        locked_by_emotion = False
        releasing_soon = False

        try:
            with open(CONVERSATION_LOCK_PATH, "r") as lock_file:
                content = lock_file.read()
                if "Locked by Emotion Detector." in content:
                    locked_by_emotion = True
                if "Freeing lock by Emotion detector in 2 minutes." in content:
                    releasing_soon = True
        except Exception as e:
            rospy.logerr(f"Error reading lock file: {e}")

        return (locked, locked_by_emotion, releasing_soon)

    def talk(self, text):
        try:
            rospy.loginfo(f"Talking: {text}")
            self.talkText(text)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def start_pomodoro(self):
        rospy.loginfo("Starting Pomodoro session...")
        self.start_engagement_pub.publish('start')
        self.state_pub.publish('work')
        self.talking_pub.publish('stop')
        time.sleep(1)

        #self.talk("Your work session has started.")
        time.sleep(1)
        self.talking_pub.publish('start')
        self.stop_event.clear()
        self.paused = False
        self.remaining_time = self.pomodoro_duration
        self.timer_thread = threading.Thread(
            target=self.run_timer,
            args=(self.pomodoro_duration, self.finish_pomodoro)
        )
        self.timer_thread.start()

    def run_timer(self, duration, callback):
        flag = False
        short_break = False
        if flag == False and duration == self.short_break_duration:
            flag = True
            short_break = True

        if short_break == True:
            remaining_time = duration
            self.emotion_publisher.publish("break")
            while remaining_time > 0 and not self.stop_event.is_set():
                if self.paused:
                    # When paused, stop displaying the current emotion
                    time.sleep(1)
                    continue

                
                time.sleep(1)
                remaining_time -= 1

        else:
            remaining_time = duration
            self.current_emotion = None
            while remaining_time > 0 and not self.stop_event.is_set():
                if self.paused:
                    time.sleep(1)
                    continue

                time.sleep(1)
                remaining_time -= 1

        if not self.stop_event.is_set():
            callback()

    def finish_pomodoro(self):
        self.talk("Your Pomodoro session has ended!")
        self.session_count += 1
        if self.session_count >= self.total_sessions:
            # self.long_break()  # Removed to prevent long break
            self.short_break()  # Changed to short_break instead of long_break
        else:
            self.short_break()

    def short_break(self):
        rospy.loginfo("Starting short break...")
        self.state_pub.publish('break')
        self.start_engagement_pub.publish('stop')
        self.talking_pub.publish('stop')
        time.sleep(1)
        self.talk("Your short break has started. You can relax for a while.")
        
        time.sleep(1)
        self.talking_pub.publish('start')
        self.stop_event.clear()
        self.paused = False
        self.timer_thread = threading.Thread(
            target=self.run_timer,
            args=(self.short_break_duration, self.stop_pomodoro)  # Changed callback from start_pomodoro to stop_pomodoro
        )
        self.timer_thread.start()

    def long_break(self):
        rospy.loginfo("Starting long break...")
        self.state_pub.publish('break')
        self.start_engagement_pub.publish('stop')
        self.talking_pub.publish('stop')
        time.sleep(1)
        self.talk("Great job! Your long break has started. You deserve a longer rest.")
        time.sleep(1)
        self.talking_pub.publish('start')
        self.stop_event.clear()
        self.paused = False
        self.timer_thread = threading.Thread(
            target=self.run_timer,
            args=(self.long_break_duration, self.finish_long_break)
        )
        self.timer_thread.start()

    def finish_long_break(self):
        self.talk("Your Pomodoro cycle has ended!")
        self.stop_pomodoro()

    def pause(self):
        rospy.loginfo("Pausing Pomodoro timer...")
        self.paused = True
        self.state_pub.publish('paused')

    def resume(self):
        rospy.loginfo("Resuming Pomodoro timer...")
        self.paused = False
        self.state_pub.publish('work')

    def stop_pomodoro(self):
        rospy.loginfo("Stopping Pomodoro session...")
        self.stop_event.set()
        self.paused = False
        self.state_pub.publish('pomodoro_completed')
        try:
            self.emotion_publisher.publish('stop')
            self.start_engagement_pub.publish('stop')
            self.talk("Your Pomodoro focus session has been stopped.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed during stop_pomodoro: {e}")

    def end_cycle(self):
        rospy.loginfo("Pomodoro cycle completed.")
        self.state_pub.publish('pomodoro_completed')
        self.start_engagement_pub.publish('stop')
        self.talking_pub.publish('stop')
        time.sleep(1)
        self.talk("Congratulations! You've completed all your Pomodoro sessions.")
        time.sleep(1)
        self.talking_pub.publish('start')

