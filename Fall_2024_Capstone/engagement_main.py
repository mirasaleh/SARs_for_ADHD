#!/usr/bin/env python3

import rospy
import time
from std_msgs.msg import Float32, String, Bool
from pomodoro_timer import PomodoroTimer
from prompt import start_conversation, post_pomodoro_conversation
import threading
import subprocess
import os
import signal
import logging
import fcntl

   
ENGAGEMENT_CONV_LOCK = "/tmp/conversation.lock"

# Remove any stale lock file on startup
if os.path.isfile(ENGAGEMENT_CONV_LOCK):
    os.remove(ENGAGEMENT_CONV_LOCK)

def try_engagement_lock():
    """
    Attempt to create and lock the engagement conversation file.
    Returns True if successful, False otherwise.
    """
    if os.path.exists(ENGAGEMENT_CONV_LOCK):
        # Lock file already present; cannot acquire
        return False
    try:
        # Create and write lock info
        with open(ENGAGEMENT_CONV_LOCK, 'w') as lf:
            lf.write("locked by engagement detection.\n")
            lf.flush()
        # Acquire the file lock using EXclusive + NonBlocking flags
        lock_file = open(ENGAGEMENT_CONV_LOCK, 'r+')
        fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
        # Immediately unlock the file handle here (or keep it open if you prefer)
        fcntl.flock(lock_file, fcntl.LOCK_UN)
        lock_file.close()
        return True
    except IOError:
        return False
    except Exception as e:
        print(f"Error while acquiring engagement lock: {e}")
        return False

def free_engagement_lock():
    """
    Unlock and remove the engagement conversation lock file.
    """
    if not os.path.isfile(ENGAGEMENT_CONV_LOCK):
        return  # Nothing to release
    try:
        lock_file = open(ENGAGEMENT_CONV_LOCK, 'r+')
        fcntl.flock(lock_file, fcntl.LOCK_UN)
        lock_file.close()
        os.remove(ENGAGEMENT_CONV_LOCK)
        print("Engagement lock released.")
    except Exception as e:
        print(f"Error freeing engagement lock: {e}")

def free_engagement_lock_with_delay(delay_seconds):
    """
    Schedule a delayed release of the engagement lock.
    Writes a note to the lock file indicating a future release,
    then removes the file after the delay.
    """
    def _delayed_release():
        try:
            with open(ENGAGEMENT_CONV_LOCK, 'a') as lf:
                lf.write("Releasing engagement lock in 2 minutes.\n")
                lf.flush()
            time.sleep(delay_seconds)
            free_engagement_lock()
        except Exception as e:
            print(f"Error in delayed lock release: {e}")

    threading.Thread(target=_delayed_release, daemon=True).start()


class MainApp:
    def __init__(self):
        self.engagement_levels = []
        rospy.loginfo("Initializing engagement main...")
        self.engagement_sub = rospy.Subscriber('/engagement_level', Float32, self.engagement_callback)
        self.pomodoro_timer = PomodoroTimer()
        self.distraction_tasks = []
        self.engagement_monitoring = False
        self.engagement_timer = None
        self.is_conversation_active = False
        self.last_conversation_time = None
        self.cooldown_period = 120
        self.conversation_requested = False
        self.display_engagement_pub = rospy.Publisher('/display_engagement', Bool, queue_size=10)
        self.pomodoro_state = 'stopped'
        self.pomodoro_state_sub = rospy.Subscriber('/pomodoro_state', String, self.pomodoro_state_callback)
        self.engagement_detector_process = None
        self.is_shutting_down = False
        self.engagement_detector_started = False
        signal.signal(signal.SIGINT, self.shutdown)

    def shutdown(self, signum, frame):
        rospy.loginfo("Shutting down MainApp...")
        self.is_shutting_down = True
        self.stop_engagement_monitoring()
        self.pomodoro_timer.stop_pomodoro()
        self.stop_engagement_detector()
        self.stop_website_blocker()
        self.engagement_sub.unregister()
        rospy.signal_shutdown("User requested shutdown.")

    def engagement_callback(self, msg):
        if not self.engagement_detector_started:
            self.engagement_detector_started = True
            rospy.loginfo("Engagement detector has started publishing.")
            self.pomodoro_timer.start_pomodoro()
        if self.engagement_monitoring:
            self.engagement_levels.append((rospy.get_time(), msg.data))
            if not self.is_conversation_active:
                rospy.loginfo(f"Received engagement level: {msg.data}")

    def start_engagement_monitoring(self):
        self.engagement_levels = []
        self.engagement_monitoring = True
        self.engagement_timer = rospy.Timer(rospy.Duration(30), self.check_engagement)

    def stop_engagement_monitoring(self):
        self.engagement_monitoring = False
        if self.engagement_timer:
            self.engagement_timer.shutdown()
            self.engagement_timer = None

    def pomodoro_state_callback(self, msg):
        if self.is_shutting_down:
            return
        self.pomodoro_state = msg.data
        if self.pomodoro_state == 'work':
            self.start_engagement_monitoring()
            self.display_engagement_pub.publish(Bool(True))
            self.resume_engagement_detector()
            self.start_website_blocker()
        elif self.pomodoro_state == 'break':
            self.stop_engagement_monitoring()
            self.display_engagement_pub.publish(Bool(False))
            self.pause_engagement_detector()
            self.stop_website_blocker()
        elif self.pomodoro_state == 'paused':
            self.stop_engagement_monitoring()
        elif self.pomodoro_state == 'pomodoro_completed':
            self.stop_engagement_monitoring()
            self.display_engagement_pub.publish(Bool(False))
            self.handle_pomodoro_completion()
            self.stop_website_blocker()
        else:
            rospy.logwarn(f"Unknown pomodoro state: {self.pomodoro_state}")

    def check_engagement(self, event):
        if self.engagement_monitoring:
            current_time = rospy.get_time()
            self.engagement_levels = [(t, val) for t, val in self.engagement_levels if current_time - t <= 30]
            if self.engagement_levels:
                values = [val for t, val in self.engagement_levels]
                average_engagement = sum(values) / len(values)
                if not self.is_conversation_active:
                    rospy.loginfo(f"Average engagement over last 30 seconds: {average_engagement}")
                if average_engagement < 0.93:
                    current_time = time.time()
                    if not self.is_conversation_active and \
                       (self.last_conversation_time is None or (current_time - self.last_conversation_time) > self.cooldown_period):
                        if try_engagement_lock():
                            self.conversation_requested = True
                        else:
                            print("Conversation locked by another script.")
                    else:
                        if not self.is_conversation_active:
                            rospy.loginfo("Cannot start conversation now.")
            else:
                if not self.is_conversation_active:
                    rospy.logwarn("No engagement levels collected in the last 30 seconds.")

    def run_conversation(self):
        self.is_conversation_active = True
        self.display_engagement_pub.publish(Bool(False))
        self.pomodoro_timer.pause()
        self.pause_engagement_detector()
        try:
            logging.disable(logging.CRITICAL)
            distraction_tasks = start_conversation()  # Uses prompt.py, now with terminal input
            if distraction_tasks:
                self.distraction_tasks.extend(distraction_tasks)
        except Exception as e:
            logging.disable(logging.NOTSET)
            rospy.logerr(f"Error during conversation: {e}")
        finally:
            logging.disable(logging.NOTSET)
            self.is_conversation_active = False
            self.last_conversation_time = time.time()
            self.display_engagement_pub.publish(Bool(True))
            self.pomodoro_timer.resume()
            free_engagement_lock_with_delay(100)
            print("Lock will be released in 2 minutes.")
            self.resume_engagement_detector()

    def handle_pomodoro_completion(self):
        if self.is_shutting_down:
            return
        rospy.loginfo("Pomodoro cycle completed.")
        self.pause_engagement_detector()
        if self.distraction_tasks:
            additional_tasks = post_pomodoro_conversation(self.distraction_tasks)
            if additional_tasks:
                self.distraction_tasks.extend(additional_tasks)
        else:
            self.pomodoro_timer.talk("Your Pomodoro session has ended. Great job!")

    def start_engagement_detector(self):
        # Start engagement detector node (assuming roslaunch)
        # Make sure your launch file is correct
        package = 'engagement_detector'
        launch_file = 'engagement_detector.launch'
        command = ['roslaunch', package, launch_file]
        self.engagement_detector_process = subprocess.Popen(command, preexec_fn=os.setsid)
        rospy.loginfo("Started engagement detector node.")

    def stop_engagement_detector(self):
        if self.engagement_detector_process:
            os.killpg(os.getpgid(self.engagement_detector_process.pid), signal.SIGTERM)
            rospy.loginfo("Stopped engagement detector node.")

    def pause_engagement_detector(self):
        if self.engagement_detector_process:
            os.killpg(os.getpgid(self.engagement_detector_process.pid), signal.SIGSTOP)
            rospy.loginfo("Paused engagement detector node.")

    def resume_engagement_detector(self):
        if self.engagement_detector_process:
            os.killpg(os.getpgid(self.engagement_detector_process.pid), signal.SIGCONT)
            rospy.loginfo("Resumed engagement detector node.")

    def start_website_blocker(self):
        rospy.loginfo("Blocking websites...")
        script_path = '/home/qtrobot/catkin_ws/src/mira/engagement_detector/engagement_prompt/website_blocker.py'
        command = ['sudo', '/usr/bin/python3', script_path, 'block']
        try:
            subprocess.run(command, check=True)
            rospy.loginfo("Websites are blocked.")
        except subprocess.CalledProcessError as e:
            rospy.logerr(f"Failed to block websites: {e}")

    def stop_website_blocker(self):
        rospy.loginfo("Unblocking websites...")
        script_path = '/home/qtrobot/catkin_ws/src/mira/engagement_detector/engagement_prompt/website_blocker.py'
        command = ['sudo', '/usr/bin/python3', script_path, 'unblock']
        try:
            subprocess.run(command, check=True)
            rospy.loginfo("Websites are unblocked.")
        except subprocess.CalledProcessError as e:
            rospy.logerr(f"Failed to unblock websites: {e}")

    def run(self):
        self.start_engagement_detector()
        rospy.loginfo("Waiting for engagement detector to start publishing...")
        while not self.engagement_detector_started and not rospy.is_shutdown():
            time.sleep(0.1)
        try:
            while not rospy.is_shutdown():
                time.sleep(1)
                if self.conversation_requested:
                    self.conversation_requested = False
                    self.run_conversation()
        except KeyboardInterrupt:
            pass
        self.stop_engagement_detector()
        self.stop_website_blocker()

if __name__ == "__main__":
    # Normally, you wouldn't run engagement_main directly if you have combined_main
    # but this is kept for backward compatibility.
    #rospy.init_node('main_app_node')
    app = MainApp()
    app.run()
