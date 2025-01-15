#!/usr/bin/env python3
import sys
import rospy
from qt_robot_interface.srv import *
from qt_vosk_app.srv import *

if __name__ == '__main__':

    rospy.init_node('my_tutorial_node')
    rospy.loginfo("my_tutorial_node started!")

    # define a ros service
    speechSay = rospy.ServiceProxy('/qt_robot/speech/say', speech_say)
    recognize = rospy.ServiceProxy('/qt_robot/speech/recognize', speech_recognize)
    # block/wait for ros service
    rospy.wait_for_service('/qt_robot/speech/say')
    rospy.wait_for_service('/qt_robot/speech/recognize')

    try:
        # call a ros service with text message
        speechSay("Say something after the beep.")
        speechSay('#CAR HORN#')
        resp = recognize("en_US", ['blue', 'green', 'red'], 10)
        rospy.loginfo("I got: %s", resp.transcript)
        speechSay("You said %s " % resp.transcript)
    except KeyboardInterrupt:
        pass

    rospy.loginfo("finsihed!")