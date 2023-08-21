#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
from geometry_msgs.msg import PoseStamped

from pattern_detector.msg import (pattern_detector_actionResult,
                                  pattern_detector_actionFeedback,
                                  pattern_detector_actionAction)
from pattern_detector_core.pattern_detector_ros import PatternDetectorROS


class PatternDetectorActionServer:
    """
    This class represents a ROS Action Server that utilizes a pattern detector to provide pose information.
    """

    _result = pattern_detector_actionResult()
    _feedback = pattern_detector_actionFeedback()

    def __init__(self, name: str):
        """
        Initializes the PatternDetectorActionServer class.

        :param name: The name of the action server.
        """
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name, pattern_detector_actionAction, self.execute_cb, False
        )
        self.pose = None
        rospy.loginfo("PatternDetectorActionServer is started.")
        self.pattern_detector = PatternDetectorROS()

        # Subscribe to the results from PatternDetectorROS
        self._result_sub = rospy.Subscriber("/pattern_detector_ros/pose", PoseStamped, self.result_cb)
        self._as.start()

    def execute_cb(self, goal):
        """
        Executes the action server's callback function.

        :param goal: The goal message sent by the action client.
        """
        rospy.loginfo("PatternDetectorActionServer has received a goal.")
        rate = rospy.Rate(5)
        if goal.activate:
            self.pattern_detector.start()
        else:
            self.pattern_detector.stop()

        while not rospy.is_shutdown():
            if self.pose is not None:
                self._result.pose = self.pose
                self._as.set_succeeded(self._result)
                self._feedback.status = "founded"
                self._as.publish_feedback(self._feedback)
                break
            else:
                self._feedback.status = "looking_for"
                self._as.publish_feedback(self._feedback)
            rate.sleep()

    def result_cb(self, msg: PoseStamped):
        """
        Processes the result callback function.

        :param msg: The message from the result callback.
        """
        rospy.loginfo("PatternDetectorActionServer received a result.")
        rospy.loginfo(f"Result: {msg}")
        self.pose = msg
