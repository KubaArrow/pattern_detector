#!/usr/bin/env python3
import rospy
from pattern_detector_action_server.pattern_detector_action_server import PatternDetectorActionServer


if __name__ == '__main__':
    server = PatternDetectorActionServer(rospy.get_name())
    rospy.spin()
