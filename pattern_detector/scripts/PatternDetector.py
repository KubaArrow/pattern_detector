#!/usr/bin/env python3

import rospy

from pattern_detector_core.pattern_detector_ros import PatternDetectorROS

if __name__ == '__main__':
    rospy.init_node('pattern_detector')
    processor = PatternDetectorROS()
    processor.start()
    rospy.spin()
