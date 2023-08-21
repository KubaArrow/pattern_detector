import math

import rospy
from geometry_msgs.msg import Pose, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from .script.point_cloud_processor import PointCloudProcessor
from .script.rviz_visualizer import RVizVisualizer
from .script.structures import PointCloud
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose


class PatternDetectorROS:
    """This class represents a pattern detector node in ROS (Robot Operating System).

    The class subscribes to a LaserScan topic, processes the incoming scans to detect patterns,
    and publishes the detected patterns as Pose messages to another topic.

    :param config: A dictionary of configurations for the pattern detector.
        (Optional: defaults are defined in the `__init__` method.)
    :type config: dict, optional
    :param scan_sub: The subscriber object for the LaserScan topic.
    :type scan_sub: rospy.Subscriber, optional
    """
    config = {}
    scan_sub = None

    def __init__(self):
        """Initializes the PatternDetectorROS object with the given configurations."""
        self.config = {
            "frame_id": rospy.get_param('~frame_id'),
            "end_frame_id": rospy.get_param('~end_frame_id'),
            "convert_to_end_frame_id": rospy.get_param('~convert_to_end_frame_id'),
            "scan_topic_name": rospy.get_param('~scan_topic_name'),
            "cluster": {
                "marker_topic_name": rospy.get_param('~cluster/marker_topic_name'),
                "marker_color": ColorRGBA(
                    rospy.get_param('~cluster/marker_color/r'),
                    rospy.get_param('~cluster/marker_color/g'),
                    rospy.get_param('~cluster/marker_color/b'),
                    rospy.get_param('~cluster/marker_color/a')
                ),
                "eps": rospy.get_param('~cluster/eps'),
                "min_samples": rospy.get_param('~cluster/min_samples'),
                "id": 0,
            },
            "subcluster": {
                "marker_topic_name": rospy.get_param('~subcluster/marker_topic_name'),
                "marker_color": ColorRGBA(
                    rospy.get_param('~subcluster/marker_color/r'),
                    rospy.get_param('~subcluster/marker_color/g'),
                    rospy.get_param('~subcluster/marker_color/b'),
                    rospy.get_param('~subcluster/marker_color/a')
                ),
                "eps": rospy.get_param('~subcluster/eps'),
                "min_samples": rospy.get_param('~subcluster/min_samples'),
                "id": 0,
            },
            "valid_clusters": {
                "marker_topic_name": rospy.get_param('~valid_clusters/marker_topic_name'),
                "marker_color": ColorRGBA(
                    rospy.get_param('~valid_clusters/marker_color/r'),
                    rospy.get_param('~valid_clusters/marker_color/g'),
                    rospy.get_param('~valid_clusters/marker_color/b'),
                    rospy.get_param('~valid_clusters/marker_color/a')
                ),
                "id": 0,
            },
            "parking_pose": {
                "marker_topic_name": rospy.get_param('~parking_pose/marker_topic_name'),
            },
            "marker_timelife": rospy.get_param('~marker_timelife'),
            "max_point_distance": rospy.get_param('~max_point_distance'),
            "scan_angle": {
                "min": rospy.get_param('~scan_angle/min'),
                "max": rospy.get_param('~scan_angle/max'),
            },
            "simple_pattern": {
                "pattern_width": rospy.get_param('~simple_pattern/pattern_width'),
                "pattern_offset": rospy.get_param('~simple_pattern/pattern_offset'),
                "required_subclusters": rospy.get_param('~simple_pattern/required_subclusters'),
            },
            "distance_from_centroid": rospy.get_param('~distance_from_centroid'),
            "pose_buffer": {
                "max_samples": rospy.get_param('~pose_buffer/max_samples'),
                "confirm_threshold": rospy.get_param('~pose_buffer/confirm_threshold'),
                "distance_threshold": rospy.get_param('~pose_buffer/distance_threshold'),
            }
        }

        self.rviz_visualizer = RVizVisualizer(self.config)
        self.processor = PointCloudProcessor(self.config)
        self.scan_sub = None
        self.result_pub = rospy.Publisher("/pattern_detector_ros/pose", PoseStamped or None, queue_size=10)
        if self.config["convert_to_end_frame_id"]:
            self.tf_buffer = Buffer()
            self.listener = TransformListener(self.tf_buffer)


    def start(self):
        """Starts the pattern detector by subscribing to the LaserScan topic."""
        if self.scan_sub is None:
            self.scan_sub = rospy.Subscriber(self.config['scan_topic_name'], LaserScan, self.scan_callback)

    def stop(self):
        """Stops the pattern detector by unsubscribing from the LaserScan topic."""
        if self.scan_sub is not None:
            self.scan_sub.unregister()
            self.scan_sub = None

    def scan_callback(self, msg: LaserScan) -> None:
        """Callback function for incoming LaserScan messages.

        :param msg: The incoming LaserScan message.
        :type msg: LaserScan
        """
        trans = None
        if self.config["convert_to_end_frame_id"]:
            trans = self.tf_buffer.lookup_transform(self.config['end_frame_id'],
                                                    self.config['frame_id'],
                                                    rospy.Time.now(),
                                                    rospy.Duration(1))

        clusters, subclusters, valid_clusters, detected_shape = self.processor.clustering(
            point_cloud=self.laser_scan_to_points(msg))

        print(detected_shape)

        if detected_shape is not None:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self.config['frame_id']
            pose_stamped.header.stamp = msg.header.stamp
            pose_stamped.pose = detected_shape
            if self.config["convert_to_end_frame_id"] and trans is not None:
                pose_stamped = do_transform_pose(pose_stamped, trans)
            self.result_pub.publish(pose_stamped)

        self.rviz_visualizer.publish_visualization(clusters, subclusters, valid_clusters, detected_shape)

    def laser_scan_to_points(self, msg: LaserScan) -> PointCloud:
        """Converts a LaserScan message into a PointCloud object.

        :param msg: The incoming LaserScan message.
        :type msg: LaserScan
        :return: The resulting PointCloud object.
        :rtype: PointCloud
        """
        data_points = PointCloud()
        for i in range(len(msg.ranges)):
            angle = msg.angle_min + i * msg.angle_increment
            if self.config['scan_angle']['min'] <= angle <= self.config['scan_angle']['max']:
                dist = msg.ranges[i]
                if dist < self.config['max_point_distance']:
                    x = dist * math.cos(angle)
                    y = dist * math.sin(angle)
                    if not math.isinf(x) and not math.isinf(y):
                        data_points.add_point([x, y])
        return data_points
