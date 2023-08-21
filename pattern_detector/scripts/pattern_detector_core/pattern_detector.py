from script.point_cloud_processor import PointCloudProcessor


class PatternDetector:
    """This class represents a pattern detector.

    The class processes incoming scans to detect patterns.

    :param config: A dictionary of configurations for the pattern detector.
        (Optional: defaults are defined in the `__init__` method.)
    :type config: dict, optional
    """
    def __init__(self,
                 max_point_distance=None,
                 scan_angle_min=None,
                 scan_angle_max=None,
                 wall_filter_work=None,
                 wall_filter_ransac_epsilon=None,
                 wall_filter_distance_threshold=None,
                 simple_pattern_pattern_width=None,
                 simple_pattern_pattern_offset=None,
                 simple_pattern_required_subclusters=None,
                 distance_from_centroid=None,
                 pose_buffer_max_samples=None,
                 pose_buffer_confirm_threshold=None,
                 pose_buffer_distance_threshold=None,
                 cluster_eps=None,
                 cluster_min_samples=None,
                 cluster_id=None,
                 subcluster_eps=None,
                 subcluster_min_samples=None,
                 subcluster_id=None):

        """Initializes the PatternDetector object with the given configurations."""
        self.config = {
            "max_point_distance": max_point_distance if max_point_distance else 3,
            "scan_angle": {
                "min": scan_angle_min if scan_angle_min else -0.785,
                "max": scan_angle_max if scan_angle_max else 0.785,
            },
            "wall_filter": {
                "work": wall_filter_work if wall_filter_work else 0,
                "ransac_epsilon": wall_filter_ransac_epsilon if wall_filter_ransac_epsilon else 0.1,
                "distance_threshold": wall_filter_distance_threshold if wall_filter_distance_threshold else 0.2,
            },
            "simple_pattern": {
                "pattern_width": simple_pattern_pattern_width if simple_pattern_pattern_width else 0.3,
                "pattern_offset": simple_pattern_pattern_offset if simple_pattern_pattern_offset else 0.05,
                "required_subclusters": simple_pattern_required_subclusters if simple_pattern_required_subclusters else 3,
            },
            "distance_from_centroid": distance_from_centroid if distance_from_centroid else 0.3,
            "pose_buffer": {
                "max_samples": pose_buffer_max_samples if pose_buffer_max_samples else 5,
                "confirm_threshold": pose_buffer_confirm_threshold if pose_buffer_confirm_threshold else 3,
                "distance_threshold": pose_buffer_distance_threshold if pose_buffer_distance_threshold else 0.1,
            },
            "cluster": {
                "eps": cluster_eps if cluster_eps else 0.07,
                "min_samples": cluster_min_samples if cluster_min_samples else 15,
                "id": cluster_id if cluster_id else 1,
            },
            "subcluster": {
                "eps": subcluster_eps if subcluster_eps else 0.02,
                "min_samples": subcluster_min_samples if subcluster_min_samples else 5,
                "id": subcluster_id if subcluster_id else 2,
            },
        }

        self.processor = PointCloudProcessor(self.config)

    def process_scan(self, point_cloud):
        """Process a scan provided as a PointCloud.

        :param point_cloud: The points in the form of a PointCloud
        :type point_cloud: PointCloud
        """
        clusters, subclusters, valid_clusters, detected_shape = self.processor.clustering(point_cloud)
        return clusters, subclusters, valid_clusters, detected_shape
