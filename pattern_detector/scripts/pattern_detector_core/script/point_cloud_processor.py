from typing import Tuple, List

import numpy as np

from scipy.spatial.distance import pdist, squareform

from sklearn.cluster import DBSCAN

from tf.transformations import quaternion_from_euler

from .pose_buffer import PoseBuffer
from .structures import PointCloud, Cluster, SubCluster, Quaternion, ValidCluster, Point, Pose


class PointCloudProcessor:
    """This class handles the processing of point clouds for pattern detection.

    It offers functionalities for clustering and sub-clustering, filtering points based
    on RANSAC and checking simple patterns.

    :param config: Configuration dictionary containing the parameters to be used by the processor.
    :type config: dict
    """

    cluster_eps = 0.25
    cluster_min_samples = 12
    cluster_id = 0

    subcluster_eps = 0.04
    subcluster_min_samples = 3
    sub_cluster_id = 0

    pattern_width = 0.6
    pattern_offset = 0.1
    required_subclusters = 3
    distance_from_centroid = 0.3

    def __init__(self, config):
        """Initializes the PointCloudProcessor object."""
        self.point_buffer = PoseBuffer(confirm_threshold=config['pose_buffer']['confirm_threshold'],
                                       distance_threshold=config['pose_buffer']['distance_threshold'],
                                       max_samples=config['pose_buffer']['max_samples'])

        self.cluster_eps = config['cluster']['eps']
        self.cluster_min_samples = config['cluster']['min_samples']
        self.cluster_id = config['cluster']['id']
        self.subcluster_eps = config['subcluster']['eps']
        self.subcluster_min_samples = config['subcluster']['min_samples']
        self.sub_cluster_id = config['subcluster']['id']
        self.pattern_width = config['simple_pattern']['pattern_width']
        self.pattern_offset = config['simple_pattern']['pattern_offset']
        self.required_subclusters = config['simple_pattern']['required_subclusters']
        self.distance_from_centroid = config['distance_from_centroid']
        self.max_samples = config['pose_buffer']['max_samples']
        self.frame = config['frame_id']

    def clustering(self, point_cloud: PointCloud) -> Tuple[
        List[Cluster], List[SubCluster], List[ValidCluster], Pose or None]:
        """Performs clustering on the input point cloud.

        :param point_cloud: The input point cloud to be processed.
        :type point_cloud: PointCloud
        :return: A tuple of list of clusters, list of subclusters, list of valid clusters, and detected shapes.
        :rtype: Tuple[List[Cluster], List[SubCluster], List[ValidCluster], PoseMy]
        """

        points_list = point_cloud.get_points()

        db = DBSCAN(eps=self.cluster_eps, min_samples=self.cluster_min_samples).fit(points_list)
        cluster_labels = db.labels_
        unique_labels = set(cluster_labels)

        clusters = []
        subclusters = []
        valid_clusters = []
        detected_shapes = None
        for label in unique_labels:
            if label == -1:
                continue
            cluster_points = np.array(points_list)[cluster_labels == label]

            if cluster_points.size > 0:
                centroid = np.mean(cluster_points, axis=0)
                pose_centroid = Pose(Point(centroid[0], centroid[1], 0), Quaternion())
                max_values = cluster_points.max(axis=0)
                min_values = cluster_points.min(axis=0)
                scale_values = max_values - min_values
                width = scale_values[0]
                height = scale_values[1]
                cluster = Cluster(pose_centroid, width, height)
                clusters.append(cluster)

                subcluster_points = self.sub_clustering(cluster_points=cluster_points)
                subclusters.extend(subcluster_points)
                pose_list = [subcluster.centroid for subcluster in subcluster_points]
                pose_array = np.array([[pose.position.x, pose.position.y]
                                       for pose in pose_list])
                if subcluster_points and self.simple_checking_pattern(len(subcluster_points), pose_array):
                    pose = self.get_pose_near_centroid_from_cluster(pose_array)
                    valid_clusters.append(ValidCluster(centroid=pose))
                    confirm_pose = self.point_buffer.checks_points(pose)
                    if confirm_pose is not None:
                        detected_shapes = confirm_pose

        return clusters, subclusters, valid_clusters, detected_shapes

    def sub_clustering(self, cluster_points) -> List[SubCluster]:
        """Performs sub-clustering on the provided cluster points.

        :param cluster_points: List of points forming a cluster.
        :type cluster_points: List[Point]
        :return: List of subclusters obtained.
        :rtype: List[SubCluster]
        """
        db_sub = DBSCAN(eps=self.subcluster_eps, min_samples=self.subcluster_min_samples).fit(cluster_points)
        sub_cluster_labels = db_sub.labels_
        unique_sub_labels = set(sub_cluster_labels)

        subclusters = []
        for sub_label in unique_sub_labels:
            if sub_label == -1:
                continue
            sub_cluster_points = np.array(cluster_points)[sub_cluster_labels == sub_label]
            sub_centroid = np.mean(sub_cluster_points, axis=0)
            subcluster = SubCluster(Pose(Point(sub_centroid[0], sub_centroid[1], 0), Quaternion()))
            subclusters.append(subcluster)
        return subclusters

    def simple_checking_pattern(self, subclusters_count: int, cluster_points) -> bool:
        """Checks if the simple pattern condition is satisfied.

        :param subclusters_count: The number of subclusters.
        :type subclusters_count: int
        :param cluster_points: The points in the cluster.
        :type cluster_points: numpy.ndarray
        :return: True if the simple pattern condition is satisfied, False otherwise.
        :rtype: bool
        """

        if subclusters_count == self.required_subclusters:
            max_values = cluster_points.max(axis=0)
            min_values = cluster_points.min(axis=0)
            cluster_width = np.linalg.norm(max_values - min_values)
            if self.pattern_width + self.pattern_offset > cluster_width > self.pattern_width - self.pattern_offset:
                return True
            else:
                return False
        else:
            return False

    @staticmethod
    def to_polar_coords(points):
        """Converts Cartesian coordinates to polar coordinates.

        :param points: List of points in Cartesian coordinates.
        :type points: List[Point]
        :return: List of points in polar coordinates.
        :rtype: List[Tuple[float, float]]
        """
        polar_points = []
        for point in points:
            rho = np.sqrt(point.x ** 2 + point.y ** 2)
            phi = np.arctan2(point.y, point.x)
            polar_points.append((rho, phi))
        return polar_points

    def get_pose_near_centroid_from_cluster(self, cluster_points: np.array) -> Pose:
        """Finds the pose near the centroid of the cluster points.

        :param cluster_points: The points in the cluster.
        :type cluster_points: numpy.ndarray
        :return: The pose near the centroid of the cluster.
        :rtype: Pose
        """

        distances = pdist(cluster_points)
        dist_matrix = squareform(distances)
        i, j = np.unravel_index(dist_matrix.argmax(), dist_matrix.shape)

        furthest_points = [cluster_points[i], cluster_points[j]]
        cluster_points_list = np.array([point for point in cluster_points if
                                        not np.array_equal(point, furthest_points[0])
                                        and not np.array_equal(point, furthest_points[1])])

        centroid = cluster_points_list[0]

        cluster_width_center = (furthest_points[0] + furthest_points[1]) / 2
        furthest_points_vector = furthest_points[1] - furthest_points[0]


        perpendicular_vector = np.array([-furthest_points_vector[1], furthest_points_vector[0]])
        normalized_perpendicular_vector = perpendicular_vector / np.linalg.norm(perpendicular_vector)

        side = np.dot(centroid, normalized_perpendicular_vector)
        if side > 0:
            point_of_interest = cluster_width_center - self.distance_from_centroid * normalized_perpendicular_vector
        else:
            point_of_interest = cluster_width_center + self.distance_from_centroid * normalized_perpendicular_vector

        angle = np.arctan2(perpendicular_vector[1], perpendicular_vector[0])

        angle = np.fmod(angle + 2 * np.pi, 2 * np.pi)

        quat = quaternion_from_euler(0, 0, angle)
        pose = Pose()
        pose.position = Point(x=point_of_interest[0], y=point_of_interest[1], z=0)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        return pose

    def filter_noise(self, points_list):
        """Filters noise using DBSCAN with subclustering parameters.

        :param points_list: List of points to filter.
        :type points_list: List[Point]
        :return: Filtered list of points.
        :rtype: List[Point]
        """
        points_array = np.array(points_list)
        db = DBSCAN(eps=0.04, min_samples=10).fit(points_array)
        labels = db.labels_
        no_noise_indices = np.where(labels != -1)[0]
        return [points_list[i] for i in no_noise_indices]

    def get_point_near_centroid_from_centroid(self, centroid) -> Point:
        """Finds the point near the centroid of the cluster points.

        :param centroid: The centroid point of the cluster.
        :type centroid: numpy.ndarray
        :return: The point near the centroid of the cluster.
        :rtype: Point
        """
        direction_vector = centroid / np.linalg.norm(centroid)
        point_of_interest = centroid - self.distance_from_centroid * direction_vector
        return Point(x=point_of_interest[0], y=point_of_interest[1], z=0)
