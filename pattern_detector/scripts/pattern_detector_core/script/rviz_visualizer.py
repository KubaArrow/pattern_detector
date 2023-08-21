import rospy
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray


class RVizVisualizer:
    """
    The RVizVisualizer class creates and publishes visualization markers for RViz.
    """

    def __init__(self, config):
        """
        Initializes the RVizVisualizer class.

        :param config: dict representing the configuration for the visualizer
        """
        self.config = config
        self.cluster_marker_pub = rospy.Publisher(self.config['cluster']['marker_topic_name'], MarkerArray, queue_size=10)
        self.subcluster_marker_pub = rospy.Publisher(self.config['subcluster']['marker_topic_name'], MarkerArray, queue_size=10)
        self.valid_clusters_marker_pub = rospy.Publisher(self.config['valid_clusters']['marker_topic_name'], MarkerArray, queue_size=10)
        self.parking_pose_pub = rospy.Publisher(self.config['parking_pose']['marker_topic_name'], PoseStamped, queue_size=10)

    def publish_visualization(self, clusters, subclusters, valid_clusters, detected_shape):
        """
        Publishes visualization markers for clusters, subclusters, valid clusters and detected shapes.

        :param clusters: list of cluster entities
        :param subclusters: list of subcluster entities
        :param valid_clusters: list of valid cluster entities
        :param detected_shape: geometry_msgs/Pose representing the detected shape
        """
        cluster_markers = self.create_cluster_markers(clusters, self.config['cluster']['marker_color'], self.config['cluster']['id'])
        self.cluster_marker_pub.publish(MarkerArray(markers=cluster_markers))

        subcluster_markers = self.create_pose_markers(subclusters, self.config['subcluster']['marker_color'], self.config['subcluster']['id'])
        self.subcluster_marker_pub.publish(MarkerArray(markers=subcluster_markers))

        valid_cluster_markers = self.create_pose_markers(valid_clusters, self.config['valid_clusters']['marker_color'], self.config['valid_clusters']['id'])
        self.valid_clusters_marker_pub.publish(MarkerArray(markers=valid_cluster_markers))
        if detected_shape is not None:
            self.publish_shape_pose(detected_shape)

    def create_cluster_markers(self, clusters, color, id):
        """
        Creates markers for the given clusters.

        :param clusters: list of cluster entities
        :param color: color for the markers
        :param id: unique identifier for the markers
        :return: list of created markers
        """
        markers = []
        for i, cluster in enumerate(clusters):
            marker = Marker()
            marker.header.frame_id = self.config['frame_id']
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.id = i + id
            marker.ns = "clusters"
            marker.pose = cluster.centroid
            marker.scale.x = cluster.width
            marker.scale.y = cluster.height
            marker.scale.z = 0.2
            marker.color = color
            marker.lifetime = rospy.Duration.from_sec(self.config['marker_timelife'])
            markers.append(marker)
        return markers

    def create_pose_markers(self, entities, color, id):
        """
        Creates markers for the given entities.

        :param entities: list of entities
        :param color: color for the markers
        :param id: unique identifier for the markers
        :return: list of created markers
        """
        markers = []
        for i, entity in enumerate(entities):
            marker = Marker()
            marker.header.frame_id = self.config['frame_id']
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.id = i + id
            marker.ns = "entities"
            marker.pose = entity.centroid
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.2
            marker.color = color
            marker.lifetime = rospy.Duration.from_sec(self.config['marker_timelife'])
            markers.append(marker)
        return markers

    def publish_shape_pose(self, shape):
        """
        Publishes a shape's pose to a designated topic.

        :param shape: geometry_msgs/Pose representing the shape
        """
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.config['frame_id']
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose = shape

        self.parking_pose_pub.publish(pose_stamped)
