from dataclasses import dataclass


@dataclass
class Point:
    """This class represents a 3D point in Cartesian coordinates.

    :param x: The x coordinate of the point.
    :type x: float, default = 0
    :param y: The y coordinate of the point.
    :type y: float, default = 0
    :param z: The z coordinate of the point.
    :type z: float, default = 0
    """

    x: float = 0
    y: float = 0
    z: float = 0


@dataclass
class Quaternion:
    """This class represents a quaternion.

    :param x: The x component of the quaternion.
    :type x: float, default = 0
    :param y: The y component of the quaternion.
    :type y: float, default = 0
    :param z: The z component of the quaternion.
    :type z: float, default = 0
    :param w: The w component of the quaternion.
    :type w: float, default = 0
    """

    x: float = 0
    y: float = 0
    z: float = 0
    w: float = 0


@dataclass
class Pose:
    """This class represents a pose, including position and orientation.

    :param position: The position, represented as a Point object.
    :type position: Point, default = Point()
    :param orientation: The orientation, represented as a Quaternion object.
    :type orientation: Quaternion, default = Quaternion()
    """

    position: Point = Point()
    orientation: Quaternion = Quaternion()


class PointCloud:
    """This class represents a point cloud.

    :param points: An iterable of points to include in the point cloud.
    :type points: list, tuple, or None, default = None
    """

    def __init__(self, points=None):
        """Initializes the PointCloud object."""
        if points is None:
            self.points = []
        else:
            self.points = [self._to_list(point) for point in points]

    @staticmethod
    def _to_list(point):
        """Converts a point to a list if it is not already.

        :param point: The point to be converted.
        :type point: list or tuple
        :return: The point converted to a list.
        :rtype: list
        """
        if isinstance(point, list):
            return point
        elif isinstance(point, tuple):
            return list(point)
        else:
            raise TypeError("point must be a list or tuple")

    def add_point(self, point):
        """Adds a point to the point cloud.

        :param point: The point to be added.
        :type point: list or tuple
        """
        self.points.append(self._to_list(point))


    def get_points(self):
        """Returns the points in the point cloud.

        :return: The points in the point cloud.
        :rtype: list of list
        """
        return self.points

@dataclass
class Cluster:
    """This class represents a cluster of points.

    :param centroid: The centroid of the cluster, represented as a Pose object.
    :type centroid: PoseMy
    :param width: The width of the cluster.
    :type width: float
    :param height: The height of the cluster.
    :type height: float
    """

    centroid: Pose
    width: float
    height: float


@dataclass
class SubCluster:
    """This class represents a sub-cluster of points.

    :param centroid: The centroid of the sub-cluster, represented as a Pose object.
    :type centroid: PoseMy
    """

    centroid: Pose


@dataclass
class ValidCluster:
    """This class represents a valid cluster of points.

    :param centroid: The centroid of the valid cluster, represented as a Pose object.
    :type centroid: PoseMy
    """

    centroid: Pose


@dataclass
class DetectedShape:
    """This class represents a detected shape.

    :param position: The position of the detected shape, represented as a Pose object.
    :type position: PoseMy
    """

    position: Pose
