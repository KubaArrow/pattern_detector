from .structures import Pose, Quaternion
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class PoseDetected:
    """
    Class PoseDetected stores information about a detected pose.
    The class also contains functions for updating, checking, and processing the stored data.
    """

    def __init__(self, pose: Pose, distance_threshold: float, confirm_threshold: int, max_samples: int):
        """
        Initializes the PoseDetected class.

        :param pose: Pose object representing the detected pose
        :param distance_threshold: float representing the distance threshold for pose confirmation
        :param confirm_threshold: int representing the confirmation threshold for pose confirmation
        :param max_samples: int representing the maximum number of samples to store in the buffer
        """
        self.pose = pose
        self.max_samples = max_samples
        self.distance_threshold = distance_threshold
        self.confirm_threshold = confirm_threshold
        self.buffer = []  # buffer storing the samples

    def add_pose(self, pose: Pose) -> bool:
        """
        Adds a new pose to the buffer. If the buffer is full, it removes the oldest pose.
        It also checks whether the new pose is close to the last recorded pose.

        :param pose: Pose object representing the new pose to be added
        :return: bool indicating whether the new pose is not close to the last recorded pose
        """
        if len(self.buffer) >= self.max_samples:
            self.buffer.pop(0)
        is_close = self.is_close(pose)
        self.buffer.append((pose, is_close))
        return not is_close

    def check_pose(self) -> bool:
        """
        Checks whether the sum of confirmations for recorded poses exceeds the confirmation threshold.

        :return: bool indicating whether the confirmation threshold is exceeded
        """
        return sum(confirm for _, confirm in self.buffer) >= self.confirm_threshold

    def to_delete(self) -> bool:
        """
        Checks whether the sum of confirmations for recorded poses is below half of the confirmation threshold.

        :return: bool indicating whether the pose should be deleted
        """
        return sum(confirm for _, confirm in self.buffer) < self.confirm_threshold / 2

    def average_confirmed_pose(self) -> Pose:
        """
        Calculates the average of confirmed poses. Converts between Euler angles and quaternions
        for the proper calculation of average orientation.

        :return: Pose object representing the average of confirmed poses or None if no confirmed poses
        """
        confirmed_poses = [pose for pose, confirm in self.buffer if confirm]
        if not confirmed_poses:
            return None

        avg_x = sum(pose.position.x for pose in confirmed_poses) / len(confirmed_poses)
        avg_y = sum(pose.position.y for pose in confirmed_poses) / len(confirmed_poses)

        euler_angles = [
            euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]) for
            pose in confirmed_poses]

        avg_euler = [sum(angle) / len(angle) for angle in zip(*euler_angles)]

        avg_quaternion = quaternion_from_euler(*avg_euler)

        pose = Pose()
        pose.position.x = avg_x
        pose.position.y = avg_y
        pose.orientation = Quaternion(*avg_quaternion)
        return pose

    def is_close(self, pose2: Pose) -> bool:
        """
        Checks whether a pose is close to the last recorded pose by comparing
        the x and y values with the distance threshold.

        :param pose2: Pose object representing the pose to be checked
        :return: bool indicating whether the pose is close to the last recorded pose
        """
        return abs(self.pose.position.x - pose2.position.x) < self.distance_threshold and abs(
            self.pose.position.y - pose2.position.y) < self.distance_threshold
