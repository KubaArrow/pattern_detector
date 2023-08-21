from tf import TransformListener

from .structures import Pose
from .pose_detected import PoseDetected


class PoseBuffer:
    """
    The PoseBuffer class handles a buffer of detected poses and
    implements methods for managing and processing these poses.
    """

    def __init__(self, distance_threshold=0.05, confirm_threshold=8, max_samples=10):
        """
        Initializes the PoseBuffer class.

        :param distance_threshold: float representing the distance threshold for pose confirmation, defaults to 0.05
        :param confirm_threshold: int representing the confirmation threshold for pose confirmation, defaults to 8
        :param max_samples: int representing the maximum number of samples to store in the buffer, defaults to 10
        """
        self.max_samples = max_samples
        self.confirm_threshold = confirm_threshold
        self.distance_threshold = distance_threshold
        self.buffer = []  # buffer storing the PoseDetected objects
        self.counter = 0  # counter used for clearing the buffer
        self.tf_listener = TransformListener()  # transform listener object for handling pose transformations

    def checks_points(self, pose: Pose) -> Pose or None:
        """
        Adds a new pose to the buffer, checks existing poses for confirmation,
        clears the buffer if necessary, and calculates the average confirmed pose.

        :param pose: Pose object representing the new pose
        :return: PoseMy object representing the average confirmed pose or None if no confirmed poses
        """
        if self.counter >= self.max_samples * 100:
            self.clear_buffer()
            self.counter = 0
        new_point = True
        for buffered_point in self.buffer:
            if not buffered_point.add_pose(pose) and new_point:
                new_point = True
            if buffered_point.check_pose():
                self.buffer.clear()
                avg = buffered_point.average_confirmed_pose()
                buffered_point.buffer.clear()
                return avg

        if new_point:
            self.buffer.append(PoseDetected(pose, self.distance_threshold, self.confirm_threshold, self.max_samples))
        self.counter += 1
        return None

    def clear_buffer(self):
        """
        Clears the buffer by removing poses that need to be deleted according to the 'to_delete' function.
        """
        for i, buffered_point in enumerate(self.buffer):
            if buffered_point.to_delete():
                del self.buffer[i]
