# Pattern Detector

The `pattern_detector` package is designed for detecting and visualizing patterns in point clouds collected from a Lidar sensor. The package utilizes the Robot Operating System (ROS) for handling sensor data and visualization.

## Overview

- **Structures**: This part of the package includes classes that define the primary data structures used by the detector, such as `Point`, `Quaternion`, `Pose`, `PointCloud`, `Cluster`, `SubCluster`, `ValidCluster`, and `DetectedShape`.

- **PointCloudProcessor**: This is a helper class that takes in the raw point cloud data and applies various processing steps. This includes clustering, subclustering, filtering, and finally detecting shapes within the point cloud.

- **RVizVisualizer**: This class is responsible for visualizing the detection results in RViz. It publishes markers for clusters, subclusters, valid clusters, and the detected shape.

- **PatternDetectorROS**: This is the primary class which orchestrates the entire detection process. It subscribes to the Lidar scan data, processes it, visualizes the results, and publishes the detected shape.

## Configuration

The behavior of the pattern detector can be adjusted via a set of ROS parameters. Here's an overview:

- `frame_id`: Frame id to be used by the detector.
- `scan_topic_name`: Name of the topic from which scan data is subscribed.
- `cluster`, `subcluster`, `valid_clusters`: These sections contain parameters for adjusting the clustering and visualization process such as `marker_topic_name`, `marker_color`, `eps`, `min_samples`, and `id`.
- `parking_pose`: Topic name for publishing the parking pose.
- `marker_timelife`: Lifetime of the markers in RViz.
- `max_point_distance`: Maximum distance from the sensor for points to be included in the point cloud.
- `scan_angle`: The minimum and maximum angles to be included in the scan.
- `wall_filter`: Parameters for RANSAC wall filtering.
- `simple_pattern`: Parameters to define the pattern for detection, such as `pattern_width`, `pattern_offset`, and `required_subclusters`.
- `distance_from_centroid`: The maximum distance from the centroid for a point to be considered as part of a cluster.
- `pose_buffer`: Parameters for pose estimation, including `max_samples`, `confirm_threshold`, and `distance_threshold`.

## Usage

Once you have adjusted the parameters to suit your setup, you can start the pattern detector with the following command:

```bash
roslaunch pattern_detector pattern_detector.launch
```

You can then visualize the results in RViz by subscribing to the relevant marker topics.

## Note

Please adjust the parameters according to your sensor setup and environment for optimal results. This package is tested with ROS Noetic and Python 3.8.