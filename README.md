# slamtk
Toolkits for SLAM



## Evaluating the SLAM algorithm

### Tools

- dataset player: parsing the CARMEN dataset, and publishing the tf(odom, base_footprint, base_scan) , /scan and /clock

- odom logger: storing the ROS2 estimated poses into CARMEN format(for evaluation)

- metric evaluator: download [here](http://ais.informatik.uni-freiburg.de/slamevaluation/software.php)

- grid_mapper: building grid map with given scans and correspondent poses

  

### Usages

#### Playing dataset and recording the odometry data for benchmark

In this example, I will walk you through how to evaluate the slam_toolbox with these tools.

- Build the map first, run this commands in separated terminals

  ```shell
  ros2 launch slam_toolbox online_sync_launch.py use_sim_time:=True
  ros2 run dataset_player dataset_player_node --ros-args -p dataset:=./ACES_Building/aces.clf
  rviz2  # (optional)
  ```

- As ["On Measuring the Accuracy of SLAM Algorithms"](http://www2.informatik.uni-freiburg.de/~stachnis/pdf/kuemmerle09auro.pdf) section 6 suggests, to benchmark the algorithm without trajectory estimates, we can simply play the dataset again and run the localization on the built map to recover the trajectory.

- Run these commands in separated terminals

  ```
  ros2 launch nav2_bringup localization_launch.py use_sim_time:=True map:=MAP_YAML_FILE
  ros2 run odom_logger odom_logger_node --ros-args -p use_sim_time:=True
  ros2 run dataset_player dataset_player_node --ros-args -p dataset:=slam_datasets/ACES\ Building/aces.clf
  ```

- Finally run the metric evaluator(use help for more details)

  ```
  ./metricEvaluator -s PATH/TO/estimated_odom.txt -r PATH/TO/relations -w "{1.0, 1.0, 1.0, 0.0, 0.0, 0.0}"
  ```

#### Building map with grid_mapper

Send a static tf of map and odom

```
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```

then run the dataset_player and grid_mapper



## Benchmark

For slam_toolbox

| Dataset               | laser maximum range(meter) | mean | std  |
| --------------------- | -------------------------- | ---- | ---- |
| 2 MIT Killian Court   |                            |      |      |
| ACES Building         | 50                         |      |      |
| Freiburg Building 101 |                            |      |      |
| Interl Research Lab   |                            |      |      |
| MIT CSAIL Building    |                            |      |      |



## Other resources

- Datasets
  - http://ais.informatik.uni-freiburg.de/slamevaluation/datasets.php
  - http://www2.informatik.uni-freiburg.de/~stachnis/datasets.html
