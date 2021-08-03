# slamtk
Toolkits for SLAM



## Evaluating the SLAM algorithm

### Tools

- dataset player: parsing the CARMEN dataset, and publishing the tf(odom, base_footprint, base_scan) , /scan and /clock

- odom logger: storing the ROS2 estimated poses into CARMEN format(for evaluation)

- metric evaluator: download [here](http://ais.informatik.uni-freiburg.de/slamevaluation/software.php)



### Usages

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
  ros2 run slamtk odom_logger_node --ros-args -p use_sim_time:=True
  ros2 run dataset_player dataset_player_node --ros-args -p dataset:=slam_datasets/ACES\ Building/aces.clf
  ```

- Finally run the metric evaluator(use help for more details)

  ```
  ./metricEvaluator -s relations_from_odom_logger.txt -r ./ACES_Building/aces.clf -w "{1.0, 1.0, 1.0, 0.0, 0.0, 0.0}"
  ```



## Other resources

- Datasets
  - http://ais.informatik.uni-freiburg.de/slamevaluation/datasets.php
  - http://www2.informatik.uni-freiburg.de/~stachnis/datasets.html
