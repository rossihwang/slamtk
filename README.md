# slamtk
Toolkits for SLAM

## dataset player

Download the datasets
- http://ais.informatik.uni-freiburg.de/slamevaluation/datasets.php
- http://www2.informatik.uni-freiburg.de/~stachnis/datasets.html

Currently only support **old format** as described [here](http://carmen.sourceforge.net/logger_playback.html) 

Play the dataset in ROS2 with below command, it will pulish the tf(odom, base_footprint, base_scan) and /scan topic

```shell
ros2 run dataset_player dataset_player_node --ros-args -p dataset:=slam_datasets/ACES\ Building/aces.clf
```

You can play with slam_toolbox to build the grid map.

```shell
ros2 launch slam_toolbox online_sync_hybrid_launch.py use_sim_time:=True
```

