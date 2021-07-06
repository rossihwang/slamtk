# slamtk
Toolkits for SLAM

## dataset player

Download the datasets [here](http://ais.informatik.uni-freiburg.de/slamevaluation/datasets.php)

Play the dataset in ROS2 with below command, it will pulish the tf(odom, base_footprint, base_scan) and /scan topic

```shell
ros2 run dataset_player dataset_player_node --ros-args -p dataset:=slam_datasets/ACES\ Building/aces.clf
```

You can play with slam_toolbox to build the grid map.


## TODO
- Play dataset according to timestamp