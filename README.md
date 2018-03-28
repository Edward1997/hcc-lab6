# hcc-lab6

### 1. Preparation
$```mkdir catkin_ws/src```

$ ```cd catkin_ws/src```

git clone this repo (put the localization folder into catkin_ws/src)

download the pcd file here [here]()

### 2. Edit

* specify pcd file name
* done the ICP part ([reference](http://pointclouds.org/documentation/tutorials/iterative_closest_point.php))

### 3. Build
(under catkin_ws)

$ ```catkin_make```

$ ```source devel/setup.bash```

### 4. Run

$ ```roscore```

$ ``` rosrun localization localization_node``` run the node

$ ```rviz```   to see the result
