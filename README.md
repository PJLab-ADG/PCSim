# LiDARSimLib and Placement Evaluation



https://user-images.githubusercontent.com/21357317/192964768-1b8711a5-742b-4b5b-bf59-c318e2e5c464.mp4



## LiDAR Simulation Library

### Requirements
1.Carla (>=0.9.12)

### Usage
1.There are two paths should be modified:

```
LiDARSimLib/Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor/livox.cpp
```

line9: modify depend on your own carla path

```
LiDARSimLib/PythonAPI/lidar_lib_example/lidar_display.py
```

line30: modify depend on your own carla path

2.rebuild carla

Run
```
make clean

make PythonAPI

make launch
```

### Results
Table for all LiDARs simulation results

![](pic/LiDARSimLib.png)

## LiDAR Placement Evaluation in V2X Scenario
