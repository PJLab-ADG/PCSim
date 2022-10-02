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

We build our evaluation code upon [v2x-vit](https://github.com/DerrickXuNu/v2x-vit) project.

### Usage

1. Follow the instruction in the readme page in [v2x-vit](https://github.com/DerrickXuNu/v2x-vit) and install the v2x-vit project.
2. Download the pretrained model from [here](https://drive.google.com/drive/folders/1r5sPiBEvo8Xby-nMaWUTnJIPK6WhY1B6?usp=sharing).
3. Use the script in Placement-Evaluation/v2xvit/tools/inference.py for evaluation.