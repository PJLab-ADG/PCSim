# ReSimAD
```
└── src
    ├── __init__.py
    ├── config # Storing data files for emulator simulation
    │   ├── object_box_db.json # carla blueprint data files (individual vehicle and pedestrian blueprint names, bounding box size, etc.)
    │   ├── sequence_index.txt
    │   └── vec_track # Intermediate Data Files
    │       └── *.pkl
    ├── preprocess
    │   ├── __init__.py
    │   ├── create_tracklets_database.py # Generating Intermediate Data Files for Simulation Using Waymo data
    │   ├── trans_seg_to_sequence.py # Get the name of the mesh imported by the current carla
    │   └── waymo_converter.py # Generate kitti format datasets by converting acquired point cloud data with intermediate data files
    ├── scene_replay.py # WaymoSim simulation data generation pipeline
    ├── sensor_generator.py
    ├── walker_control.py
    └── waymo_sequence # Storing waymo raw data
        └── *.tfrecord
```

## Requirements

+ Carla (>=0.9.12)
+ PythonAPI for Carla
+ [requirements.txt](ReSimAD/requirements.txt)
  
## Prepare mesh for CARLA

Follow the instruction [Import a Large Map into CARLA](https://carla.readthedocs.io/en/latest/content_authoring_large_maps/#import-a-large-map-into-carla) to import reconstruct mesh file(.fbx)

## Build LiDAR
Following [LiDAR Simulation Library](../README.md/#lidar-simulation-library) to use build different LiDAR.

## Generate simulated data

### 1. Prepare intermediate pkl file

    Organize the raw Waymo files(*.tfrecord) in `waymo_sequence` folder. 

    Generate the data infos by running the following command:

    ```bash
    python preprocess/create_tracklets_database.py
    ```

### 2. Simulated in CARLA
    
    In [lidar_sensor python file](ReSimAD/src/sensor_generator.py), 
    - `local_save_dir` *line 53*: change the lidar data save path
    - `lidar_name` *line 58*: change different LiDAR, such as: "hdl64", "hdl32", "pandar_40p" ...
    - `lidar_transform` *line 62*: change the attachment location of LiDAR.

    Generate Lidar data infos by running the following command:
    ```bash
    python scene_replay.py --waymo_sequence segment-9320169289978396279_1040_000_1060_000
    ```

### 3. Convert lidar data to Kitti like using the annotations from intermediate file.
    ```bash
    python preprocess/waymo_converter.py
    ```


The demo fbx and pkl file can be access from [google drive](https://drive.google.com/drive/folders/1r6_4OlHrg_mXQK7PHKcOXEIF5n3CFNyH?usp=sharing)