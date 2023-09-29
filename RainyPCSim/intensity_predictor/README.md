# Weather-based intensity predictor

## Installation
```
conda create -n intensity_predictor python=3.8
pip install -r RainyPCSim/intensity_predictor/python/requirements.txt
```

## Prepare Dataset
Please download the official [Waymo Open Dataset](https://waymo.com/open/download/), and convert the raw data to multi-channel range image data stored as npy file.

```python
# multi-channel range image data
# 64x2650x14
# hwc
channels = ['depth', 'x-coor', 'y-coor', 'z-coor', 'intensity', 'intensity_bin', 'intensity_dist','label', 'red', 'green', 'Blue, color_mask, returned_ray_mask', 'weather']
# label is the semantic label of lidar.
# red, green, blue are the rgb value of the LIDAR point cloud projected onto the corresponding pixel of the camera.
# color mask indicates whether the point cloud can be projected to the camera.
# weather tagged sunny(0), wet ground(1), light rain(2), heavy rain(3).
```

## Configs

Folder [configs](RainyPCSim/intensity_predictor/configs) contains configs for starting either `python/model_eval.py` or `python/model_train.py`.

You may change [model_l2_v2_1376.yml](RainyPCSim/intensity_predictor/configs/includes/reflect_configs/model_l2_v2_1376.yml) and [model_l2_weather_v2_1376.yml](RainyPCSim/intensity_predictor/configs/includes/reflect_configs/model_l2_weather_v2_1376.yml), which contains paths for data.

## Usage

```bash
python -u model_run.py configs/train.reflect-l2.depth.rgb.label.model_l2_weather_v2_1376.yml
python -u model_eval.py configs/train.reflect-l2.depth.rgb.label.model_l2_weather_v2_1376.yml
```

## Acknowledge
* Our code is based on [lidar-intensity](https://github.com/ctu-vras/lidar-intensity). Thanks for their codebase.
