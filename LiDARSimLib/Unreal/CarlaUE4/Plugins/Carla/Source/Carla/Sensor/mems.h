#pragma once

#include "Carla/Sensor/LidarDescription.h"
FLidarDescription create_mems(const std::string& lidar_name); //chose lidar by name
FLidarDescription create_mems_m1(); //return mems_m1's description