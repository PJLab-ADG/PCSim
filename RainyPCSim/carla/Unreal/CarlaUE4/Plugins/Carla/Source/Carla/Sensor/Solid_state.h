#pragma once

#include "Carla/Sensor/LidarDescription.h"
FLidarDescription create_Solid_state(const std::string& lidar_name); //chose lidar by name
FLidarDescription create_rs_m1(); //return rs_m1's description