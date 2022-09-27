#pragma once

#include "Carla/Sensor/LidarDescription.h"
std::vector<std::vector<float>> livox_csv_reader(const std::string& lidar_name); //zwq, read livox csv
FLidarDescription create_livox(const std::string& lidar_name);