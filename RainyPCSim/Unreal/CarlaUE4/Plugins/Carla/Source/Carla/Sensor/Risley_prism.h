#pragma once

#include "Carla/Sensor/LidarDescription.h"
std::vector<std::vector<float>> Risley_prism_csv_reader(const std::string& lidar_name); //read Risley_prism csv
FLidarDescription create_Risley_prism(const std::string& lidar_name);