#pragma once

#include "Carla/Sensor/LidarDescription.h"

FLidarDescription create_mechanical(const std::string& lidar_name);

//1.pandar64
FLidarDescription create_pandar64();

//2.ruby128
FLidarDescription create_ruby128();

//3.pandar128
FLidarDescription create_pandar128();

//4.vlp16
FLidarDescription create_vlp16();

//5.hdl64
FLidarDescription create_hdl64();

//6.rs_m1
FLidarDescription create_rs_m1();

//7.pandar_qt
FLidarDescription create_pandar_qt();

//8.bpearl
FLidarDescription create_bpearl();

//9.pandar_40m
FLidarDescription create_pandar_40m();

//10.pandar_40p
FLidarDescription create_pandar_40p();

//11.pandar_xt
FLidarDescription create_pandar_xt();

//12.vlp32
FLidarDescription create_vlp32();

//13. os1_64gen2
FLidarDescription create_os1_64_gen2();