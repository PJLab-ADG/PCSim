#include "Carla/Sensor/Surround.h"

FLidarDescription create_Surround(const std::string& lidar_name)
{
	std::cout<<"create "<<lidar_name<<" successfully"<<std::endl;

	if (lidar_name == "pandar64")
	{
		return create_pandar64();
	}
	else if (lidar_name == "ruby128")
	{
		return create_ruby128();
	}
	else if (lidar_name == "pandar128")
	{
		return create_pandar128();
	}
	else if (lidar_name == "vlp16")
	{
		return create_vlp16();
	}
	else if (lidar_name == "hdl64")
	{
		return create_hdl64();
	}
	else if (lidar_name == "pandar_qt")
	{
		return create_pandar_qt();
	}
	else if (lidar_name == "bpearl")
	{
		return create_bpearl();
	}
	else if (lidar_name == "pandar_40m")
	{
		return create_pandar_40m();
	}
	else if (lidar_name == "pandar_40p")
	{
		return create_pandar_40p();
	}
	else if (lidar_name == "pandar_xt")
	{
		return create_pandar_xt();
	}
	else if (lidar_name == "vlp32")
	{
		return create_vlp32();
	}
	else if (lidar_name == "os1_64_gen2")
	{
		return create_os1_64_gen2();
	}
	return FLidarDescription(); //return a default description
}

//1.pandar64
FLidarDescription create_pandar64()
{
	FLidarDescription pandar64;
	pandar64.LidarType = "Surround";
	pandar64.NAME = "pandar64";
	pandar64.Channels = 64;
	pandar64.Range = 200;
	pandar64.HorizontalFov = 360.0f;
	pandar64.UpperFovLimit = 15;
	pandar64.LowerFovLimit = -25;
	pandar64.PointsPerSecond = 1152000;
	pandar64.RotationFrequency = 10.0f;

	pandar64.NoiseStdDev = 0;
	pandar64.DropOffGenRate = 0.18;

	pandar64.vfov = {
			14.882, 11.032, 8.059,   5.057,   3.04,    2.028,  1.86,    1.688,
			1.522,  1.351,  1.184,   1.013,   -1.184,  -1.351, -1.522,  -1.688,
			-1.86,  -2.028, -2.198,  -2.365,  -2.536,  -2.7,   -2.873,  0.846,
			0.675,  0.508,  0.337,   0.169,   0,       -0.169, -0.337,  -0.508,
			-0.675, -0.845, -1.013,  -3.04,   -3.21,   -3.375, -3.548,  -3.712,
			-3.884, -4.05,  -4.221,  -4.385,  -4.558,  -4.72,  -4.892,  -5.057,
			-5.229, -5.391, -5.565,  -5.726,  -5.898,  -6.061, -7.063,  -8.059,
			-9.06,  -9.885, -11.032, -12.006, -12.974, -13.93, -18.889, -24.897 };

	return pandar64;
}

//2.ruby128
FLidarDescription create_ruby128()
{
	FLidarDescription ruby128;
	ruby128.LidarType = "Surround";
	ruby128.NAME = "ruby128";
	ruby128.Channels = 128;
	ruby128.Range = 200;
	ruby128.HorizontalFov = 360.0f;
	ruby128.PointsPerSecond = 2300000;
	ruby128.RotationFrequency = 10.0f;
	ruby128.vfov = {
			-13.565, -1.09,   -4.39, 1.91,  -6.65,   -0.29,  -3.59, 2.71,  -5.79,
			0.51,    -2.79,   3.51,  -4.99, 1.31,    -1.99,  5.06,  -4.19, 2.11,
			-19.582, -1.29,   -3.39, 2.91,  -7.15,   -0.49,  -2.59, 3.71,  -5.99,
			0.31,    -1.79,   5.96,  -5.19, 1.11,    -0.99,  -4.29, 2.01,  -25,
			-0.19,   -3.49,   2.81,  -7.65, 0.61,    -2.69,  3.61,  -6.09, 1.41,
			-1.89,   5.46,    -5.29, 2.21,  -16.042, -1.19,  -4.49, 3.01,  -6.85,
			-0.39,   -3.69,   3.81,  -5.89, 0.41,    -2.89,  6.56,  -5.09, 1.21,
			-2.09,   -8.352,  -0.69, -3.99, 2.31,    -6.19,  0.11,  -3.19, 3.11,
			-5.39,   0.91,    -2.39, 3.96,  -4.59,   1.71,   -1.59, 7.41,  -3.79,
			2.51,    -10.346, -0.89, -2.99, 3.31,    -6.39,  -0.09, -2.19, 4.41,
			-5.59,   0.71,    -1.39, 11.5,  -4.79,   1.51,   -0.59, -3.89, 2.41,
			-11.742, 0.21,    -3.09, 3.21,  -6.5,    1.01,   -2.29, 4.16,  -5.69,
			1.81,    -1.49,   9,     -4.89, 2.61,    -9.244, -0.79, -4.09, 3.41,
			-6.29,   0.01,    -3.29, 4.71,  -5.49,   0.81,   -2.49, 15,    -4.69,
			1.61,    -1.69 };

	return ruby128;
}

//3.pandar128
FLidarDescription create_pandar128()
{
	FLidarDescription pandar128;
	pandar128.LidarType = "Surround";
	pandar128.NAME = "pandar128";
	pandar128.Channels = 128;
	pandar128.Range = 200;
	pandar128.HorizontalFov = 360.0f;
	pandar128.PointsPerSecond = 3456000;
	pandar128.RotationFrequency = 10.0f;

	pandar128.NoiseStdDev = 0;//0.019894833;
	pandar128.DropOffGenRate = 0.45;//0.75;//new

	for (int i = 0; i < 64; i++) {
		pandar128.vfov.push_back(-6 + 0.125 * i);
	}
	for (int i = 0; i < 36; i++) {
		pandar128.vfov.push_back(-6.5 - 0.5 * i);
	}
	for (int i = 0; i < 24; i++) {
		pandar128.vfov.push_back(2 + 0.5 * i);
	}
	for (int i = 0; i < 2; i++) {
		pandar128.vfov.push_back(14 + i);
		pandar128.vfov.push_back(-26 + i);
	}

	return pandar128;
}

//4.vlp16
FLidarDescription create_vlp16()
{
	FLidarDescription vlp16;
	vlp16.LidarType = "Surround";
	vlp16.NAME = "vlp16";
	vlp16.Channels = 16;
	vlp16.Range = 200;
	vlp16.HorizontalFov = 360.0f;
	vlp16.RotationFrequency = 10.0f;

	vlp16.NoiseStdDev = 0;//0.019894833
	vlp16.DropOffGenRate = 0.45;//0.75;//new

	vlp16.PointsPerSecond = 300000;
	for (int i = 0; i < 16; i++) {
		vlp16.vfov.push_back(-15.0 + 2.0 * i);
	}

	return vlp16;
}

//5.hdl64
FLidarDescription create_hdl64()
{
	FLidarDescription hdl64;

	//hdl64.NAME=5;
	hdl64.LidarType = "Surround";
	hdl64.NAME = "hdl64";
	hdl64.Channels=64;
    hdl64.Range=200;
    hdl64.HorizontalFov=360.0f;
    hdl64.RotationFrequency=10.0f;
    // hdl64.PointsPerSecond=
    for (int i = 0; i < 64; i++) {
        hdl64.vfov.push_back(-24.9 + 0.427 * i);
    }

	return hdl64;
}

//6.pandar_qt
FLidarDescription create_pandar_qt()
{
	FLidarDescription pandar_qt;
	pandar_qt.LidarType = "Surround";
	pandar_qt.NAME = "pandar_qt";
	pandar_qt.Channels = 64;
	pandar_qt.Range = 20;
	pandar_qt.HorizontalFov = 360.0f;
	pandar_qt.RotationFrequency = 10.0f;
	pandar_qt.PointsPerSecond = 384000;
	pandar_qt.NoiseStdDev = 0;
	pandar_qt.DropOffGenRate = 0.2;

	pandar_qt.vfov = {
		52.133,  49.795,  47.587,  45.487,  43.475,  41.537,  39.662,
		37.84,  36.064,  34.328,  32.627,  30.957,  29.315,  27.697,
		26.101,  24.524,  22.959,  21.415,  19.885,  18.368,  16.861,
		15.365,  13.877,  12.397,  10.923,  9.456,  7.993,   6.534,
		5.079,   3.626,   2.175,   0.725,   -0.725,   -2.175,  -3.626,
		-5.079,  -6.535,  -7.994,  -9.457,  -10.925, -12.399, -13.88,
		-15.368, -16.865, -18.372, -19.889, -21.42, -22.964, -24.517,
		-26.094, -27.69, -29.308, -30.95, -32.619, -34.32, -36.055,
		-37.831, -39.653, -41.528, -43.465, -45.477, -47.577, -49.785,
		-52.121 };

	return pandar_qt;
}

//7.bpearl
FLidarDescription create_bpearl()
{
	FLidarDescription bpearl;
	bpearl.LidarType = "Surround";
	bpearl.NAME = "bpearl";
	bpearl.Channels = 32;
	bpearl.Range = 30;
	bpearl.HorizontalFov = 360.0f;
	bpearl.RotationFrequency = 10.0f;
	bpearl.PointsPerSecond = 4608000;
	for (int i = 0; i < 32; i++) {
		bpearl.vfov.push_back((90 / 32) * i);
	}

	return bpearl;
}

//8.pandar_40m
FLidarDescription create_pandar_40m()
{
	FLidarDescription pandar_40m;
	pandar_40m.LidarType = "Surround";
	pandar_40m.NAME = "pandar_40m";
	pandar_40m.Channels = 40;
	pandar_40m.HorizontalFov = 360.0f;
	pandar_40m.Range = 120;
	pandar_40m.RotationFrequency = 10.0f;
	pandar_40m.PointsPerSecond = 1440000;
	pandar_40m.vfov = {
			15,    11,    8,     5,     3,     2,     1.67,  1.33,  1,     0.67,
			0.33,  0,     -0.33, -0.67, -1,    -1.33, -1.67, -2.00, -2.33, -2.67,
			-3.00, -3.33, -3.67, -4.00, -4.33, -4.67, -5.00, -5.33, -5.67, -6.00,
			-7,    -8,    -9,    -10,   -11,   -12,   -13,   -14,   -19,   -25 };

	return pandar_40m;
}

//9.pandar_40p
FLidarDescription create_pandar_40p()
{
	FLidarDescription pandar_40p;
	pandar_40p.LidarType = "Surround";
	pandar_40p.NAME = "pandar_40p";
	pandar_40p.Channels = 40;
	pandar_40p.HorizontalFov = 360.0f;
	pandar_40p.Range = 200;
	pandar_40p.RotationFrequency = 10.0f;
	pandar_40p.PointsPerSecond = 1440000;
	pandar_40p.vfov = {
			15,    11,    8,     5,     3,     2,     1.67,  1.33,  1,     0.67,
			0.33,  0,     -0.33, -0.67, -1,    -1.33, -1.67, -2.00, -2.33, -2.67,
			-3.00, -3.33, -3.67, -4.00, -4.33, -4.67, -5.00, -5.33, -5.67, -6.00,
			-7,    -8,    -9,    -10,   -11,   -12,   -13,   -14,   -19,   -25 };
	
	return pandar_40p;
}

//10.pandar_xt
FLidarDescription create_pandar_xt()
{
	FLidarDescription pandar_xt;
	pandar_xt.LidarType = "Surround";
	pandar_xt.NAME = "pandar_xt";
    pandar_xt.Channels=32;
    pandar_xt.HorizontalFov=360.0f;
	pandar_xt.RotationFrequency = 10.0f;
    for (int i = 0; i < 8; i++) {
    pandar_xt.vfov.push_back(15 - i);
    // pandar_xt.range = 50;
    }
    for (int i = 8; i < 25; i++) {
    pandar_xt.vfov.push_back(15 - i);
    // pandar_xt.range = 80;
    }
    for (int i = 25; i < 32; i++) {
    pandar_xt.vfov.push_back(15 - i);
    // pandar_xt.range = 50;
    }

	return pandar_xt;
}

//11.vlp32
FLidarDescription create_vlp32()
{
	FLidarDescription vlp32;
	vlp32.LidarType = "Surround";
	vlp32.NAME = "vlp32";
	vlp32.Channels = 32;
	vlp32.Range = 120;
	vlp32.HorizontalFov = 360.0f;
	vlp32.RotationFrequency = 10.0f;
	vlp32.PointsPerSecond = 600000;
	for (int i = 0; i < 32; i++)
	{
		vlp32.vfov.push_back(-25 + 40.0f / 32.0f* i);
	}
	vlp32.NoiseStdDev = 0;//0.019894833
	vlp32.DropOffGenRate = 0.45;//0.75;//new

	return vlp32;
}

//12. os1_64gen2
FLidarDescription create_os1_64_gen2()
{
	FLidarDescription os1_64_gen2;
	os1_64_gen2.LidarType = "Surround";
	os1_64_gen2.NAME = "os1_64_gen2";
	os1_64_gen2.Channels = 64;
	os1_64_gen2.Range = 120;
	os1_64_gen2.HorizontalFov = 360.0f;
	os1_64_gen2.RotationFrequency = 10.0f; //10 or 20
	os1_64_gen2.PointsPerSecond = 1311000;

	for (int i = 0; i < 64; i++) {
		os1_64_gen2.vfov.push_back(-22.5 + (45.0f / 64.0f) * i);
	}
	os1_64_gen2.NoiseStdDev = 0;

	return os1_64_gen2;
}