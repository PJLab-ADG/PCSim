#include "Carla/Sensor/livox.h"
#include <iostream>
#include <math.h> 
#include <vector>

std::vector<std::vector<float>> livox_csv_reader(const std::string& lidar_name)
{
	//std::string lidar_name = my_livox.NAME;
	std::string filename = "/home/PJLAB/zhaowenquan/Desktop/carla/LivoxCsv/"; //modify depend on your own carla path
	if (lidar_name == "horizon")
	{
		filename += "horizon.csv";
		std::cout << "Horizon_csv file read successfully" << std::endl;
	}
	if (lidar_name == "mid40")
	{
		filename += "mid40.csv";
		std::cout << "Mid40_csv file read successfully" << std::endl;
	}
	if (lidar_name == "mid70")
	{
		filename += "mid70.csv";
		std::cout << "Mid70_csv file read successfully" << std::endl;
	}
	if (lidar_name == "avia")
	{
		filename += "avia.csv";
		std::cout << "Avia_csv file read successfully" << std::endl;
	}
	if (lidar_name == "tele")
	{
		filename += "tele.csv";
		std::cout << "Tele_csv file read successfully" << std::endl;
	}

	std::ifstream LivoxCsv(filename, std::ios::in);
	std::string lineStr;
	std::vector<std::vector<float>> LivoxPointInfo;
	while (getline(LivoxCsv, lineStr))
	{
		std::stringstream ss(lineStr);
		std::string str;
		std::vector<float> lineArray;

		while (getline(ss, str, ','))
		{
			lineArray.push_back(std::stof(str));
		}	
		LivoxPointInfo.push_back(lineArray);
	}

	return LivoxPointInfo;
}

FLidarDescription create_livox(const string& lidar_name)
{
	FLidarDescription livox;

	//livox.NAME = 11;
	livox.LidarType = "livox";
	livox.NAME = FString(lidar_name.c_str());
	livox.pointnums = livox.hfov.size();
	livox.Channels = 1;
	livox.PointsPerSecond = 240000;
	livox.Range = 260;

	// livox.NoiseStdDev = 0.0264734585;
	// livox.DropOffGenRate = 0.504;//new

	livox.Decay = 0.25;
	livox.DropOffIntensityLimit = 0.0;
	livox.DropOffAtZeroIntensity = 0.0;
	livox.DropOffGenRate = 0.3;


	
	// livox.Decay = 0.25;
	livox.livox_csv_info = livox_csv_reader(lidar_name); //zwq, read csv
	livox.LivoxSize = livox.livox_csv_info.size();
	livox.LivoxCount = 0;
	
	std::cout<<"create "<<lidar_name<<" successfully"<<std::endl;

	return livox;

}

