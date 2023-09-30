// #include <stdio.h>
// #include <math.h>
#include "Carla/Sensor/Solid_state.h"
#include <iostream>
#include <math.h> 
#include <vector>

FLidarDescription create_Solid_state(const std::string& lidar_name)
{
    std::cout<<"create "<<lidar_name<<" successfully"<<std::endl;
	if (lidar_name == "rs_m1")
	{
		return create_rs_m1();
	}
    return FLidarDescription(); //return a default description
}

FLidarDescription create_rs_m1()
{
	FLidarDescription rs_m1;

    int m1_fov_num=0;
    float fps=10.0f;
    float wx=7200.0f;
    float wy=100.0f;
    float phi=0.5*M_PI;
    float theta1=0.01*M_PI;
    float theta2=-0.01*M_PI;
    float theta3=0.02*M_PI;
    float theta4=-0.02*M_PI;

    std::vector<float> t;
    std::vector<float> hfov1,vfov1,hfov2,vfov2,x3,y3,x4,y4,hfov3,vfov3,hfov4,vfov4,x5,y5,x6,y6,hfov5,vfov5,hfov6,vfov6,x7,y7,x8,y8,hfov7,vfov7,hfov8,vfov8,x9,y9,x10,y10,hfov9,vfov9,hfov10,vfov10;
    std::vector<float> rs_m1_hfov,rs_m1_vfov;
    for(int i=0;i<=1150;i++)
    {
        t.push_back(float(0+(1.0f/11500.0f/fps)*i));
    }
    for(int i=0;i<t.size();i++)
    {   
        hfov1.push_back(12.5*cos(2*M_PI*wx*t[i]));

        vfov1.push_back(9.25*sin(2*M_PI*wy*t[i]+phi)+3.25);

        hfov2.push_back(12.5*cos(2*M_PI*wx*t[i]));

        vfov2.push_back(7.25*sin(2*M_PI*wy*t[i]+phi)-5.25);

        x3.push_back(12.5*cos(2*M_PI*wx*t[i])-24);
        y3.push_back(9.25*sin(2*M_PI*wy*t[i]+phi)+2.25);
        x4.push_back(12.5*cos(2*M_PI*wx*t[i])-24);
        y4.push_back(7.25*sin(2*M_PI*wy*t[i]+phi)-6.25);

        hfov3.push_back(x3[i]*cos(theta2)+y3[i]*sin(theta2));

        vfov3.push_back(-x3[i]*sin(theta1)+y3[i]*cos(theta1));

        hfov4.push_back(x4[i]*cos(theta2)+y4[i]*sin(theta2));

        vfov4.push_back(-x4[i]*sin(theta1)+y4[i]*cos(theta1));

        x5.push_back(12.5*cos(2*M_PI*wx*t[i])+24);
        y5.push_back(9.25*sin(2*M_PI*wy*t[i]+phi)+2.25);
        x6.push_back(12.5*cos(2*M_PI*wx*t[i])+24);
        y6.push_back(7.25*sin(2*M_PI*wy*t[i]+phi)-6.25);
        hfov5.push_back(x5[i]*cos(theta1)+y5[i]*sin(theta1));

        vfov5.push_back(-x5[i]*sin(theta2)+y5[i]*cos(theta2));

        hfov6.push_back(x6[i]*cos(theta2)+y6[i]*sin(theta1));

        vfov6.push_back(-x6[i]*sin(theta2)+y6[i]*cos(theta2));

        x7.push_back(12.5*cos(2*M_PI*wx*t[i])-48);
        y7.push_back(9.25*sin(2*M_PI*wy*t[i]+phi)+0.25);
        x8.push_back(12.5*cos(2*M_PI*wx*t[i])-48);
        y8.push_back(7.25*sin(2*M_PI*wy*t[i]+phi)-8.25);
        hfov7.push_back(x7[i]*cos(theta4)+y7[i]*sin(theta4));

        vfov7.push_back(-x7[i]*sin(theta3)+y7[i]*cos(theta3));

        hfov8.push_back(x8[i]*cos(theta4)+y8[i]*sin(theta4));

        vfov8.push_back(-x8[i]*sin(theta3)+y8[i]*cos(theta3));   

        x9.push_back(12.5*cos(2*M_PI*wx*t[i])+48);
        y9.push_back(9.25*sin(2*M_PI*wy*t[i]+phi)+0.25);
        x10.push_back(12.5*cos(2*M_PI*wx*t[i])+48);
        y10.push_back(7.25*sin(2*M_PI*wy*t[i]+phi)-8.25);
        hfov9.push_back(x9[i]*cos(theta3)+y9[i]*sin(theta3));

        vfov9.push_back(-x9[i]*sin(theta4)+y9[i]*cos(theta4));

        hfov10.push_back(x10[i]*cos(theta3)+y10[i]*sin(theta3));

        vfov10.push_back(-x10[i]*sin(theta4)+y10[i]*cos(theta4));

    }



    rs_m1_hfov=hfov1;
    for(int i=0;i<t.size();i++)
    {
    rs_m1_hfov.push_back(hfov2[i]);
    }
    for(int i=0;i<t.size();i++)
    {
    rs_m1_hfov.push_back(hfov3[i]);
    }
    for(int i=0;i<t.size();i++)
    {
    rs_m1_hfov.push_back(hfov4[i]);
    }
    for(int i=0;i<t.size();i++)
    {
    rs_m1_hfov.push_back(hfov5[i]);
    }
    for(int i=0;i<t.size();i++)
    {
    rs_m1_hfov.push_back(hfov6[i]);
    }
    for(int i=0;i<t.size();i++)
    {
    rs_m1_hfov.push_back(hfov7[i]);
    }
    for(int i=0;i<t.size();i++)
    {
    rs_m1_hfov.push_back(hfov8[i]);
    }
    for(int i=0;i<t.size();i++)
    {
    rs_m1_hfov.push_back(hfov9[i]);
    }
    for(int i=0;i<t.size();i++)
    {
    rs_m1_hfov.push_back(hfov10[i]);
    }

    rs_m1_vfov=vfov1;
    for(int i=0;i<t.size();i++)
    {
    rs_m1_vfov.push_back(vfov2[i]);
    }
    for(int i=0;i<t.size();i++)
    {
    rs_m1_vfov.push_back(vfov3[i]);
    }
    for(int i=0;i<t.size();i++)
    {
    rs_m1_vfov.push_back(vfov4[i]);
    }
    for(int i=0;i<t.size();i++)
    {
    rs_m1_vfov.push_back(vfov5[i]);
    }
    for(int i=0;i<t.size();i++)
    {
    rs_m1_vfov.push_back(vfov6[i]);
    }
    for(int i=0;i<t.size();i++)
    {
    rs_m1_vfov.push_back(vfov7[i]);
    }
    for(int i=0;i<t.size();i++)
    {
    rs_m1_vfov.push_back(vfov8[i]);
    }
    for(int i=0;i<t.size();i++)
    {
    rs_m1_vfov.push_back(vfov9[i]);
    }
    for(int i=0;i<t.size();i++)
    {
    rs_m1_vfov.push_back(vfov10[i]);
    }

    rs_m1.sub_vfov.push_back(vfov1);
    rs_m1.sub_vfov.push_back(vfov2);
    rs_m1.sub_vfov.push_back(vfov3);
    rs_m1.sub_vfov.push_back(vfov4);
    rs_m1.sub_vfov.push_back(vfov5);
    rs_m1.sub_vfov.push_back(vfov6);
    rs_m1.sub_vfov.push_back(vfov7);
    rs_m1.sub_vfov.push_back(vfov8);
    rs_m1.sub_vfov.push_back(vfov9);
    rs_m1.sub_vfov.push_back(vfov10);

    rs_m1.sub_hfov.push_back(hfov1);
    rs_m1.sub_hfov.push_back(hfov2);
    rs_m1.sub_hfov.push_back(hfov3);
    rs_m1.sub_hfov.push_back(hfov4);
    rs_m1.sub_hfov.push_back(hfov5);
    rs_m1.sub_hfov.push_back(hfov6);
    rs_m1.sub_hfov.push_back(hfov7);
    rs_m1.sub_hfov.push_back(hfov8);
    rs_m1.sub_hfov.push_back(hfov9);
    rs_m1.sub_hfov.push_back(hfov10);
	
	rs_m1.LidarType = "Solid_state";
    rs_m1.NAME="rs_m1";
    rs_m1.vfov=rs_m1_vfov;
    rs_m1.hfov=rs_m1_hfov;
    rs_m1.pointnums=rs_m1_hfov.size();
    rs_m1.Channels=1;
    rs_m1.PointsPerSecond=1500000;

    rs_m1.NoiseStdDev=0.02753913;
    rs_m1.DropOffGenRate=0.952;//new

	return rs_m1;
}