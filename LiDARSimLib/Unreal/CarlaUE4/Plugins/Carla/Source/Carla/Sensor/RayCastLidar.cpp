// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <PxScene.h>
#include <cmath>
#include "Carla.h"
#include "Carla/Sensor/RayCastLidar.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include "carla/geom/Math.h"

#include <compiler/disable-ue4-macros.h>
#include "carla/geom/Math.h"
#include "carla/geom/Location.h"
#include <compiler/enable-ue4-macros.h>

#include "DrawDebugHelpers.h"
#include "Engine/CollisionProfile.h"
#include "Runtime/Engine/Classes/Kismet/KismetMathLibrary.h"
#include "Carla/Sensor/Risley_prism.h"  //新增
#include "Carla/Sensor/Solid_state.h" 
#include "Carla/Sensor/Surround.h"
#include "Carla/Sensor/noise.h"

extern float noise1;
FActorDefinition ARayCastLidar::GetSensorDefinition()
{
  return UActorBlueprintFunctionLibrary::MakeLidarDefinition(TEXT("ray_cast"));
}


ARayCastLidar::ARayCastLidar(const FObjectInitializer& ObjectInitializer)
  : Super(ObjectInitializer) {

  RandomEngine = CreateDefaultSubobject<URandomEngine>(TEXT("RandomEngine"));
  SetSeed(Description.RandomSeed);
}

void ARayCastLidar::Set(const FActorDescription &ActorDescription)
{
  ASensor::Set(ActorDescription);
  FLidarDescription LidarDescription;
  UActorBlueprintFunctionLibrary::SetLidar(ActorDescription, LidarDescription); //从python接口获得并设置lidar属性

  //若lidar_type == "default"， 则采用carla自带的lidar
  std::string created_lidar_name = TCHAR_TO_UTF8(*LidarDescription.NAME);
  std::string created_lidar_type = TCHAR_TO_UTF8(*LidarDescription.LidarType);

  bool enable_ghost = LidarDescription.EnableGhost;

  if (created_lidar_type == "Risley_prism")  //Risley_prism式初始化  
  {
    for (int i = 0; i < 10; ++i) std::cout<<"creating "<<created_lidar_name<<std::endl;
	  LidarDescription = create_Risley_prism(created_lidar_name);
    LidarDescription.Range=LidarDescription.Range*100;
  }
  else if (created_lidar_type == "Solid_state")  //Solid_state式初始化
  {
    for (int i = 0; i < 10; ++i) std::cout<<"creating "<<created_lidar_name<<std::endl;
	  LidarDescription = create_Solid_state(created_lidar_name);
    LidarDescription.Range=LidarDescription.Range*100;
  }
  else if (created_lidar_type == "Surround")  //机械式初始化
  {
    for (int i = 0; i < 10; ++i) std::cout<<"creating "<<created_lidar_name<<std::endl;
	  LidarDescription = create_Surround(created_lidar_name);
    LidarDescription.Range=LidarDescription.Range*100;
  }

  LidarDescription.EnableGhost = enable_ghost;
  
  // LidarDescription.NoiseStdDev=noise1;//增加噪声属性

  Set(LidarDescription);
}

void ARayCastLidar::Set(const FLidarDescription &LidarDescription)
{
  Description = LidarDescription;
  LidarData = FLidarData(Description.Channels);
  CreateLasers();
  PointsPerChannel.resize(Description.Channels);
  
  // Compute drop off model parameters
  DropOffBeta = 1.0f - Description.DropOffAtZeroIntensity;
  DropOffAlpha = Description.DropOffAtZeroIntensity / Description.DropOffIntensityLimit;
  DropOffGenActive = Description.DropOffGenRate > std::numeric_limits<float>::epsilon();
}

void ARayCastLidar::PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaTime)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(ARayCastLidar::PostPhysTick);
  SimulateLidar(DeltaTime);

  {
    TRACE_CPUPROFILER_EVENT_SCOPE_STR("Send Stream");
    auto DataStream = GetDataStream(*this);
    DataStream.Send(*this, LidarData, DataStream.PopBufferFromPool());
  }
}

float ARayCastLidar::ComputeIntensity(const FSemanticDetection& RawDetection) const
{
  const carla::geom::Location HitPoint = RawDetection.point;
  const float Distance = HitPoint.Length();
  const float AttenAtm = Description.AtmospAttenRate;
  const float AbsAtm = exp(-AttenAtm * Distance);

  const float IntRec = AbsAtm;

  return IntRec;
}

ARayCastLidar::FDetection ARayCastLidar::ComputeDetection(const FHitResult& HitInfo, const FTransform& SensorTransf) const
{
  FDetection Detection;
  const FVector HitPoint = HitInfo.ImpactPoint;
  Detection.point = SensorTransf.Inverse().TransformPosition(HitPoint);

  const float Distance = Detection.point.Length();

  const float AttenAtm = Description.AtmospAttenRate;
  const float AbsAtm = exp(-AttenAtm * Distance);

  const float IntRec = AbsAtm;
  
  Detection.intensity = IntRec;

  Detection.livox_timestamp = HitInfo.Time;

  return Detection;
}

  void ARayCastLidar::PreprocessRays(uint32_t Channels, uint32_t MaxPointsPerChannel) {
    Super::PreprocessRays(Channels, MaxPointsPerChannel);

    for (auto ch = 0u; ch < Channels; ch++) {
      for (auto p = 0u; p < MaxPointsPerChannel; p++) {
        RayPreprocessCondition[ch][p] = !(DropOffGenActive && RandomEngine->GetUniformFloat() < Description.DropOffGenRate);
      }
    }
  }

  bool ARayCastLidar::PostprocessDetection(FDetection& Detection) const
  {
    if (Description.NoiseStdDev > std::numeric_limits<float>::epsilon()) {
      const auto ForwardVector = Detection.point.MakeUnitVector();
      const auto Noise = ForwardVector * RandomEngine->GetNormalDistribution(0.0f, Description.NoiseStdDev);
      Detection.point += Noise;
    }

    const float Intensity = Detection.intensity;
    if(Intensity > Description.DropOffIntensityLimit)
      return true;
    else
      return RandomEngine->GetUniformFloat() < DropOffAlpha * Intensity + DropOffBeta;
  }

  void ARayCastLidar::ComputeAndSaveDetections(const FTransform& SensorTransform) {
      for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel)
        {
        PointsPerChannel[idxChannel] = RecordedHits[idxChannel].size();
        }

      LidarData.ResetMemory(PointsPerChannel);

      for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel) {
        for (auto& hit : RecordedHits[idxChannel]) {
          FDetection Detection = ComputeDetection(hit, SensorTransform);
          if (PostprocessDetection(Detection))
            LidarData.WritePointSync(Detection);
          else
            PointsPerChannel[idxChannel]--;
      }
    }
      LidarData.WriteChannelCount(PointsPerChannel);
  }
