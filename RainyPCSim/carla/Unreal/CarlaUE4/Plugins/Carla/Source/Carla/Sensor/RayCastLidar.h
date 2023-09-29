// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once


#include "Carla/Actor/ActorDefinition.h"
#include "Carla/Sensor/LidarDescription.h"
#include "Carla/Sensor/Sensor.h"
#include "Carla/Sensor/RayCastSemanticLidar.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"

#include <compiler/disable-ue4-macros.h>
#include <carla/sensor/data/LidarData.h>
#include <compiler/enable-ue4-macros.h>

#include "RayCastLidar.generated.h"

struct ParticleNoise {
    float Angle;
    float Radius;
    float Distance;
    float Diameter;
    ParticleNoise(){}
    ParticleNoise(float a_,float r_, float dis, float dia):Angle(a_),Radius(r_),Distance(dis),Diameter(dia){}
    float DisSquare() const { return (Distance * Distance + Radius * Radius); }
    float Dis() const { return sqrt(Distance * Distance + Radius * Radius); }
    bool operator <(const ParticleNoise &p) const {return Diameter < p.Diameter;}
};
/// A ray-cast based Lidar sensor.
UCLASS()
class CARLA_API ARayCastLidar : public ARayCastSemanticLidar
{
  GENERATED_BODY()

  using FLidarData = carla::sensor::data::LidarData;
  using FDetection = carla::sensor::data::LidarDetection;

public:
  static FActorDefinition GetSensorDefinition();

  ARayCastLidar(const FObjectInitializer &ObjectInitializer);
  virtual void Set(const FActorDescription &Description) override;
  virtual void Set(const FLidarDescription &LidarDescription) override;

  virtual void PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaTime);

private:
  /// Compute the received intensity of the point
  float ComputeIntensity(const FSemanticDetection& RawDetection) const;
  FDetection ComputeDetection(const FHitResult& HitInfo, const FTransform& SensorTransf) const;

  void PreprocessRays(uint32_t Channels, uint32_t MaxPointsPerChannel) override;
  bool PostprocessDetection(FDetection& Detection) const;

  void ComputeAndSaveDetections(const FTransform& SensorTransform) override;

  void SampleBeamParticles(std::vector<ParticleNoise> &beam_particles);
  FDetection ComputeDetectionAdvance(const FHitResult &HitInfo, const FTransform &SensorTransf, int idxChannel, int idxHorizon);
  void ComputeAndSaveDetectionsAdvance(const FTransform &SensorTransform);
  float ReadAlphaFromFile(float perception);
  void UpsampleDroplet(const FTransform &SensorTransf);
  void LidarLogging(char ch);
  void LidarLogging(int pSize);
  void LidarLogging(std::string pSize);
  FLidarData LidarData;

  /// Enable/Disable general dropoff of lidar points
  bool DropOffGenActive;

  /// Slope for the intensity dropoff of lidar points, it is calculated
  /// throught the dropoff limit and the dropoff at zero intensity
  /// The points is kept with a probality alpha*Intensity + beta where
  /// alpha = (1 - dropoff_zero_intensity) / droppoff_limit
  /// beta = (1 - dropoff_zero_intensity)
  float DropOffAlpha;
  float DropOffBeta;
  std::vector<float> w_R = std::vector<float>(2501, 0.0);
  std::vector<float> w_Intensity = std::vector<float>(2501, 0.0);
  // std::vector<std::vector> _ser_points
};
