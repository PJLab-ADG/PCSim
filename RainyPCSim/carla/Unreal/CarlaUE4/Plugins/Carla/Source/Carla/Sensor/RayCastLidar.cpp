// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <PxScene.h>
#include <cmath>
#include <chrono>
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
#include "Carla/Sensor/Risley_prism.h" //新增
#include "Carla/Sensor/Solid_state.h"
#include "Carla/Sensor/Surround.h"
#include "Carla/Sensor/noise.h"

int32_t debug_particle_count = 0;
static float material_data[51][4] = {
    // Basecolor rough specular metal
    {0.5, 0.5, 0.5, 0},      // None
    {0.5, 0.5, 0.5, 0},      // Buildings
    {0.5, 0.5, 0.5, 0},      // Fences
    {0.5, 0.5, 0.5, 0},      // Other
    {0.3, 0.5, 0.5, 0},      // Pedestrians
    {0.5, 0.5, 0.5, 0.2},    // Poles
    {0.8, 0.95, 0.5, 0},     // RoadLines
    {0.2, 0.9, 0.5, 0},      // Roads
    {0.4, 0.8, 0.5, 0},      // Sidewalks
    {0.21, 0.5, 0.5, 0},     // Vegetation
    {0.5, 0.5, 0.5, 0},      // Vehicles
    {0.5, 0.5, 0.5, 0},      // Walls
    {0.90, 0.2, 0.5, 1},     // TrafficSigns
    {0.5, 0.5, 0.5, 0},      // Sky
    {0.1, 0.8, 0.5, 0},      // Ground
    {0.5, 0.5, 0.5, 0},      // Bridge
    {0.5, 0.2, 0.5, 1},      // RailTrack
    {0.5, 0.3, 0.5, 0},      // GuardRail
    {0.5, 0.3, 0.5, 0},      // TrafficLight
    {0.5, 0.5, 0.5, 0},      // Static
    {0.5, 0.5, 0.5, 0},      // Dynamic
    {0.5, 0.05, 0.255, 0.6}, // Water
    {0.1, 0.8, 0.5, 0},      // Terrain
    {0.02, 0.05, 0.5, 0.6},  // Particle
    {0.5, 0.5, 0.5, 0},      // Any
};
extern float noise1;
FActorDefinition ARayCastLidar::GetSensorDefinition()
{
  return UActorBlueprintFunctionLibrary::MakeLidarDefinition(TEXT("ray_cast"));
}

ARayCastLidar::ARayCastLidar(const FObjectInitializer &ObjectInitializer)
    : Super(ObjectInitializer)
{

  RandomEngine = CreateDefaultSubobject<URandomEngine>(TEXT("RandomEngine"));
  SetSeed(Description.RandomSeed);
}
float ARayCastLidar::ReadAlphaFromFile(const float perception)
{
  std::string filename = "/home/PJLAB/yangdonglin/workplace/carla/0.9.12/carla_LiDARSimLibBackUp/MieCsv/mie_ext.csv"; // modify depend on your own carla path
  std::ifstream MieCsv(filename, std::ios::in);
  std::string lineStr;
  float final_alpha = 0.0f;
  float rain_rate = 1.0 * perception / 100 * 50;
  rain_rate = rain_rate > 50 ? 50 : rain_rate;
  rain_rate = rain_rate < 0 ? 0 : rain_rate;
  while (getline(MieCsv, lineStr))
  {
    float rain, cur_alpha;
    sscanf(lineStr.c_str(), "%f %f", &rain, &cur_alpha);
    // std::cout << rain << " " << cur_alpha << std::endl;
    if (rain_rate > rain)
    {
      final_alpha = (final_alpha + cur_alpha) / 2;
    }
    else
    {
      final_alpha = cur_alpha;
      break;
    }
  }
  std::cout << "perception " << perception << " additional alpha is " << final_alpha << std::endl;
  return final_alpha;
}
void ARayCastLidar::Set(const FActorDescription &ActorDescription)
{
  ASensor::Set(ActorDescription);
  FLidarDescription LidarDescription;
  UActorBlueprintFunctionLibrary::SetLidar(ActorDescription, LidarDescription); // 从python接口获得并设置lidar属性

  // 若lidar_type == "default"， 则采用carla自带的lidar
  std::string created_lidar_name = TCHAR_TO_UTF8(*LidarDescription.NAME);
  std::string created_lidar_type = TCHAR_TO_UTF8(*LidarDescription.LidarType);

  bool enable_ghost = LidarDescription.EnableGhost;

  if (created_lidar_type == "Risley_prism") // Risley_prism式初始化
  {
    for (int i = 0; i < 10; ++i)
      std::cout << "creating " << created_lidar_name << std::endl;
    LidarDescription = create_Risley_prism(created_lidar_name);
    LidarDescription.Range = LidarDescription.Range * 100;
  }
  else if (created_lidar_type == "Solid_state") // Solid_state式初始化
  {
    for (int i = 0; i < 10; ++i)
      std::cout << "creating " << created_lidar_name << std::endl;
    LidarDescription = create_Solid_state(created_lidar_name);
    LidarDescription.Range = LidarDescription.Range * 100;
  }
  else if (created_lidar_type == "Surround") // 机械式初始化
  {
    for (int i = 0; i < 10; ++i)
      std::cout << "creating " << created_lidar_name << std::endl;
    LidarDescription = create_Surround(created_lidar_name);
    LidarDescription.Range = LidarDescription.Range * 100;
  }

  LidarDescription.EnableGhost = enable_ghost;

  float Precipitation = GetRainRate();
  LidarDescription.AtmospAttenRate = ReadAlphaFromFile(Precipitation) * 2 + LidarDescription.AtmospAttenRate;

  // LidarDescription.NoiseStdDev=noise1;//增加噪声属性

  Set(LidarDescription);
  // LidarLogging('S');
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

float ARayCastLidar::ComputeIntensity(const FSemanticDetection &RawDetection) const
{
  // not used
  const carla::geom::Location HitPoint = RawDetection.point;
  const float Distance = HitPoint.Length();

  const float AttenAtm = Description.AtmospAttenRate;
  const float AbsAtm = exp(-AttenAtm * Distance);

  const float IntRec = AbsAtm;

  return IntRec;
}

ARayCastLidar::FDetection ARayCastLidar::ComputeDetection(const FHitResult &HitInfo, const FTransform &SensorTransf) const
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

void ARayCastLidar::PreprocessRays(uint32_t Channels, uint32_t MaxPointsPerChannel)
{
  Super::PreprocessRays(Channels, MaxPointsPerChannel);
  // prepare random drop (unconditionally) at first
  for (auto ch = 0u; ch < Channels; ch++)
  {
    for (auto p = 0u; p < MaxPointsPerChannel; p++)
    {
      RayPreprocessCondition[ch][p] = !(DropOffGenActive && RandomEngine->GetUniformFloat() < Description.DropOffGenRate);
    }
  }
}

bool ARayCastLidar::PostprocessDetection(FDetection &Detection) const
{
  // add noise & check and drop the intensity lower than threshold
  if (Description.NoiseStdDev > std::numeric_limits<float>::epsilon())
  {
    const auto ForwardVector = Detection.point.MakeUnitVector();
    const auto Noise = ForwardVector * RandomEngine->GetNormalDistribution(0.0f, Description.NoiseStdDev);
    Detection.point += Noise;
  }

  const float Intensity = Detection.intensity;
  if (Intensity > Description.DropOffIntensityLimit)
    return true;
  else if (Intensity <= 1e-7)
    return false;
  else
    return RandomEngine->GetUniformFloat() < DropOffAlpha * Intensity + DropOffBeta;
}

void ARayCastLidar::ComputeAndSaveDetections(const FTransform &SensorTransform)
{
  ComputeAndSaveDetectionsAdvance(SensorTransform);
  // for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel)
  // {
  //   PointsPerChannel[idxChannel] = RecordedHits[idxChannel].size();
  // }

  // LidarData.ResetMemory(PointsPerChannel);
  // for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel)
  // {
  //   for (auto &hit : RecordedHits[idxChannel])
  //   {
  //     FDetection Detection = ComputeDetection(hit, SensorTransform);
  //     if (PostprocessDetection(Detection))
  //       LidarData.WritePointSync(Detection);
  //     else
  //       PointsPerChannel[idxChannel]--;
  //   }
  // }
  // LidarData.WriteChannelCount(PointsPerChannel);
}

void ARayCastLidar::SampleBeamParticles(std::vector<ParticleNoise> &beam_particles)
{
  // todo get the param [rain_rate beam_divergence] from blueprine param
  const float rain_rate(50);         // mm/h rain rate
  const float beam_divergence(5e-4); // radians
  const char model = 'S';            // snow | rain
  float particle_distribute_lambda, N0, occupancy_rate(1.0), Np;
  if (model == 'S') // Marshall Gunn
  {
    particle_distribute_lambda = 2.55 * pow(rain_rate, -0.48); // 1/mm
    N0 = 3800 * pow(rain_rate, -0.87);                         // mm^-4
    occupancy_rate = rain_rate / 8;
  }
  else // (model == 'R') Marshall Palmer
  {
    particle_distribute_lambda = 4.1 * pow(rain_rate, -0.21); // 1/mm
    N0 = 8000;                                                // mm^-4
    occupancy_rate = 1.0 * rain_rate;
  }
  Np = N0 * exp(-particle_distribute_lambda * 0.05) / particle_distribute_lambda; // calculate the particles nums where larger than 0.05
  // std::cout << "N0" << N0 <<" Np " << Np <<std::endl;
  particle_distribute_lambda = 1 / particle_distribute_lambda; // mm

  const float particle_generate_range = Description.Range / 100; // from cm to m
  const float tan_half_divergence = tan(beam_divergence / 2);    // half for radius
  const float beam_volumn = pow(tan_half_divergence * particle_generate_range, 2) * particle_generate_range * PI / 3;
  Np *= beam_volumn;
  float particle_diameter = 20, particle_beam_angle, particle_beam_distance, particle_beam_radius;
  int particles_len = 0;
  beam_particles.clear();
  beam_particles.reserve(Np);
  // float debug_particles_diameter = particle_distribute_lambda;
  // std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
  while (Np > particles_len)
  {
    do
    {
      particle_diameter = RandomEngine->GetExponentialDistribution(particle_distribute_lambda);
    } while (particle_diameter > 20);
    // todo check the diameter boundary condition
    particle_diameter = particle_diameter / 1000 + 1e-6;                                                              // from mm to m
    particle_beam_angle = RandomEngine->GetUniformFloatInRange(0, 2 * PI);                                            // radian
    particle_beam_distance = 0.01 + RandomEngine->GetUniformFloatInRange(particle_diameter, particle_generate_range); // m
    particle_beam_radius = RandomEngine->GetUniformFloatInRange(0, tan_half_divergence * particle_beam_distance);     // m
    // std::cout << particle_diameter << " " << particle_beam_angle << " " << particle_beam_distance << " " << particle_beam_radius << " " << particle_generate_range << " " << tan_half_divergence * particle_beam_distance << std::endl;
    ++particles_len;
    // debug_particles_diameter = debug_particles_diameter * 0.9 + particle_diameter * 100;
    beam_particles.emplace_back(ParticleNoise(particle_beam_angle, particle_beam_radius, particle_beam_distance, particle_diameter));
    // if (N0 < particles_len){
    //   std::cout <<"lambda" << particle_distribute_lambda << "beam_volumn: " << beam_volumn << " precipitation: " << precipitation << std::endl;
    //   std::cout << "generated particle " << particles_len << ": R="<< particle_diameter << " " << generated_particle_volume/precipitation << std::endl;
    //   std::cout << "generated over formula " << N0 << "<-" << particles_len << std::endl;
    // }
  }
  // std::chrono::system_clock::time_point end = std::chrono::system_clock::now();
  // std::cout << std::chrono::duration_cast<std::chrono::microseconds>(end-start).count() << std::endl;
  // std::cout << "debug_particles_diameter" << debug_particles_diameter << std::endl;
  // std::cout << "Beam Volumn" << beam_volumn << " InBeam cnt " << Np << " generated particles " << particles_len << " successfully" << std::endl;
}

ARayCastLidar::FDetection ARayCastLidar::ComputeDetectionAdvance(const FHitResult &HitInfo, const FTransform &SensorTransf, int idxChannel, int idxHorizon)
{
  FDetection Detection;
  const FVector HitPoint = HitInfo.ImpactPoint;
  Detection.point = SensorTransf.Inverse().TransformPosition(HitPoint);
  // Detection.livox_timestamp = HitInfo.Time;
  const float Distance = Detection.point.Length();
  uint32_t object_tag = static_cast<uint32_t>(HitInfo.Component->CustomDepthStencilValue);
  Detection.livox_timestamp = object_tag;
  object_tag = object_tag > 24 ? 0 : object_tag;
  FVector L = -(HitPoint - SensorTransf.GetLocation()).GetSafeNormal(); // incident vec
  FVector N = HitInfo.ImpactNormal;
  FVector V = L;
  FVector H = L; // (V + L).GetSafeNormal()
  const float cos_inc_angle = FVector::DotProduct(-(HitPoint - SensorTransf.GetLocation()).GetSafeNormal(), HitInfo.ImpactNormal);
  float BaseColor = material_data[object_tag][0]; // albedo
  float Roughness = material_data[object_tag][1];
  float Specular = material_data[object_tag][2];
  float Metallic = material_data[object_tag][3];

  // if (HitInfo.Component->GetMaterial(0) != nullptr && HitInfo.Component->GetMaterial(0)->GetBaseMaterial() != nullptr)
  // {
  //   Roughness = (HitInfo.Component->GetMaterial(0)->GetBaseMaterial()->Roughness).Constant;
  //   Specular = (HitInfo.Component->GetMaterial(0)->GetBaseMaterial()->Specular).Constant;
  //   Metallic = (HitInfo.Component->GetMaterial(0)->GetBaseMaterial()->Metallic).Constant;
  //   auto R = (HitInfo.Component->GetMaterial(0)->GetBaseMaterial()->BaseColor).Constant.R;
  //   auto G = (HitInfo.Component->GetMaterial(0)->GetBaseMaterial()->BaseColor).Constant.G;
  //   auto B = (HitInfo.Component->GetMaterial(0)->GetBaseMaterial()->BaseColor).Constant.B;
  //   BaseColor = (R + G + B) * (1.0 / 3.0 / 255.0);
  // }
  float DiffuseColor = BaseColor * (1 - Metallic);
  float F0 = 0.08 * Specular * (1 - Metallic) + BaseColor * Metallic;
  if (object_tag == 23) // droplet particle
  {
    debug_particle_count++;
    float occupiedRatio = 5e-4 / (Distance * tan(5e-4 / 2)); // const float tanHalfBeam = tan(5e-4 / 2);
    occupiedRatio = occupiedRatio * occupiedRatio;
    occupiedRatio = occupiedRatio > 1.0 ? 1.0 : occupiedRatio;
    BaseColor = occupiedRatio * BaseColor;

    auto tmp = (HitInfo.TraceEnd - HitInfo.TraceStart);
  }
  float a = Roughness * Roughness;
  float a2 = a * a;
  float NoH = FVector::DotProduct(N, H);
  float NoL = FVector::DotProduct(N, L);
  float NoV = FVector::DotProduct(N, V);
  float VoH = FVector::DotProduct(V, H);
  float d = (NoH * a2 - NoH) * NoH + 1;
  float D = a2 / (PI * d * d); // * Energy GGX [Walter et al. 2007, "Microfacet models for refraction through rough surfaces"]
  float Vis_SmithV = NoL * (NoV * (1 - a) + a);
  float Vis_SmithL = NoV * (NoL * (1 - a) + a);
  float Vis = 0.5f / ((Vis_SmithV + Vis_SmithL) + 1e-4); // Vis_SmithJointApprox [Heitz 2014, "Understanding the Masking-Shadowing Function in Microfacet-Based BRDFs" https://jcgt.org/published/0003/02/03/paper.pdf]
  Vis = Vis > 1 ? 1 : Vis;
  float f = (D * Vis) * F0;                              // float Fc = FMath::Pow(1 - VoH, 5); float F = Fc + (1 - Fc) * F0; // Fresnel [Schlick 1994, "An Inexpensive BRDF Model for Physically-Based Rendering"]
  float kd = (1 - F0) * (1 - Metallic);
  float cooked_f = kd * DiffuseColor / PI + f;

  const float DR = 6.4e-2;
  const float Cf = 4;
  float IntRec = Cf * cooked_f * FMath::Pow(cos_inc_angle, 0.8) * exp(-Description.AtmospAttenRate * Distance); // * PI * DR * DR / (Distance * Distance)
  bool is_water = true;
  if (is_water && object_tag == 7)
  {
    float t = 1 - (0.02 + 0.9799822978042073 * FMath::Pow(1 - cos_inc_angle, 5));
    t = t * t;
    IntRec *= t;
    if (IntRec < 4e-5 * Distance + 2e-3*RandomEngine->GetNormalDistribution(0,0.2))
      IntRec = 0;
  }
  // LidarLogging("object_tag " + std::to_string(object_tag) + " in " + std::to_string(acos(cos_inc_angle) / PI * 180) + " F0: " + std::to_string(F0) + " D:" + std::to_string(D) + " Vis:" + std::to_string(Vis) + " f: " + std::to_string(f) + "\n" + "  kd: " + std::to_string(kd) + " DiffColor: " + std::to_string(DiffuseColor) + " cooked_f:" + std::to_string(cooked_f) + " Dis: " + std::to_string(Distance) + " intensity: " + std::to_string(IntRec) + "\n");

  IntRec = IntRec > 1 ? 1 : IntRec;
  Detection.intensity = IntRec;

  return Detection;
}

void ARayCastLidar::LidarLogging(int pSize)
{
  std::string filename = "/home/PJLAB/yangdonglin/workplace/carla/0.9.12/carla_LiDARSimLibBackUp/debug/debug_particle.txt"; // modify depend on your own carla path
  std::ofstream Csv(filename, std::ios::out | std::ios::app);
  Csv << pSize << "\n";
  Csv.close();
}

void ARayCastLidar::LidarLogging(std::string pSize)
{
  std::string filename = "/home/PJLAB/yangdonglin/workplace/carla/0.9.12/carla_LiDARSimLibBackUp/debug/debug_particle.txt"; // modify depend on your own carla path
  std::ofstream Csv(filename, std::ios::out | std::ios::app);
  Csv << pSize << "\n";
  Csv.close();
}
void ARayCastLidar::LidarLogging(char ch)
{
  std::string filename = "/home/PJLAB/yangdonglin/workplace/carla/0.9.12/carla_LiDARSimLibBackUp/debug/debug_particle.txt"; // modify depend on your own carla path
  std::ofstream Csv(filename, std::ios::out | std::ios::app);
  Csv << ch << "\n";
  Csv.close();
}
void ARayCastLidar::UpsampleDroplet(const FTransform &SensorTransf)
{
  std::vector<std::vector<bool>> visited;
  std::vector<std::vector<FHitResult>> SWPRecordedHits;
  SWPRecordedHits.resize(Description.Channels);
  for (auto &hits : SWPRecordedHits)
  {
    hits.reserve(3072);
  }
  for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel)
  {
    PointsPerChannel[idxChannel] = RecordedHits[idxChannel].size();
    visited.emplace_back(std::vector<bool>(PointsPerChannel[idxChannel], false));
    // sort(RecordedHits[idxChannel].begin(), RecordedHits[idxChannel].end(), [&](const FHitResult &a, const FHitResult &b) -> bool
    //      {
    //   // return copysign(1. - a.TraceStart[0]/(fabs(a.TraceStart[0]) + fabs(a.TraceStart[1])),a.TraceStart[1]) < copysign(1. - b.TraceStart[0]/(fabs(b.TraceStart[0]) + fabs(b.TraceStart[1])),b.TraceStart[1]);
    //   return atan2(a.TraceStart[1],a.TraceStart[0]) < atan2(b.TraceStart[1],b.TraceStart[0]); });
    for (int idxHorizon = 0; idxHorizon < RecordedHits[idxChannel].size(); ++idxHorizon)
    {
      auto &hit = RecordedHits[idxChannel][idxHorizon];
      if (hit.bBlockingHit == false)
        continue;
      const uint32_t object_tag = static_cast<uint32_t>(hit.Component->CustomDepthStencilValue);
      if (object_tag == 23)
        visited[idxChannel][idxHorizon] = true;
    }
  }
  const float diffused_radius = 1.35f; // m
  const float resolution = 0.0021f;   // m
  const auto TransMat = SensorTransf.Inverse();
  for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel)
  {
    if (RandomEngine->GetUniformFloat() < 0.2) continue;
    for (int idxHorizon = 0; idxHorizon < RecordedHits[idxChannel].size(); ++idxHorizon)
    {
      auto &hit = RecordedHits[idxChannel][idxHorizon];
      if (hit.bBlockingHit == false)
        continue;
      const uint32_t object_tag = static_cast<uint32_t>(hit.Component->CustomDepthStencilValue);
      auto Distance = carla::geom::Location(TransMat.TransformPosition(hit.ImpactPoint)).Length();
      if (object_tag == 23 && visited[idxChannel][idxHorizon] == true && Distance > 5)
      {
        if (RandomEngine->GetUniformFloat() < 0.5) continue;
        auto grid = resolution * Distance;
        int grid_size = diffused_radius / grid;
        grid_size = grid_size > 64 ? 64 : grid_size;
        grid_size = std::min(static_cast<int>(Distance*Distance), grid_size);
        int adding_count = 0;
        float w_t = PI/2/grid_size;
        float A = grid_size * grid / 2;
        int seed = static_cast<int>(Distance*Distance*1e5)%10003;
        const float p_cluster = RandomEngine->GetUniformFloatInRange(0.01,0.14);
        for (int row = std::max(static_cast<int>(idxChannel - grid_size/2), 0); row < std::min(static_cast<int>(idxChannel + grid_size/2), static_cast<int>(Description.Channels)); ++row)
        {
          auto col_bias = RandomEngine->GetUniformIntInRange(static_cast<int>(-grid_size/4),static_cast<int>(grid_size/4));
          int col = std::max(static_cast<int>(idxHorizon - grid_size + col_bias), 0);
          int col_end = std::min(static_cast<int>(idxHorizon + grid_size + col_bias), static_cast<int>(PointsPerChannel[idxChannel]));
          for (; col < col_end; ++col)
          {
            if (visited[row][col] == true)
              continue;  
            float delta_range = std::max(std::min(diffused_radius - std::abs(col-idxHorizon)*grid, grid_size * grid), 0.0f);
            if (delta_range < 0.4) continue;
            auto &curhit = RecordedHits[row][col];
            if (curhit.bBlockingHit == false)
            {
              auto p = RandomEngine->GetUniformFloat();
              if (p < 0.004) // 0.012
              {
                float distort = (1- p) * A * sin(w_t * col + seed) + p / 0.035 * A * sin(w_t * row * 2 + seed - 1e4);
                float delta_dis = Distance + RandomEngine->GetUniformFloatInRange(-delta_range, delta_range) + p / 0.001 * A * sin(w_t * col + seed); // m
                delta_dis = std::max(delta_dis, 1.0f);
                curhit.bBlockingHit = true;
                curhit.ImpactPoint = curhit.TraceStart + (curhit.TraceEnd - curhit.TraceStart).GetSafeNormal() * delta_dis * 100;
                curhit.ImpactNormal = hit.ImpactNormal;
                curhit.Component = hit.Component;
                adding_count += 1;
              }
            }
            else
            {
              auto curdis = carla::geom::Location(TransMat.TransformPosition(curhit.ImpactPoint)).Length();
              if (curdis >= Distance)
              {
                auto p = RandomEngine->GetUniformFloat();
                if (p < p_cluster) //0.002
                {
                  float distort = (1 - p) * A * sin(w_t * col + seed) + p / 0.035 * A * sin(w_t * row * 2 + seed - 1e4);
                  float delta_dis = Distance + distort + RandomEngine->GetUniformFloatInRange(-delta_range, delta_range); // m
                  delta_dis = std::max(delta_dis, 1.0f);
                  curhit.bBlockingHit = true;
                  curhit.ImpactPoint = curhit.TraceStart + (curhit.TraceEnd - curhit.TraceStart).GetSafeNormal() * delta_dis * 100;
                  curhit.ImpactNormal = hit.ImpactNormal;
                  curhit.Component = hit.Component;
                  adding_count += 1;
                }
              }
            }
            if (curhit.ImpactPoint[2] < 0)
              curhit.bBlockingHit = false;
          }
        }
        // std::cout << adding_count << std::endl;
      }
    }
  }
  for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel)
  {
    int idxHorizon = 0;
    for (auto &hit : RecordedHits[idxChannel])
    {
      if (hit.bBlockingHit == true)
        SWPRecordedHits[idxChannel].emplace_back(hit);
    }
  }
  RecordedHits.swap(SWPRecordedHits);
}
void ARayCastLidar::ComputeAndSaveDetectionsAdvance(const FTransform &SensorTransform)
{
  if (Description.LidarType == "Surround" && Description.NAME == "waymo_top")
  {
    UpsampleDroplet(SensorTransform);
    std::cout<<"upsample"<<std::endl;
  }
  for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel)
  {
    PointsPerChannel[idxChannel] = RecordedHits[idxChannel].size();
  }
  LidarData.ResetMemory(PointsPerChannel);

  // todo get rain fall rate to replace the const value 3.5
  // const double particle_diameter_expected = 3.5 * 10; // *10 for mm
  // const double particle_distribute_lambda = 1 / particle_diameter_expected;
  // std::vector<ParticleNoise> beam_particles;
  // int particles_count = 0;
  debug_particle_count = 0;
  for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel)
  {
    int idxHorizon = 0;
    for (auto &hit : RecordedHits[idxChannel])
    {
      // SampleBeamParticles(beam_particles);
      // particles_count += beam_particles.size();
      // std::cout << "start Compute Detection, sampled particles " << beam_particles.size() << std::endl;
      FDetection Detection = ComputeDetectionAdvance(hit, SensorTransform, idxChannel, idxHorizon++);
      if (PostprocessDetection(Detection))
        LidarData.WritePointSync(Detection);
      else
        PointsPerChannel[idxChannel]--;
    }
  }
  LidarData.WriteChannelCount(PointsPerChannel);
  // LidarLogging(debug_particle_count);
  std::cout << "LiDAR detect particle " << debug_particle_count << std::endl;
}
