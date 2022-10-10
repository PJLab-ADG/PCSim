// Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <PxScene.h>
#include <cmath>
#include "Carla.h"
#include "Carla/Actor/ActorBlueprintFunctionLibrary.h"
#include "Carla/Sensor/RayCastSemanticLidar.h"

#include <compiler/disable-ue4-macros.h>
#include "carla/geom/Math.h"
#include <compiler/enable-ue4-macros.h>
#include "DrawDebugHelpers.h"
#include "Engine/CollisionProfile.h"
#include "Runtime/Engine/Classes/Kismet/KismetMathLibrary.h"
#include "Runtime/Core/Public/Async/ParallelFor.h"
#include "Carla/Sensor/Surround.h" 
#include "Carla/Sensor/Solid_state.h" 
#include "Carla/Sensor/Risley_prism.h"
#include "Carla/Sensor/noise.h"
#include "iostream"
#include "ctime"
#include <cstdlib>

namespace crp = carla::rpc;

FActorDefinition ARayCastSemanticLidar::GetSensorDefinition()
{
  return UActorBlueprintFunctionLibrary::MakeLidarDefinition(TEXT("ray_cast_semantic"));
}

ARayCastSemanticLidar::ARayCastSemanticLidar(const FObjectInitializer& ObjectInitializer)
  : Super(ObjectInitializer)
{
  PrimaryActorTick.bCanEverTick = true;
}

void ARayCastSemanticLidar::Set(const FActorDescription &ActorDescription)
{
  Super::Set(ActorDescription);
  FLidarDescription LidarDescription;
  UActorBlueprintFunctionLibrary::SetLidar(ActorDescription, LidarDescription); //get description from pythonAPI and set lidar

  //if lidar_type == "default", use carla default lidar
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
  // LidarDescription.NoiseStdDev=noise1;
  Set(LidarDescription);
}

void ARayCastSemanticLidar::Set(const FLidarDescription &LidarDescription)
{
  Description = LidarDescription;
  SemanticLidarData = FSemanticLidarData(Description.Channels);
  CreateLasers();
  PointsPerChannel.resize(Description.Channels);
}

void ARayCastSemanticLidar::CreateLasers()
{
	const auto NumberOfLasers = Description.Channels;
	check(NumberOfLasers > 0u);
	const float DeltaAngle = NumberOfLasers == 1u ? 0.f :
		(Description.UpperFovLimit - Description.LowerFovLimit) /
		static_cast<float>(NumberOfLasers - 1);
	// LaserAngles.resize(NumberOfLasers);
  LaserAngles.clear();

  std::string my_lidar_type = TCHAR_TO_UTF8(*Description.LidarType);//FString to std::string
	if (Description.LidarType == "default")
	{
    std::cout<<"creating lasers of "<<my_lidar_type<<std::endl;
		//carla default uniform resolution
		for (auto i = 0u; i < NumberOfLasers; ++i)
		{
			const float VerticalAngle =
				Description.UpperFovLimit - static_cast<float>(i) * DeltaAngle;
			LaserAngles.emplace_back(VerticalAngle);
		}
	}
	else if (Description.LidarType == "Surround")
	{
    std::cout<<"creating lasers of "<<my_lidar_type<<std::endl;
    //ֱassign value use predifined vfov
		LaserAngles = Description.vfov;
	}
	else if (Description.LidarType == "Solid_state")
	{
    std::cout<<"creating lasers of "<<my_lidar_type<<std::endl;
		//SimulateLidar with vfov and hfov
	}
	else if (Description.LidarType == "Risley_prism")
	{
    std::cout<<"creating lasers of "<<my_lidar_type<<std::endl;
		//read information from csv (done in Risley_prism.cpp)
	}
}

void ARayCastSemanticLidar::PostPhysTick(UWorld *World, ELevelTick TickType, float DeltaTime)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(ARayCastSemanticLidar::PostPhysTick);
  SimulateLidar(DeltaTime);

  {
    TRACE_CPUPROFILER_EVENT_SCOPE_STR("Send Stream");
    auto DataStream = GetDataStream(*this);
    DataStream.Send(*this, SemanticLidarData, DataStream.PopBufferFromPool());
  }
}

void ARayCastSemanticLidar::SimulateLidar(const float DeltaTime)
{
  TRACE_CPUPROFILER_EVENT_SCOPE(ARayCastSemanticLidar::SimulateLidar);
  if((Description.LidarType == "default") || (Description.LidarType == "Surround"))
  {
    const uint32 ChannelCount = Description.Channels;
    const uint32 PointsToScanWithOneLaser =
      FMath::RoundHalfFromZero(
          (Description.PointsPerSecond * DeltaTime)/float(ChannelCount));

    if (PointsToScanWithOneLaser <= 0)
    {
      UE_LOG(
          LogCarla,
          Warning,
          TEXT("%s: no points requested this frame, try increasing the number of points per second."),
          *GetName());
      return;
    }

    check(ChannelCount == LaserAngles.size());

    const float CurrentHorizontalAngle = carla::geom::Math::ToDegrees(
        SemanticLidarData.GetHorizontalAngle());
    const float AngleDistanceOfTick = Description.RotationFrequency * Description.HorizontalFov
        * DeltaTime;
    const float AngleDistanceOfLaserMeasure = AngleDistanceOfTick / PointsToScanWithOneLaser;

    ResetRecordedHits(ChannelCount, PointsToScanWithOneLaser);
    PreprocessRays(ChannelCount, PointsToScanWithOneLaser);

    GetWorld()->GetPhysicsScene()->GetPxScene()->lockRead();
    {
      TRACE_CPUPROFILER_EVENT_SCOPE(ParallelFor);
      ParallelFor(ChannelCount, [&](int32 idxChannel) {
        TRACE_CPUPROFILER_EVENT_SCOPE(ParallelForTask);

        FCollisionQueryParams TraceParams = FCollisionQueryParams(FName(TEXT("Laser_Trace")), true, this);
        TraceParams.bTraceComplex = true;
        TraceParams.bReturnPhysicalMaterial = false;

        for (auto idxPtsOneLaser = 0u; idxPtsOneLaser < PointsToScanWithOneLaser; idxPtsOneLaser++) {
          FHitResult HitResult;
          const float VertAngle = LaserAngles[idxChannel];
          const float HorizAngle = std::fmod(CurrentHorizontalAngle + AngleDistanceOfLaserMeasure
              * idxPtsOneLaser, Description.HorizontalFov) - Description.HorizontalFov / 2;
          const bool PreprocessResult = RayPreprocessCondition[idxChannel][idxPtsOneLaser];

          if (PreprocessResult && ShootLaser(VertAngle, HorizAngle, HitResult, TraceParams, 0.0)) {
            WritePointAsync(idxChannel, HitResult);
          }
        };
      });
    }
    GetWorld()->GetPhysicsScene()->GetPxScene()->unlockRead();

    FTransform ActorTransf = GetTransform();
    ComputeAndSaveDetections(ActorTransf);

    const float HorizontalAngle = carla::geom::Math::ToRadians(
        std::fmod(CurrentHorizontalAngle + AngleDistanceOfTick, Description.HorizontalFov));
    SemanticLidarData.SetHorizontalAngle(HorizontalAngle);
  }
  else if (Description.LidarType == "Solid_state")
  {
    const uint32 ChannelCount = Description.Channels;
    const uint32 PointsToScanWithOneLaser =(Description.PointsPerSecond)/ChannelCount;
    const int CurrentidxPtsOneLaser = SemanticLidarData.GetidxPtsOneLaser();    
    int LastidxPtsOneLaser;
    if (PointsToScanWithOneLaser <= 0)
    {
      UE_LOG(
          LogCarla,
          Warning,
          TEXT("%s: no points requested this frame, try increasing the number of points per second."),
          *GetName());
      return;
    }
    ResetRecordedHits(ChannelCount, PointsToScanWithOneLaser);
    PreprocessRays(ChannelCount, PointsToScanWithOneLaser);
    GetWorld()->GetPhysicsScene()->GetPxScene()->lockRead();
    {
        TRACE_CPUPROFILER_EVENT_SCOPE(ParallelFor);
        ParallelFor(ChannelCount, [&](int32 idxChannel) {
        TRACE_CPUPROFILER_EVENT_SCOPE(ParallelForTask);

        FCollisionQueryParams TraceParams = FCollisionQueryParams(FName(TEXT("Laser_Trace")), true, this);
        TraceParams.bTraceComplex = true;
        TraceParams.bReturnPhysicalMaterial = false;

        for (auto idxPtsOneLaser = 0u; idxPtsOneLaser < PointsToScanWithOneLaser; idxPtsOneLaser++) {
          FHitResult HitResult;
          float VertAngle = Description.vfov[(idxPtsOneLaser+CurrentidxPtsOneLaser)%(Description.pointnums/ChannelCount)];
          float HorizAngle= Description.hfov[(idxPtsOneLaser+CurrentidxPtsOneLaser)%(Description.pointnums/ChannelCount)];
          bool PreprocessResult = RayPreprocessCondition[idxChannel][idxPtsOneLaser];

          if (PreprocessResult && ShootLaser(VertAngle, HorizAngle, HitResult, TraceParams, 0.0)) {
            WritePointAsync(idxChannel, HitResult);
       }
      };
    });
  }
    GetWorld()->GetPhysicsScene()->GetPxScene()->unlockRead();
    
    FTransform ActorTransf = GetTransform();
    ComputeAndSaveDetections(ActorTransf);
    const float HorizontalAngle=carla::geom::Math::ToRadians(
      Description.hfov[(PointsToScanWithOneLaser+CurrentidxPtsOneLaser-1)%(Description.pointnums/ChannelCount)]);

    LastidxPtsOneLaser=(CurrentidxPtsOneLaser+PointsToScanWithOneLaser)%(Description.pointnums/ChannelCount);
    SemanticLidarData.SetHorizontalAngle(HorizontalAngle);
    SemanticLidarData.SetidxPtsOneLaser(LastidxPtsOneLaser);
  }
  else if (Description.LidarType == "Risley_prism")
  {
  const uint32 ChannelCount = Description.Channels;

  float decayTime = Description.Decay; // Point cloud density control parameter
  const uint32 PointsToScanWithOneLaser = FMath::RoundHalfFromZero(float(Description.LivoxSize) * DeltaTime * decayTime);

  if (PointsToScanWithOneLaser <= 0)
  {
	  UE_LOG(
		  LogCarla,
		  Warning,
		  TEXT("%s: no points requested this frame, try increasing the number of points per second."),
		  *GetName());
	  return;
  }

  const float CurrentHorizontalAngle = carla::geom::Math::ToDegrees(
	  SemanticLidarData.GetHorizontalAngle());
  const float AngleDistanceOfTick = Description.RotationFrequency * Description.HorizontalFov * DeltaTime;
  const float AngleDistanceOfLaserMeasure = AngleDistanceOfTick / PointsToScanWithOneLaser;

  ResetRecordedHits(ChannelCount, PointsToScanWithOneLaser);
  PreprocessRays(ChannelCount, PointsToScanWithOneLaser); // MaxPointsPerChannel

  GetWorld()->GetPhysicsScene()->GetPxScene()->lockRead();
  {
	  TRACE_CPUPROFILER_EVENT_SCOPE(ParallelFor);
	  ParallelFor(ChannelCount, [&](int32 idxChannel)
	  {
		  TRACE_CPUPROFILER_EVENT_SCOPE(ParallelForTask);

		  FCollisionQueryParams TraceParams = FCollisionQueryParams(FName(TEXT("Laser_Trace")), true, this);
		  TraceParams.bTraceComplex = true;
		  TraceParams.bReturnPhysicalMaterial = false;

		  int i_count = Description.LivoxCount;

		  int i_limit = i_count + FMath::RoundHalfFromZero(float(Description.LivoxSize) * DeltaTime * decayTime);

      float l_timestamp = 0.0;//for timestamp

		  int RayCheck = 0;
		  for (int i = i_count; i < i_limit; i++)
		  {

			  if (i >= Description.LivoxSize)
			  {
				  i_count = 0;
				  i = 0;
				  i_limit = i_limit - Description.LivoxSize;

          Description.Livox_loop_count++;//for timestamp

				  Description.LivoxCount = i;
				  continue;
			  }

			  FHitResult HitResult;
			  float Azimuth = Description.livox_csv_info[i][1]; // Azimuth angle
			  float Height = Description.livox_csv_info[i][2] - 90; // Height angle

        float simTime = Description.livox_csv_info[i][0]; //for timestamp
        l_timestamp = Description.Livox_loop_count * 4.0 + simTime; //for timestamp

			  const float VertAngle = Height;
			  const float HorizAngle = Azimuth;

			  const bool PreprocessResult = RayPreprocessCondition[idxChannel][RayCheck];
			  ++RayCheck;

			  if (PreprocessResult && ShootLaser(VertAngle, HorizAngle, HitResult, TraceParams, l_timestamp))
			  {
				  WritePointAsync(idxChannel, HitResult);
			  }
			  Description.LivoxCount = i;
		  } });
  }
  GetWorld()->GetPhysicsScene()->GetPxScene()->unlockRead();

  FTransform ActorTransf = GetTransform();
  ComputeAndSaveDetections(ActorTransf);

  const float HorizontalAngle = carla::geom::Math::ToRadians(
	  std::fmod(CurrentHorizontalAngle + AngleDistanceOfTick, Description.HorizontalFov));
  SemanticLidarData.SetHorizontalAngle(HorizontalAngle);
 }
  
}

void ARayCastSemanticLidar::ResetRecordedHits(uint32_t Channels, uint32_t MaxPointsPerChannel) {
  RecordedHits.resize(Channels);

  for (auto& hits : RecordedHits) {
    hits.clear();
    hits.reserve(MaxPointsPerChannel);
  }
}

void ARayCastSemanticLidar::PreprocessRays(uint32_t Channels, uint32_t MaxPointsPerChannel) {

  RayPreprocessCondition.resize(Channels);

  for (auto& conds : RayPreprocessCondition) {
    conds.clear();
    conds.resize(MaxPointsPerChannel);
    std::fill(conds.begin(), conds.end(), true);
  }
}

void ARayCastSemanticLidar::WritePointAsync(uint32_t channel, FHitResult &detection) {
	TRACE_CPUPROFILER_EVENT_SCOPE_STR(__FUNCTION__);
  DEBUG_ASSERT(GetChannelCount() > channel);
  RecordedHits[channel].emplace_back(detection);
}

void ARayCastSemanticLidar::ComputeAndSaveDetections(const FTransform& SensorTransform) {
  // if(Description.NAME!=6)
  if(Description.LidarType!="Solid_state")
  {
    TRACE_CPUPROFILER_EVENT_SCOPE_STR(__FUNCTION__);
    for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel)
      PointsPerChannel[idxChannel] = RecordedHits[idxChannel].size();
    SemanticLidarData.ResetMemory(PointsPerChannel);

    for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel) {
      for (auto& hit : RecordedHits[idxChannel]) {
        FSemanticDetection detection;
        ComputeRawDetection(hit, SensorTransform, detection);
        SemanticLidarData.WritePointSync(detection);
      }
    }

    SemanticLidarData.WriteChannelCount(PointsPerChannel);
  }
  else 
  {
    TRACE_CPUPROFILER_EVENT_SCOPE_STR(__FUNCTION__);
    for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel)
      PointsPerChannel[idxChannel] = RecordedHits[idxChannel].size();
    SemanticLidarData.ResetMemory(PointsPerChannel);

    for (auto idxChannel = 0u; idxChannel < Description.Channels; ++idxChannel) {
      for (auto& hit : RecordedHits[idxChannel]) {
        FSemanticDetection detection;
        ComputeRawDetection(hit, SensorTransform, detection);
        SemanticLidarData.WritePointSync(detection);
      }
    }

    SemanticLidarData.WriteChannelCount(PointsPerChannel);
  }
}


void ARayCastSemanticLidar::ComputeRawDetection(const FHitResult& HitInfo, const FTransform& SensorTransf, FSemanticDetection& Detection) const
{
    const FVector HitPoint = HitInfo.ImpactPoint;
    Detection.point = SensorTransf.Inverse().TransformPosition(HitPoint);
    
    const FVector VecInc = - (HitPoint - SensorTransf.GetLocation()).GetSafeNormal();
    Detection.cos_inc_angle = FVector::DotProduct(VecInc, HitInfo.ImpactNormal);

    const FActorRegistry &Registry = GetEpisode().GetActorRegistry();

    const AActor* actor = HitInfo.Actor.Get();
    Detection.object_idx = 0;
    Detection.object_tag = static_cast<uint32_t>(HitInfo.Component->CustomDepthStencilValue);

    if (actor != nullptr) {

      const FCarlaActor* view = Registry.FindCarlaActor(actor);
      if(view)
        Detection.object_idx = view->GetActorId();

    }
    else {
      UE_LOG(LogCarla, Warning, TEXT("Actor not valid %p!!!!"), actor);
    }
}


bool ARayCastSemanticLidar::ShootLaser(const float VerticalAngle, const float HorizontalAngle, FHitResult& HitResult, FCollisionQueryParams& TraceParams, const float l_timestamp) const
{
  TRACE_CPUPROFILER_EVENT_SCOPE_STR(__FUNCTION__);

  FHitResult HitInfo(ForceInit);
  FHitResult ReflectHitInfo(ForceInit);

  FTransform ActorTransf = GetTransform();
  FVector LidarBodyLoc = ActorTransf.GetLocation();
  FRotator LidarBodyRot = ActorTransf.Rotator();

  FRotator LaserRot (VerticalAngle, HorizontalAngle, 0);  // float InPitch, float InYaw, float InRoll
  FRotator ResultRot = UKismetMathLibrary::ComposeRotators(
    LaserRot,
    LidarBodyRot
  );

  const auto Range = Description.Range;
  FVector EndTrace = Range * UKismetMathLibrary::GetForwardVector(ResultRot) + LidarBodyLoc;

  GetWorld()->ParallelLineTraceSingleByChannel(
    HitInfo,
    LidarBodyLoc,
    EndTrace,
    ECC_GameTraceChannel2,
    TraceParams,
    FCollisionResponseParams::DefaultResponseParam
  );

  HitInfo.Time = l_timestamp;

  if (Description.EnableGhost && HitInfo.bBlockingHit) //enableghost and hit
  {
    std::cout<<"Enable Lidar Ghost Character!"<<std::endl;
    float ReflectRange=0.0f;
    float Incidencedistance,distance=0.0f;
    int objtag=0;
    HitResult = HitInfo;
    FVector FirstHitPoint,SecondHitPoint,HitPoint,ExitPoint,UnitPoint,Incidencevector,Exitvector;
    objtag=static_cast<uint32_t>(HitResult.Component->CustomDepthStencilValue);
    FirstHitPoint=HitResult.ImpactPoint;
    ExitPoint=FirstHitPoint+HitResult.ImpactNormal * 0.1f;
    std::vector<float> Reflectivity={0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.80,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.80,0.0};

    if(Reflectivity[objtag]>0.5)
    {
      float random=0.0f;
      std::srand(time(NULL));
      random=std::rand()%(999+1)/(float)(999+1);
      if(random>0.5)
      {
        Incidencedistance=sqrt(pow((FirstHitPoint.X-LidarBodyLoc.X),2)+pow((FirstHitPoint.Y-LidarBodyLoc.Y),2)+pow((FirstHitPoint.Z-LidarBodyLoc.Z),2));
        Incidencevector=FirstHitPoint - LidarBodyLoc;
        UnitPoint.X=Incidencevector.X/Incidencedistance;
        UnitPoint.Y=Incidencevector.Y/Incidencedistance;
        UnitPoint.Z=Incidencevector.Z/Incidencedistance;
        ReflectRange=(Range-Incidencedistance)*0.8f;
        // Exitvector=UnitPoint-2*FVector::DotProduct(UnitPoint,HitInfo.ImpactNormal)*HitInfo.ImpactNormal; 
        Exitvector=FMath::GetReflectionVector(UnitPoint, HitResult.ImpactNormal);
        FVector EndTrace1 = ReflectRange * Exitvector + ExitPoint;
        GetWorld()->ParallelLineTraceSingleByChannel(
            ReflectHitInfo,
            ExitPoint,
            EndTrace1,
            ECC_GameTraceChannel2,
            TraceParams,
            FCollisionResponseParams::DefaultResponseParam
        );
        if (ReflectHitInfo.bBlockingHit) 
        {
            // DrawDe1bugLine(GetWorld(), ExitPoint, EndTrace1, FColor::Green, false, 4.f);
            int objtag1=0;
            objtag1=static_cast<uint32_t>(ReflectHitInfo.Component->CustomDepthStencilValue);
            SecondHitPoint=ReflectHitInfo.ImpactPoint;
            distance=sqrt(pow((SecondHitPoint.X-FirstHitPoint.X),2)+pow((SecondHitPoint.Y-FirstHitPoint.Y),2)+pow((SecondHitPoint.Z-FirstHitPoint.Z),2));
            SecondHitPoint.X=FirstHitPoint.X+Incidencevector.X*distance/Incidencedistance;
            SecondHitPoint.Y=FirstHitPoint.Y+Incidencevector.Y*distance/Incidencedistance;
            SecondHitPoint.Z=FirstHitPoint.Z+Incidencevector.Z*distance/Incidencedistance;         
            HitPoint=SecondHitPoint;
            HitResult=ReflectHitInfo;
            HitResult.ImpactPoint=HitPoint;
            return true;
        } 
        else 
        {
            return false;
        }
      }
      else
      {
        HitResult = HitInfo;
        return true;
      }

    }
    else
    {
      HitResult = HitInfo;
      return true;
    }

  }
  else if (HitInfo.bBlockingHit) //unableghost but hit
  {
    HitResult = HitInfo;
    return true;
  }
  else  //unableghost and unhit
  {
    return false;
  }
}
