// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <util/ue-header-guard-begin.h>
#include "CoreMinimal.h"
#include "UObject/Object.h"
#include "UObject/StrongObjectPtr.h"
#include <util/ue-header-guard-end.h>

#include "ServerDriveReport.generated.h"

class ACarlaWheeledVehicle;
class ATrafficLightBase;
class FServerDriveReport;
class UCarlaEpisode;

/// Forwards the player vehicle's OnActorHit, a dynamic delegate, to the
/// (non-UObject) drive report.
UCLASS()
class CARLA_API UServerDriveHitListener : public UObject
{
  GENERATED_BODY()

public:

  FServerDriveReport *Report = nullptr;

  UFUNCTION()
  void OnHit(AActor *SelfActor, AActor *OtherActor, FVector NormalImpulse, const FHitResult &Hit);
};

/// Records one drive of the server player's vehicle (from spawn to the end of
/// the episode, or to the vehicle's destruction) and writes two CSV reports:
///
///   drive_<start>_summary.csv     metric,value: start and end, duration,
///                                 distance, average speed, the violation
///                                 count, and a count per event type
///   drive_<start>_everything.csv  one row per event: time, drive_time_s,
///                                 event, severity, details, other_actor,
///                                 speed_kmh, speed_limit_kmh, x, y
///
/// Events: vehicle spawned; collisions with vehicles, pedestrians and other
/// objects (one per contact, with its peak impulse and duration); speeding
/// (one per stretch above the limit, with its peak); running a red light;
/// crossing a solid line; changing lanes over a broken line without a turn
/// signal. Locations are CARLA's (meters).
///
/// Both files are rewritten every few seconds while driving, so a crash or a
/// killed process loses at most that much, and once more when the drive ends.
class FServerDriveReport
{
public:

  /// The folder the reports go to: -server-reports=DIR, else Reports on the
  /// user's desktop.
  static FString GetDirectory();

  FServerDriveReport(UCarlaEpisode &Episode, ACarlaWheeledVehicle &Vehicle, const FString &BlueprintId);
  ~FServerDriveReport();

  FServerDriveReport(const FServerDriveReport &) = delete;
  FServerDriveReport &operator=(const FServerDriveReport &) = delete;

  /// Samples the vehicle; call once per world tick.
  void Tick();

  int32 GetViolationCount() const { return ViolationCount; }

  /// Game time of the last lane change made with a turn signal on, or a
  /// negative value: the player uses it to cancel the signal afterwards.
  double GetLastSignalledLaneChange() const { return LastSignalledLaneChange; }

  void OnHit(AActor *OtherActor, const FVector &NormalImpulse, const FHitResult &Hit);

private:

  struct FEvent
  {
    FDateTime Wall;
    double DriveTime = 0.0;
    FString Name;
    bool bViolation = false;
    FString Details;
    FString OtherActor;
    float SpeedKmh = -1.0f;
    float SpeedLimitKmh = -1.0f;
    FVector Location = FVector::ZeroVector; // cm
  };

  struct FContact
  {
    FEvent Event;
    double LastHit = 0.0;
    float PeakImpulse = 0.0f; // N s
    bool bReportable = false;
  };

  double GameTime() const;
  float SpeedKmh() const;
  float SpeedLimitKmh() const;
  FEvent MakeEvent(const TCHAR *Name, bool bViolation) const;
  void Add(FEvent Event);

  void CloseContacts(bool bAll);
  void TrackSpeeding(bool bFinish);
  void TrackLaneMarkings();
  void TrackTrafficLight();

  FString DescribeActor(const AActor &Actor) const;
  void Write();

  UCarlaEpisode &Episode;
  TWeakObjectPtr<ACarlaWheeledVehicle> Vehicle;
  TStrongObjectPtr<UServerDriveHitListener> Listener;

  FString SummaryPath;
  FString EverythingPath;
  FDateTime StartWall;
  double StartTime = 0.0;
  double LastSample = 0.0;
  double LastWrite = 0.0;
  double DistanceMeters = 0.0;
  bool bDirty = true;

  TArray<FEvent> Events;
  int32 ViolationCount = 0;

  TMap<TWeakObjectPtr<AActor>, FContact> Contacts;

  bool bSpeeding = false;
  FEvent SpeedingEvent;
  double SpeedingStart = 0.0;
  float SpeedingPeak = 0.0f;

  bool bHaveCorners = false;
  FVector Corners[4];
  double LastSolidCrossing = -100.0;
  double LastBrokenCrossing = -100.0;
  double LastSignalledLaneChange = -1.0;

  TWeakObjectPtr<ATrafficLightBase> TrafficLight;
};
