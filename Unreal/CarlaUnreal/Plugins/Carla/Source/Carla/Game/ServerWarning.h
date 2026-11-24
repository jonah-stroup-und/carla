// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include <util/ue-header-guard-begin.h>
#include "CoreMinimal.h"
#include "UObject/StrongObjectPtr.h"
#include <util/ue-header-guard-end.h>

class AActor;
class UAudioComponent;
class USoundWaveProcedural;

/// How a driver aid (blind-spot monitor, A-pillar monitor) warns.
enum class EServerWarningMode : uint8
{
  Off,          ///< nothing shown or heard
  Visual,       ///< its indicator lights
  VisualAudio,  ///< its indicator lights, and a chime sounds
  Count
};

/// "off", "visual" or "audio" (also "visual+audio", 0, 1, 2); false when unknown.
bool ParseServerWarningMode(const FString &Text, EServerWarningMode &Out);

/// "off", "visual" or "visual + audio".
const TCHAR *GetServerWarningModeName(EServerWarningMode Mode);

EServerWarningMode NextServerWarningMode(EServerWarningMode Mode);

/// A warning chime: a few short sine beeps, generated rather than an asset,
/// played by a 2D audio component that runs from construction, silent until
/// Play() queues a chime's samples.
class FServerChime
{
public:

  FServerChime(AActor &Owner, float Hz, int32 Beeps, float BeepSeconds, float GapSeconds, float Volume);

  /// Any generated sound: mono 16-bit samples at GetSampleRate().
  FServerChime(AActor &Owner, TArray<int16> InSamples);

  ~FServerChime();

  static int32 GetSampleRate();

  FServerChime(const FServerChime &) = delete;
  FServerChime &operator=(const FServerChime &) = delete;

  /// Sounds the chime, unless one is still playing.
  void Play();

private:

  void CreatePlayer(AActor &Owner);

  TStrongObjectPtr<USoundWaveProcedural> Wave;
  TWeakObjectPtr<UAudioComponent> Component;
  TArray<int16> Samples;
};
