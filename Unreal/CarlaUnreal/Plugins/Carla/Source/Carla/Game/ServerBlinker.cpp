// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Game/ServerBlinker.h"
#include "Carla.h"

#include "Carla/Game/ServerWarning.h"

#include <util/ue-header-guard-begin.h>
#include "HAL/PlatformTime.h"
#include "Math/RandomStream.h"
#include <util/ue-header-guard-end.h>

namespace
{
  /// One flash, on then off (s): about 85 a minute.
  constexpr double BlinkerPeriodSeconds = 0.7;
  /// The part of a flash the lamp is lit.
  constexpr double BlinkerLitFraction = 0.5;

  /// A relay click: two damped resonances over a burst of noise.
  TArray<int16> MakeBlinkerClick(float HighHz, float LowHz, float DecaySeconds, float Volume, int32 Seed)
  {
    const int32 Rate = FServerChime::GetSampleRate();
    const int32 Count = FMath::RoundToInt(DecaySeconds * 8.0f * Rate);
    FRandomStream Noise(Seed);
    TArray<int16> Samples;
    Samples.Reserve(Count);
    for (int32 Index = 0; Index < Count; ++Index)
    {
      const float Time = static_cast<float>(Index) / Rate;
      const float Envelope = FMath::Exp(-Time / DecaySeconds);
      // A short noise burst for the contact's snap, the rest is the ring.
      const float Snap = FMath::Exp(-Time / (DecaySeconds * 0.3f)) * Noise.FRandRange(-1.0f, 1.0f);
      const float Ring = 0.6f * FMath::Sin(2.0f * PI * HighHz * Time) + 0.4f * FMath::Sin(2.0f * PI * LowHz * Time);
      const float Value = Volume * Envelope * (0.55f * Ring + 0.45f * Snap);
      Samples.Add(static_cast<int16>(FMath::Clamp(Value, -1.0f, 1.0f) * 32767.0f));
    }
    return Samples;
  }
} // namespace

FServerBlinker::FServerBlinker(AActor &Owner)
  : TickSound(MakeUnique<FServerChime>(Owner, MakeBlinkerClick(2800.0f, 1100.0f, 0.004f, 0.55f, 1))),
    TockSound(MakeUnique<FServerChime>(Owner, MakeBlinkerClick(1700.0f, 650.0f, 0.005f, 0.32f, 2))) {}

FServerBlinker::~FServerBlinker() = default;

void FServerBlinker::Tick(bool bInLeft, bool bInRight)
{
  const double Now = FPlatformTime::Seconds();
  const bool bWasOn = bLeft || bRight;
  const bool bOn = bInLeft || bInRight;
  // A new signal (or a switch to the other side) starts a flash, lit.
  if (bOn && (!bWasOn || bInLeft != bLeft || bInRight != bRight))
  {
    Start = Now;
  }
  bLeft = bInLeft;
  bRight = bInRight;

  bool bNowLit = false;
  if (bOn)
  {
    const double Phase = FMath::Fmod(Now - Start, BlinkerPeriodSeconds);
    bNowLit = Phase < BlinkerPeriodSeconds * BlinkerLitFraction;
  }
  if (bNowLit != bLit)
  {
    (bNowLit ? TickSound : TockSound)->Play();
  }
  bLit = bNowLit;
}
