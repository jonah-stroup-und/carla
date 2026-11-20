// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Game/ServerWarning.h"
#include "Carla.h"

#include <util/ue-header-guard-begin.h>
#include "Components/AudioComponent.h"
#include "GameFramework/Actor.h"
#include "Sound/SoundWaveProcedural.h"
#include <util/ue-header-guard-end.h>

namespace
{
  constexpr int32 ChimeRate = 48000;
  /// Fade in and out of each beep, so it does not click.
  constexpr float ChimeFadeSeconds = 0.005f;
}

bool ParseServerWarningMode(const FString &Text, EServerWarningMode &Out)
{
  if (Text.Equals(TEXT("off"), ESearchCase::IgnoreCase) || Text == TEXT("0"))
  {
    Out = EServerWarningMode::Off;
  }
  else if (Text.Equals(TEXT("visual"), ESearchCase::IgnoreCase) || Text == TEXT("1"))
  {
    Out = EServerWarningMode::Visual;
  }
  else if (Text.Equals(TEXT("audio"), ESearchCase::IgnoreCase) || Text.Equals(TEXT("visual+audio"), ESearchCase::IgnoreCase) ||
      Text == TEXT("2"))
  {
    Out = EServerWarningMode::VisualAudio;
  }
  else
  {
    return false;
  }
  return true;
}

const TCHAR *GetServerWarningModeName(EServerWarningMode Mode)
{
  switch (Mode)
  {
    case EServerWarningMode::Off:         return TEXT("off");
    case EServerWarningMode::Visual:      return TEXT("visual");
    case EServerWarningMode::VisualAudio: return TEXT("visual + audio");
    default:                              return TEXT("?");
  }
}

EServerWarningMode NextServerWarningMode(EServerWarningMode Mode)
{
  return static_cast<EServerWarningMode>((static_cast<uint8>(Mode) + 1) % static_cast<uint8>(EServerWarningMode::Count));
}

FServerChime::FServerChime(AActor &Owner, float Hz, int32 Beeps, float BeepSeconds, float GapSeconds, float Volume)
{
  USoundWaveProcedural *NewWave = NewObject<USoundWaveProcedural>();
  NewWave->SetSampleRate(ChimeRate);
  NewWave->NumChannels = 1;
  NewWave->Duration = INDEFINITELY_LOOPING_DURATION;
  NewWave->SoundGroup = SOUNDGROUP_Default;
  NewWave->bLooping = false;
  Wave.Reset(NewWave);

  UAudioComponent *NewComponent = NewObject<UAudioComponent>(&Owner);
  NewComponent->bAutoActivate = false;
  NewComponent->bAllowSpatialization = false;
  NewComponent->bIsUISound = true;
  NewComponent->SetSound(NewWave);
  NewComponent->RegisterComponent();
  NewComponent->Play();
  Component = NewComponent;

  const int32 Beep = FMath::RoundToInt(BeepSeconds * ChimeRate);
  const int32 Gap = FMath::RoundToInt(GapSeconds * ChimeRate);
  const int32 Fade = FMath::Max(1, FMath::RoundToInt(ChimeFadeSeconds * ChimeRate));
  for (int32 Index = 0; Index < Beeps; ++Index)
  {
    for (int32 Sample = 0; Sample < Beep; ++Sample)
    {
      const float Envelope = FMath::Min3(1.0f, Sample / static_cast<float>(Fade), (Beep - Sample) / static_cast<float>(Fade));
      const float Value = Volume * Envelope * FMath::Sin(2.0f * PI * Hz * Sample / ChimeRate);
      Samples.Add(static_cast<int16>(Value * 32767.0f));
    }
    Samples.AddZeroed(Gap);
  }
}

FServerChime::~FServerChime()
{
  if (Component.IsValid())
  {
    Component->Stop();
    Component->DestroyComponent();
  }
}

void FServerChime::Play()
{
  USoundWaveProcedural *Sound = Wave.Get();
  if (Sound == nullptr || Sound->GetAvailableAudioByteCount() > 0)
  {
    return; // one is still playing
  }
  if (Component.IsValid() && !Component->IsPlaying())
  {
    Component->Play();
  }
  Sound->QueueAudio(reinterpret_cast<const uint8 *>(Samples.GetData()), Samples.Num() * sizeof(int16));
}
