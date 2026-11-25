// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Game/ServerCarSound.h"
#include "Carla.h"

#include "Carla/Game/ServerWarning.h"
#include "Carla/Vehicle/CarlaWheeledVehicle.h"

#include <util/ue-header-guard-begin.h>
#include "ChaosWheeledVehicleMovementComponent.h"
#include "Components/AudioComponent.h"
#include "Containers/Queue.h"
#include "Dom/JsonObject.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformTime.h"
#include "Misc/CommandLine.h"
#include "Misc/FileHelper.h"
#include "Misc/Parse.h"
#include "Misc/Paths.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Sound/SoundWaveProcedural.h"
#include <util/ue-header-guard-end.h>

#include <atomic>

DEFINE_LOG_CATEGORY_STATIC(LogCarlaServerCarSound, Log, All);

namespace
{
  constexpr double CarSoundReloadCheckSeconds = 0.5;
  constexpr double CarSoundDebugSeconds = 0.5;
  /// Parameters glide to new values over this (s), so they do not zip.
  constexpr float CarSoundSmoothSeconds = 0.04f;
  /// Filters are retuned every this many samples.
  constexpr int32 CarSoundBlock = 32;
  constexpr int32 CarSoundMaxCrashes = 6;

  /// Cylinders of the engine (a four-stroke fires half of them a turn).
  constexpr float CarSoundCylinders = 6.0f;

  /// Below this impulse (N s) a contact is silent; this one is a full crash.
  constexpr float CarSoundMinImpulse = 40.0f;
  constexpr float CarSoundFullImpulse = 2500.0f;
  /// A contact this long after the last one is a new crash (s).
  constexpr double CarSoundNewContactSeconds = 0.3;

  /// Tyre squeal: from this slip angle (deg), full this much beyond it.
  constexpr float CarSoundSlipAngleStart = 7.0f;
  constexpr float CarSoundSlipAngleRange = 10.0f;
  /// And from this skid (Chaos' magnitude), full this much beyond it.
  constexpr float CarSoundSkidStart = 150.0f;
  constexpr float CarSoundSkidRange = 400.0f;

  /// A state-variable filter (Simper's trapezoidal form): stable however it
  /// is retuned.
  struct FCarSoundFilter
  {
    float A1 = 0.0f, A2 = 0.0f, A3 = 0.0f, K = 1.0f;
    float Ic1 = 0.0f, Ic2 = 0.0f;
    float Low = 0.0f, Band = 0.0f;

    void Set(float Hz, float Q, float Rate)
    {
      Hz = FMath::Clamp(Hz, 20.0f, Rate * 0.45f);
      const float G = FMath::Tan(PI * Hz / Rate);
      K = 1.0f / FMath::Max(Q, 0.1f);
      A1 = 1.0f / (1.0f + G * (G + K));
      A2 = G * A1;
      A3 = G * A2;
    }

    void Run(float In)
    {
      const float V3 = In - Ic2;
      const float V1 = A1 * Ic1 + A2 * V3;
      const float V2 = Ic2 + A2 * Ic1 + A3 * V3;
      Ic1 = 2.0f * V1 - Ic1;
      Ic2 = 2.0f * V2 - Ic2;
      Low = V2;
      Band = V1;
    }

    float LowPass(float In) { Run(In); return Low; }
    float HighPass(float In) { Run(In); return In - K * Band - Low; }
    /// Band-pass with a peak gain of 1.
    float BandPass(float In) { Run(In); return Band * K; }
  };

  /// A smoothed parameter: the game thread sets the target, the audio
  /// thread glides to it.
  struct FCarSoundParam
  {
    std::atomic<float> Target{0.0f};
    float Value = 0.0f;

    void Set(float In) { Target.store(In, std::memory_order_relaxed); }
    float Step(float Alpha)
    {
      Value += (Target.load(std::memory_order_relaxed) - Value) * Alpha;
      return Value;
    }
  };

  float CarSoundSoftClip(float X)
  {
    X = FMath::Clamp(X, -3.0f, 3.0f);
    return X * (27.0f + X * X) / (27.0f + 9.0f * X * X); // tanh, near enough
  }
} // namespace

class FServerCarSound::FSynth : public TSharedFromThis<FSynth, ESPMode::ThreadSafe>
{
public:

  FCarSoundParam Rpm, RpmNorm, Load, Speed, Squeal, Scrape, Interior;
  FCarSoundParam Volume, EngineLevel, RoadLevel, TyresLevel, CollisionLevel;
  TQueue<float, EQueueMode::Spsc> Crashes;

  FSynth() : Rate(static_cast<float>(FServerChime::GetSampleRate()))
  {
    for (int32 Harmonic = 0; Harmonic < Harmonics; ++Harmonic)
    {
      HarmonicWeight[Harmonic] = 1.0f / FMath::Pow(static_cast<float>(Harmonic + 1), 1.3f);
    }
  }

  /// The audio thread asks for more.
  void OnUnderflow(USoundWaveProcedural *Wave, int32 SamplesRequired)
  {
    if (Wave == nullptr || SamplesRequired <= 0)
    {
      return;
    }
    Buffer.SetNumUninitialized(SamplesRequired);
    Generate(Buffer.GetData(), SamplesRequired);
    Wave->QueueAudio(reinterpret_cast<const uint8 *>(Buffer.GetData()), SamplesRequired * sizeof(int16));
  }

private:

  /// A crash, heard from inside the car: a dull, muffled pound.
  struct FCrash
  {
    float Time = 0.0f;
    float Strength = 0.0f;
    float ThudPhase = 0.0f;
    FCarSoundFilter Body, Muffle;
  };

  float Rate;
  TArray<int16> Buffer;
  uint32 Seed = 0x9E3779B9u;

  /// The engine's harmonics of its firing frequency, each drifting a little
  /// in level so the hum is alive but never pulses.
  static constexpr int32 Harmonics = 8;
  float EnginePhase = 0.0f;
  float HarmonicLevel[Harmonics] = {};
  float HarmonicDrift[Harmonics] = {};
  float HarmonicWeight[Harmonics] = {};
  FCarSoundFilter EngineTone, EngineBody, EngineHighPass;

  FCarSoundFilter RoadBody, RoadRumble, RoadHiss, SquealA, SquealB, ScrapeLow, ScrapeHigh;
  float ScrapeGrain = 1.0f;
  float SquealWobble = 0.0f;
  int32 BlockLeft = 0;

  TArray<FCrash> Active;

  float Noise()
  {
    Seed ^= Seed << 13;
    Seed ^= Seed >> 17;
    Seed ^= Seed << 5;
    return static_cast<float>(Seed) * (2.0f / 4294967295.0f) - 1.0f;
  }

  void StartCrash(float Strength)
  {
    if (Active.Num() >= CarSoundMaxCrashes)
    {
      Active.RemoveAt(0);
    }
    FCrash &Crash = Active.AddDefaulted_GetRef();
    Crash.Strength = Strength;
    Crash.Body.Set(160.0f + 60.0f * FMath::Abs(Noise()), 0.7f, Rate);
    // The car's body between the driver and the hit takes the top off.
    Crash.Muffle.Set(320.0f, 0.6f, Rate);
  }

  void Generate(int16 *Out, int32 Count)
  {
    const float Alpha = 1.0f - FMath::Exp(-1.0f / (Rate * CarSoundSmoothSeconds));
    float Pending = 0.0f;
    while (Crashes.Dequeue(Pending))
    {
      StartCrash(Pending);
    }

    for (int32 Index = 0; Index < Count; ++Index)
    {
      const float R = Rpm.Step(Alpha);
      const float RN = RpmNorm.Step(Alpha);
      const float L = Load.Step(Alpha);
      const float V = Speed.Step(Alpha);
      const float Sq = Squeal.Step(Alpha);
      const float Sc = Scrape.Step(Alpha);
      const float In = Interior.Step(Alpha);
      const float Master = Volume.Step(Alpha);
      const float EngineGain = EngineLevel.Step(Alpha);
      const float RoadGain = RoadLevel.Step(Alpha);
      const float TyresGain = TyresLevel.Step(Alpha);
      const float CrashGain = CollisionLevel.Step(Alpha);

      if (--BlockLeft <= 0)
      {
        BlockLeft = CarSoundBlock;
        // Muffled at low revs; throttle and revs open it up, the cabin closes
        // it down.
        EngineTone.Set((200.0f + 900.0f * L * (0.3f + RN) + 1400.0f * RN) * (1.0f - 0.4f * In), 0.6f, Rate);
        EngineBody.Set(280.0f, 1.0f, Rate);
        EngineHighPass.Set(50.0f, 0.7f, Rate);
        for (int32 Harmonic = 0; Harmonic < Harmonics; ++Harmonic)
        {
          HarmonicDrift[Harmonic] = FMath::Clamp(HarmonicDrift[Harmonic] + 0.02f * Noise(), -0.15f, 0.15f);
        }
        RoadBody.Set(150.0f + 18.0f * V, 0.6f, Rate);
        RoadRumble.Set(85.0f, 1.5f, Rate);
        RoadHiss.Set(2600.0f + 30.0f * V, 0.8f, Rate);
        SquealWobble += 2.0f * PI * 5.0f * CarSoundBlock / Rate;
        SquealA.Set(1150.0f * (1.0f + 0.03f * FMath::Sin(SquealWobble)), 12.0f, Rate);
        SquealB.Set(1720.0f * (1.0f + 0.02f * FMath::Sin(SquealWobble * 1.3f)), 14.0f, Rate);
        ScrapeLow.Set(300.0f, 1.0f, Rate);
        ScrapeHigh.Set(800.0f, 1.5f, Rate);
        ScrapeGrain = 0.6f + 0.4f * FMath::Abs(Noise());
      }

      // -- Engine: the harmonics of its firing, a smooth hum with no ------
      // -- separate pulses, muffled; no noise, so it never hisses. ---------
      const float FiringHz = FMath::Max(R, 300.0f) / 60.0f * CarSoundCylinders * 0.5f;
      EnginePhase = FMath::Fmod(EnginePhase + FiringHz / Rate, 1.0f);
      float Hum = 0.0f;
      for (int32 Harmonic = 0; Harmonic < Harmonics; ++Harmonic)
      {
        HarmonicLevel[Harmonic] += (HarmonicDrift[Harmonic] - HarmonicLevel[Harmonic]) * 0.0005f;
        const float Order = static_cast<float>(Harmonic + 1);
        // Fixed offsets keep the harmonics from lining up into a pulse.
        Hum += (1.0f + HarmonicLevel[Harmonic]) * HarmonicWeight[Harmonic] *
            FMath::Sin(2.0f * PI * (Order * EnginePhase + 0.37f * Order * Order));
      }
      const float Tone = 0.8f * EngineTone.LowPass(Hum) + 0.25f * EngineBody.BandPass(Hum);
      const float Engine = EngineHighPass.HighPass(Tone);
      float Mix = Engine * (0.15f + 0.35f * L + 0.25f * RN) * 0.35f * EngineGain;

      // -- Road -------------------------------------------------------------
      const float RoadAmount = FMath::Pow(FMath::Clamp(V / 30.0f, 0.0f, 1.2f), 1.1f);
      const float White = Noise();
      Mix += RoadGain * RoadAmount * (0.35f * RoadBody.LowPass(White) + 0.25f * RoadRumble.BandPass(White) +
          0.05f * (1.0f - 0.8f * In) * RoadHiss.BandPass(White));

      // -- Tyres ---------------------------------------------------------------
      if (Sq > 1.0e-3f)
      {
        const float Hiss = Noise();
        Mix += TyresGain * Sq * (0.9f * SquealA.BandPass(Hiss) + 0.6f * SquealB.BandPass(Hiss));
      }

      // -- Collisions ----------------------------------------------------------
      if (Sc > 1.0e-3f)
      {
        const float Grit = Noise() * ScrapeGrain;
        Mix += CrashGain * Sc * (0.25f * ScrapeLow.BandPass(Grit) + 0.1f * ScrapeHigh.BandPass(Grit));
      }
      for (int32 Crash = Active.Num() - 1; Crash >= 0; --Crash)
      {
        FCrash &Hit = Active[Crash];
        const float T = Hit.Time;
        // A pound: a soft few-ms attack (no click), a low body that sags in
        // pitch, gone in a tenth of a second, through the muffling.
        const float Envelope = FMath::Min(T / 0.004f, 1.0f) * FMath::Exp(-T / 0.07f);
        Hit.ThudPhase = FMath::Fmod(Hit.ThudPhase + (48.0f + 30.0f * FMath::Exp(-T / 0.03f)) / Rate, 1.0f);
        const float Pound = 0.7f * FMath::Sin(2.0f * PI * Hit.ThudPhase) + 1.2f * Hit.Body.LowPass(Noise());
        Mix += CrashGain * Hit.Strength * 0.4f * Hit.Muffle.LowPass(Pound * Envelope);
        Hit.Time += 1.0f / Rate;
        if (Hit.Time > 0.4f)
        {
          Active.RemoveAt(Crash);
        }
      }

      Out[Index] = static_cast<int16>(CarSoundSoftClip(Mix * Master) * 32000.0f);
    }
  }
};

FServerCarSound::FServerCarSound(ACarlaWheeledVehicle &InVehicle)
  : Vehicle(&InVehicle),
    Synth(MakeShared<FSynth, ESPMode::ThreadSafe>())
{
  FString Path;
  ConfigPath = FParse::Value(FCommandLine::Get(), TEXT("-server-sound="), Path) && !Path.IsEmpty()
      ? FPaths::ConvertRelativePathToFull(Path)
      : FPaths::ConvertRelativePathToFull(FPaths::ProjectConfigDir() / TEXT("ServerSound.json"));
  Reload();

  USoundWaveProcedural *NewWave = NewObject<USoundWaveProcedural>();
  NewWave->SetSampleRate(FServerChime::GetSampleRate());
  NewWave->NumChannels = 1;
  NewWave->Duration = INDEFINITELY_LOOPING_DURATION;
  NewWave->SoundGroup = SOUNDGROUP_Default;
  NewWave->bLooping = false;
  // Generated on the audio thread as it plays; the delegate holds the
  // synth only while it runs, so the car can go at any time.
  NewWave->OnSoundWaveProceduralUnderflow.BindThreadSafeSP(Synth.ToSharedRef(), &FSynth::OnUnderflow);
  Wave.Reset(NewWave);

  UAudioComponent *NewComponent = NewObject<UAudioComponent>(&InVehicle);
  NewComponent->bAutoActivate = false;
  NewComponent->bAllowSpatialization = false;
  NewComponent->bIsUISound = true;
  NewComponent->SetSound(NewWave);
  NewComponent->RegisterComponent();
  NewComponent->Play();
  Component = NewComponent;
}

FServerCarSound::~FServerCarSound()
{
  if (Component.IsValid())
  {
    Component->Stop();
    Component->DestroyComponent();
  }
}

void FServerCarSound::AddImpact(float Impulse)
{
  FrameImpulse += Impulse;
}

void FServerCarSound::Tick(float DeltaSeconds, bool bInterior)
{
  const double Now = FPlatformTime::Seconds();
  if (Now - LastReloadCheck >= CarSoundReloadCheckSeconds)
  {
    LastReloadCheck = Now;
    if (IFileManager::Get().GetTimeStamp(*ConfigPath) != ConfigStamp)
    {
      Reload();
    }
  }
  ACarlaWheeledVehicle *Car = Vehicle.Get();
  UChaosWheeledVehicleMovementComponent *Movement = Car ? Car->GetChaosWheeledVehicleMovementComponent() : nullptr;
  if (Movement == nullptr)
  {
    return;
  }

  const float Idle = FMath::Max(Movement->EngineSetup.EngineIdleRPM, 300.0f);
  const float Max = FMath::Max(Movement->EngineSetup.MaxRPM, Idle + 500.0f);
  const float Rpm = FMath::Clamp(Movement->GetEngineRotationSpeed(), Idle, Max * 1.05f);
  const float Throttle = FMath::Clamp(Car->GetVehicleControl().Throttle, 0.0f, 1.0f);
  const float Speed = Car->GetVelocity().Size() / 100.0f; // m/s

  // The tyres: the worst sliding one, while the car is moving.
  float SlipAngle = 0.0f;
  float Skid = 0.0f;
  float Slip = 0.0f;
  for (int32 Index = 0; Index < Movement->Wheels.Num(); ++Index)
  {
    const FWheelStatus &Status = Movement->GetWheelState(Index);
    if (!Status.bInContact)
    {
      continue;
    }
    SlipAngle = FMath::Max(SlipAngle, FMath::Abs(FMath::RadiansToDegrees(Status.SlipAngle)));
    Skid = FMath::Max(Skid, Status.bIsSkidding ? Status.SkidMagnitude : 0.0f);
    Slip = FMath::Max(Slip, Status.SlipMagnitude);
  }
  float Squeal = 0.0f;
  if (Speed > 2.0f)
  {
    Squeal = FMath::Max(FMath::Clamp((SlipAngle - CarSoundSlipAngleStart) / CarSoundSlipAngleRange, 0.0f, 1.0f),
        FMath::Clamp((Skid - CarSoundSkidStart) / CarSoundSkidRange, 0.0f, 1.0f));
  }

  // A contact after a quiet spell, or a much harder hit during one, is a
  // crash; a contact that goes on while the car moves is a scrape.
  if (FrameImpulse >= CarSoundMinImpulse)
  {
    if (Now - LastContact > CarSoundNewContactSeconds || FrameImpulse > 3.0f * FMath::Max(ContactLevel, 100.0f))
    {
      Synth->Crashes.Enqueue(FMath::Sqrt(FMath::Clamp(FrameImpulse / CarSoundFullImpulse, 0.02f, 1.0f)));
    }
    ContactLevel = FMath::Lerp(ContactLevel, FrameImpulse, 0.3f);
    LastContact = Now;
  }
  else if (Now - LastContact > 0.15)
  {
    ContactLevel = 0.0f;
  }
  FrameImpulse = 0.0f;
  const float Scrape = ContactLevel > 0.0f
      ? FMath::Clamp(ContactLevel / 600.0f, 0.0f, 1.0f) * FMath::Clamp(Speed / 4.0f, 0.0f, 1.0f)
      : 0.0f;

  Synth->Rpm.Set(Rpm);
  Synth->RpmNorm.Set((Rpm - Idle) / (Max - Idle));
  Synth->Load.Set(Throttle);
  Synth->Speed.Set(Speed);
  Synth->Squeal.Set(Squeal);
  Synth->Scrape.Set(Scrape);
  Synth->Interior.Set(bInterior ? 1.0f : 0.0f);

  if (bDebug && Now - LastDebugLog >= CarSoundDebugSeconds)
  {
    LastDebugLog = Now;
    UE_LOG(LogCarlaServerCarSound, Log,
        TEXT("Sound: rpm %.0f (idle %.0f, max %.0f) gear %d throttle %.2f speed %.1f m/s slip angle %.1f deg skid %.0f slip %.0f squeal %.2f scrape %.2f"),
        Rpm, Idle, Max, Movement->GetCurrentGear(), Throttle, Speed, SlipAngle, Skid, Slip, Squeal, Scrape);
  }
}

void FServerCarSound::Reload()
{
  ConfigStamp = IFileManager::Get().GetTimeStamp(*ConfigPath);
  float Volume = 0.8f, Engine = 1.0f, Road = 1.0f, Tyres = 1.0f, Collisions = 1.0f;
  bool bNewDebug = false;
  FString Text;
  TSharedPtr<FJsonObject> Root;
  if (FFileHelper::LoadFileToString(Text, *ConfigPath) &&
      FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text), Root) && Root.IsValid())
  {
    const auto Read = [&Root](const TCHAR *Field, float &Out) {
      double Number = 0.0;
      if (Root->TryGetNumberField(Field, Number))
      {
        Out = FMath::Max(static_cast<float>(Number), 0.0f);
      }
    };
    Read(TEXT("volume"), Volume);
    Read(TEXT("engine"), Engine);
    Read(TEXT("road"), Road);
    Read(TEXT("tyres"), Tyres);
    Read(TEXT("collisions"), Collisions);
    Root->TryGetBoolField(TEXT("debug"), bNewDebug);
  }
  bDebug = bNewDebug;
  Synth->Volume.Set(Volume);
  Synth->EngineLevel.Set(Engine);
  Synth->RoadLevel.Set(Road);
  Synth->TyresLevel.Set(Tyres);
  Synth->CollisionLevel.Set(Collisions);
  UE_LOG(LogCarlaServerCarSound, Log, TEXT("Sound: volume %.2f, engine %.2f, road %.2f, tyres %.2f, collisions %.2f%s"),
      Volume, Engine, Road, Tyres, Collisions, bDebug ? TEXT(", debug log") : TEXT(""));
}
