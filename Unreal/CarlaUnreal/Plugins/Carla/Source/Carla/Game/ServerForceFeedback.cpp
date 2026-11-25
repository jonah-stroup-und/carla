// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Game/ServerForceFeedback.h"
#include "Carla.h"

#include "Carla/Game/ServerWheel.h"

#include <util/ue-header-guard-begin.h>
#include "Dom/JsonObject.h"
#include "Engine/Engine.h"
#include "Engine/GameViewportClient.h"
#include "GenericPlatform/GenericWindow.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformProcess.h"
#include "HAL/PlatformTime.h"
#include "Misc/FileHelper.h"
#include "Misc/Parse.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Widgets/SWindow.h"
#include <util/ue-header-guard-end.h>

DEFINE_LOG_CATEGORY_STATIC(LogCarlaServerForceFeedback, Log, All);

// DirectInput 8, loaded at run time and called through its COM vtables.
// As with winmm in ServerWheel.cpp, <dinput.h> would drag <windows.h> into
// the unity build and break the boost.asio files after it. The layouts and
// vtable slots below were checked against the Windows SDK's dinput.h (x64).
namespace
{
  struct FServerFfbGuid
  {
    uint32 Data1;
    uint16 Data2, Data3;
    uint8 Data4[8];
  };

  constexpr FServerFfbGuid ServerFfbIidDirectInput8W{0xBF798031, 0x483A, 0x4DA2, {0xAA, 0x99, 0x5D, 0x64, 0xED, 0x36, 0x97, 0x00}};
  constexpr FServerFfbGuid ServerFfbXAxis{0xA36D02E0, 0xC9F3, 0x11CF, {0xBF, 0xC7, 0x44, 0x45, 0x53, 0x54, 0x00, 0x00}};
  constexpr FServerFfbGuid ServerFfbConstantForce{0x13541C20, 0x8E33, 0x11D0, {0x9A, 0xD0, 0x00, 0xA0, 0xC9, 0xA0, 0x6E, 0x35}};
  constexpr FServerFfbGuid ServerFfbSine{0x13541C23, 0x8E33, 0x11D0, {0x9A, 0xD0, 0x00, 0xA0, 0xC9, 0xA0, 0x6E, 0x35}};
  constexpr FServerFfbGuid ServerFfbSpring{0x13541C27, 0x8E33, 0x11D0, {0x9A, 0xD0, 0x00, 0xA0, 0xC9, 0xA0, 0x6E, 0x35}};
  constexpr FServerFfbGuid ServerFfbDamper{0x13541C28, 0x8E33, 0x11D0, {0x9A, 0xD0, 0x00, 0xA0, 0xC9, 0xA0, 0x6E, 0x35}};
  constexpr FServerFfbGuid ServerFfbFriction{0x13541C2A, 0x8E33, 0x11D0, {0x9A, 0xD0, 0x00, 0xA0, 0xC9, 0xA0, 0x6E, 0x35}};

  struct FServerFfbDeviceInstance // DIDEVICEINSTANCEW
  {
    uint32 dwSize;
    FServerFfbGuid guidInstance, guidProduct;
    uint32 dwDevType;
    TCHAR tszInstanceName[260], tszProductName[260];
    FServerFfbGuid guidFFDriver;
    uint16 wUsagePage, wUsage;
  };

  struct FServerFfbObjectDataFormat // DIOBJECTDATAFORMAT
  {
    const FServerFfbGuid *pguid;
    uint32 dwOfs, dwType, dwFlags;
  };

  struct FServerFfbDataFormat // DIDATAFORMAT
  {
    uint32 dwSize, dwObjSize, dwFlags, dwDataSize, dwNumObjs;
    const FServerFfbObjectDataFormat *rgodf;
  };

  struct FServerFfbEffect // DIEFFECT
  {
    uint32 dwSize, dwFlags, dwDuration, dwSamplePeriod, dwGain, dwTriggerButton, dwTriggerRepeatInterval, cAxes;
    uint32 *rgdwAxes;
    int32 *rglDirection;
    void *lpEnvelope;
    uint32 cbTypeSpecificParams;
    void *lpvTypeSpecificParams;
    uint32 dwStartDelay;
  };

  struct FServerFfbCondition // DICONDITION
  {
    int32 lOffset, lPositiveCoefficient, lNegativeCoefficient;
    uint32 dwPositiveSaturation, dwNegativeSaturation;
    int32 lDeadBand;
  };

  struct FServerFfbPeriodic // DIPERIODIC
  {
    uint32 dwMagnitude;
    int32 lOffset;
    uint32 dwPhase, dwPeriod;
  };

  struct FServerFfbPropDword // DIPROPDWORD
  {
    uint32 dwSize, dwHeaderSize, dwObj, dwHow, dwData;
  };

  static_assert(sizeof(FServerFfbDeviceInstance) == 1100 && sizeof(FServerFfbObjectDataFormat) == 24 &&
      sizeof(FServerFfbDataFormat) == 32 && sizeof(FServerFfbEffect) == 80 && offsetof(FServerFfbEffect, cbTypeSpecificParams) == 56 &&
      sizeof(FServerFfbCondition) == 24 && sizeof(FServerFfbPeriodic) == 16 && sizeof(FServerFfbPropDword) == 20,
      "DirectInput struct layout");

  using FServerFfbResult = int32; // HRESULT

  constexpr uint32 ServerFfbClassGameController = 4;           // DI8DEVCLASS_GAMECTRL
  constexpr uint32 ServerFfbEnumFlags = 0x1 | 0x100;           // DIEDFL_ATTACHEDONLY | DIEDFL_FORCEFEEDBACK
  constexpr uint32 ServerFfbExclusiveBackground = 0x1 | 0x8;   // DISCL_EXCLUSIVE | DISCL_BACKGROUND
  constexpr uint32 ServerFfbAxisAnyInstance = 0x3 | 0xFFFF00;  // DIDFT_AXIS | DIDFT_ANYINSTANCE
  constexpr uint32 ServerFfbEffectFlags = 0x10 | 0x2;          // DIEFF_CARTESIAN | DIEFF_OBJECTOFFSETS
  constexpr uint32 ServerFfbTypeSpecificParams = 0x100;        // DIEP_TYPESPECIFICPARAMS
  constexpr uint32 ServerFfbInfinite = 0xFFFFFFFF;             // INFINITE, DIEB_NOTRIGGER
  constexpr uint32 ServerFfbStopAll = 0x2;                     // DISFFC_STOPALL
  constexpr uint32 ServerFfbReset = 0x1;                       // DISFFC_RESET
  constexpr uint32 ServerFfbActuatorsOn = 0x10;                // DISFFC_SETACTUATORSON
  constexpr int32 ServerFfbMax = 10000;                        // DI_FFNOMINALMAX

  /// A COM object: a pointer to its vtable.
  struct FServerFfbCom
  {
    void **Vtbl;
  };

  template <typename TFunction, typename... TArgs>
  FServerFfbResult ServerFfbCall(FServerFfbCom *Object, int32 Slot, TArgs... Args)
  {
    return reinterpret_cast<TFunction>(Object->Vtbl[Slot])(Object, Args...);
  }

  void ServerFfbRelease(FServerFfbCom *&Object)
  {
    if (Object != nullptr)
    {
      reinterpret_cast<uint32 (__stdcall *)(FServerFfbCom *)>(Object->Vtbl[2])(Object);
      Object = nullptr;
    }
  }

  using FServerFfbEnumCallback = int32 (__stdcall *)(const FServerFfbDeviceInstance *, void *);

  // IDirectInput8W
  FServerFfbResult ServerFfbCreateDevice(FServerFfbCom *Di, const FServerFfbGuid &Guid, FServerFfbCom **Out)
  {
    return ServerFfbCall<FServerFfbResult (__stdcall *)(FServerFfbCom *, const FServerFfbGuid &, FServerFfbCom **, void *)>(Di, 3, Guid, Out, nullptr);
  }
  FServerFfbResult ServerFfbEnumDevices(FServerFfbCom *Di, FServerFfbEnumCallback Callback, void *Ref)
  {
    return ServerFfbCall<FServerFfbResult (__stdcall *)(FServerFfbCom *, uint32, FServerFfbEnumCallback, void *, uint32)>(
        Di, 4, ServerFfbClassGameController, Callback, Ref, ServerFfbEnumFlags);
  }

  // IDirectInputDevice8W
  FServerFfbResult ServerFfbSetProperty(FServerFfbCom *Device, uintptr_t Property, const FServerFfbPropDword &Value)
  {
    return ServerFfbCall<FServerFfbResult (__stdcall *)(FServerFfbCom *, const void *, const FServerFfbPropDword *)>(
        Device, 6, reinterpret_cast<const void *>(Property), &Value);
  }
  FServerFfbResult ServerFfbAcquire(FServerFfbCom *Device)
  {
    return ServerFfbCall<FServerFfbResult (__stdcall *)(FServerFfbCom *)>(Device, 7);
  }
  FServerFfbResult ServerFfbUnacquire(FServerFfbCom *Device)
  {
    return ServerFfbCall<FServerFfbResult (__stdcall *)(FServerFfbCom *)>(Device, 8);
  }
  FServerFfbResult ServerFfbSetDataFormat(FServerFfbCom *Device, const FServerFfbDataFormat &Format)
  {
    return ServerFfbCall<FServerFfbResult (__stdcall *)(FServerFfbCom *, const FServerFfbDataFormat *)>(Device, 11, &Format);
  }
  FServerFfbResult ServerFfbSetCooperativeLevel(FServerFfbCom *Device, void *Window, uint32 Flags)
  {
    return ServerFfbCall<FServerFfbResult (__stdcall *)(FServerFfbCom *, void *, uint32)>(Device, 13, Window, Flags);
  }
  FServerFfbResult ServerFfbCreateEffect(FServerFfbCom *Device, const FServerFfbGuid &Guid, const FServerFfbEffect &Effect, FServerFfbCom **Out)
  {
    return ServerFfbCall<FServerFfbResult (__stdcall *)(FServerFfbCom *, const FServerFfbGuid &, const FServerFfbEffect *, FServerFfbCom **, void *)>(
        Device, 18, Guid, &Effect, Out, nullptr);
  }
  FServerFfbResult ServerFfbCommand(FServerFfbCom *Device, uint32 Command)
  {
    return ServerFfbCall<FServerFfbResult (__stdcall *)(FServerFfbCom *, uint32)>(Device, 22, Command);
  }

  // IDirectInputEffect
  FServerFfbResult ServerFfbSetParameters(FServerFfbCom *Effect, const FServerFfbEffect &Parameters, uint32 Flags)
  {
    return ServerFfbCall<FServerFfbResult (__stdcall *)(FServerFfbCom *, const FServerFfbEffect *, uint32)>(Effect, 6, &Parameters, Flags);
  }
  FServerFfbResult ServerFfbStart(FServerFfbCom *Effect)
  {
    return ServerFfbCall<FServerFfbResult (__stdcall *)(FServerFfbCom *, uint32, uint32)>(Effect, 7, 1u, 0u);
  }

  /// How often to look for the wheel while there is none (s).
  constexpr double ServerFfbRescanSeconds = 5.0;
  constexpr double ServerFfbReloadCheckSeconds = 0.5;
  /// Frames in a row an update may fail before the device is dropped.
  constexpr int32 ServerFfbMaxFailures = 30;

  /// Speed at which the centring is about two thirds of its full (m/s).
  constexpr float ServerFfbCenteringSpeed = 8.0f;
  /// Centring spring at standstill and at speed, of full.
  constexpr float ServerFfbSpringStill = 0.1f;
  constexpr float ServerFfbSpringMoving = 0.9f;
  /// Damping at standstill and at speed, of full.
  constexpr float ServerFfbDamperStill = 0.35f;
  constexpr float ServerFfbDamperMoving = 0.12f;
  /// Tyre scrub (friction) at standstill, gone by about 3 m/s.
  constexpr float ServerFfbFrictionStill = 0.3f;
  constexpr float ServerFfbFrictionSpeed = 1.5f;
  /// An impulse (N s) that gives a full jolt: about a 1500 kg car stopping
  /// from 6 km/h.
  constexpr float ServerFfbFullImpulse = 2500.0f;
  /// How fast the jolt and rumble die away (time constants, s).
  constexpr float ServerFfbJoltDecay = 0.12f;
  constexpr float ServerFfbRumbleDecay = 0.35f;
  /// The rumble's peak, of full, and period (us).
  constexpr float ServerFfbRumbleLevel = 0.6f;
  constexpr uint32 ServerFfbRumblePeriod = 35000;

  struct FServerFfbSettings
  {
    bool bEnabled = true;
    float Strength = 0.5f;
    float Centering = 1.0f;
    float Damping = 1.0f;
    float Collisions = 1.0f;
    uint32 Vendor = 0x0EB7;
  };

  int32 ServerFfbLevel(float Fraction)
  {
    return FMath::Clamp(FMath::RoundToInt(Fraction * ServerFfbMax), -ServerFfbMax, ServerFfbMax);
  }
} // namespace

class FServerForceFeedback::FImpl
{
public:

  FImpl()
    : ConfigPath(FServerWheel::GetConfigPath())
  {
#if PLATFORM_WINDOWS
    if (void *Dll = FPlatformProcess::GetDllHandle(TEXT("dinput8.dll")))
    {
      using FCreate = FServerFfbResult (__stdcall *)(void *, uint32, const FServerFfbGuid &, void **, void *);
      using FGetModuleHandle = void *(__stdcall *)(const TCHAR *);
      FCreate Create = reinterpret_cast<FCreate>(FPlatformProcess::GetDllExport(Dll, TEXT("DirectInput8Create")));
      void *Kernel = FPlatformProcess::GetDllHandle(TEXT("kernel32.dll"));
      FGetModuleHandle GetModule = Kernel ? reinterpret_cast<FGetModuleHandle>(FPlatformProcess::GetDllExport(Kernel, TEXT("GetModuleHandleW"))) : nullptr;
      void *Object = nullptr;
      if (Create && GetModule && Create(GetModule(nullptr), 0x0800, ServerFfbIidDirectInput8W, &Object, nullptr) >= 0)
      {
        DirectInput = static_cast<FServerFfbCom *>(Object);
      }
    }
#endif
    if (DirectInput == nullptr)
    {
      UE_LOG(LogCarlaServerForceFeedback, Warning, TEXT("Force feedback: DirectInput is unavailable"));
    }
    Reload();
  }

  ~FImpl()
  {
    Close();
    ServerFfbRelease(DirectInput);
  }

  void Tick(float DeltaSeconds, float Speed, bool bDriving)
  {
    const double Now = FPlatformTime::Seconds();
    if (Now - LastReloadCheck >= ServerFfbReloadCheckSeconds)
    {
      LastReloadCheck = Now;
      if (IFileManager::Get().GetTimeStamp(*ConfigPath) != ConfigStamp)
      {
        Reload();
      }
    }
    if (!Settings.bEnabled || DirectInput == nullptr)
    {
      Close();
      return;
    }
    if (Device == nullptr && (Now - LastScan < ServerFfbRescanSeconds || !Open()))
    {
      return;
    }

    const float Dt = FMath::Max(DeltaSeconds, 0.0f);
    Jolt *= FMath::Exp(-Dt / ServerFfbJoltDecay);
    Rumble *= FMath::Exp(-Dt / ServerFfbRumbleDecay);

    const float Moving = 1.0f - FMath::Exp(-FMath::Abs(Speed) / ServerFfbCenteringSpeed);
    const float Strength = FMath::Clamp(Settings.Strength, 0.0f, 1.0f);
    const float Spring = bDriving ? Strength * Settings.Centering * FMath::Lerp(ServerFfbSpringStill, ServerFfbSpringMoving, Moving) : 0.0f;
    const float Damper = Strength * Settings.Damping * FMath::Lerp(ServerFfbDamperStill, ServerFfbDamperMoving, Moving);
    const float Friction = bDriving ? Strength * Settings.Damping * ServerFfbFrictionStill * FMath::Exp(-FMath::Abs(Speed) / ServerFfbFrictionSpeed) : 0.0f;
    const float Push = bDriving ? Strength * Settings.Collisions * Jolt : 0.0f;
    const float Shake = bDriving ? Strength * Settings.Collisions * ServerFfbRumbleLevel * Rumble : 0.0f;

    bool bOk = SetCondition(Effects[SpringEffect], Sent[SpringEffect], ServerFfbLevel(Spring));
    bOk = bOk && SetCondition(Effects[DamperEffect], Sent[DamperEffect], ServerFfbLevel(Damper));
    bOk = bOk && SetCondition(Effects[FrictionEffect], Sent[FrictionEffect], ServerFfbLevel(Friction));
    bOk = bOk && SetConstant(ServerFfbLevel(Push));
    bOk = bOk && SetPeriodic(ServerFfbLevel(Shake));
    if (bOk)
    {
      Failures = 0;
    }
    else if (++Failures > ServerFfbMaxFailures || !Reacquire())
    {
      // Focus or the device was lost and could not be taken back (above, it
      // was, and the effects restarted).
      UE_LOG(LogCarlaServerForceFeedback, Log, TEXT("Force feedback: lost the wheel; looking for it again"));
      Close();
      Failures = 0;
    }
  }

  void AddImpact(float Impulse, float Side)
  {
    const float Size = FMath::Clamp(Impulse / ServerFfbFullImpulse, 0.0f, 1.0f);
    // A hit on the right drives the front wheels, and the rim, to the right.
    Jolt = FMath::Clamp(Jolt + Size * FMath::Clamp(Side, -1.0f, 1.0f), -1.0f, 1.0f);
    Rumble = FMath::Max(Rumble, Size);
  }

  bool IsActive() const { return Device != nullptr; }

private:

  enum EEffect { SpringEffect, DamperEffect, FrictionEffect, ConstantEffect, RumbleEffect, EffectCount };

  bool Open()
  {
    LastScan = FPlatformTime::Seconds();
    void *Window = nullptr;
    if (GEngine && GEngine->GameViewport)
    {
      TSharedPtr<SWindow> Slate = GEngine->GameViewport->GetWindow();
      if (Slate.IsValid() && Slate->GetNativeWindow().IsValid())
      {
        Window = Slate->GetNativeWindow()->GetOSWindowHandle();
      }
    }
    if (Window == nullptr)
    {
      LastScan = 0.0; // no window yet: try again next frame
      return false;
    }

    struct FFound
    {
      uint32 Vendor;
      TArray<FServerFfbDeviceInstance> Devices;
    } Found{Settings.Vendor, {}};
    ServerFfbEnumDevices(DirectInput, [](const FServerFfbDeviceInstance *Instance, void *Ref) -> int32 {
      FFound &Out = *static_cast<FFound *>(Ref);
      if (Out.Vendor == 0 || (Instance->guidProduct.Data1 & 0xFFFF) == Out.Vendor)
      {
        Out.Devices.Add(*Instance);
      }
      return 1; // DIENUM_CONTINUE
    }, &Found);

    // A base can show as more than one device (the wheel's HID collections):
    // take the first that plays effects.
    for (const FServerFfbDeviceInstance &Instance : Found.Devices)
    {
      if (OpenDevice(Instance, Window))
      {
        UE_LOG(LogCarlaServerForceFeedback, Log, TEXT("Force feedback: %s"), Instance.tszProductName);
        return true;
      }
      Close();
    }
    return false;
  }

  bool OpenDevice(const FServerFfbDeviceInstance &Instance, void *Window)
  {
    if (ServerFfbCreateDevice(DirectInput, Instance.guidInstance, &Device) < 0)
    {
      Device = nullptr;
      return false;
    }
    // Only the steering axis: effects need a data format and an axis to act on.
    static const FServerFfbObjectDataFormat Objects[] = {{&ServerFfbXAxis, 0, ServerFfbAxisAnyInstance, 0}};
    static const FServerFfbDataFormat Format{sizeof(FServerFfbDataFormat), sizeof(FServerFfbObjectDataFormat), 1 /* DIDF_ABSAXIS */, 4, 1, Objects};
    FServerFfbResult Result = ServerFfbSetDataFormat(Device, Format);
    if (Result >= 0)
    {
      Result = ServerFfbSetCooperativeLevel(Device, Window, ServerFfbExclusiveBackground);
    }
    if (Result < 0)
    {
      UE_LOG(LogCarlaServerForceFeedback, Warning, TEXT("Force feedback: could not set up %s (0x%08x)"), Instance.tszProductName, Result);
      return false;
    }
    // The base's own centring would fight the spring.
    const FServerFfbPropDword AutoCenterOff{sizeof(FServerFfbPropDword), 16, 0, 0 /* DIPH_DEVICE */, 0 /* DIPROPAUTOCENTER_OFF */};
    ServerFfbSetProperty(Device, 9 /* DIPROP_AUTOCENTER */, AutoCenterOff);
    if (ServerFfbAcquire(Device) < 0)
    {
      UE_LOG(LogCarlaServerForceFeedback, Log, TEXT("Force feedback: %s is in use by another program"), Instance.tszProductName);
      return false;
    }
    ServerFfbCommand(Device, ServerFfbReset);
    ServerFfbCommand(Device, ServerFfbActuatorsOn);

    FServerFfbCondition Condition{0, 0, 0, ServerFfbMax, ServerFfbMax, 0};
    FServerFfbPeriodic Periodic{0, 0, 0, ServerFfbRumblePeriod};
    int32 Constant = 0;
    const FServerFfbGuid *Guids[EffectCount] = {&ServerFfbSpring, &ServerFfbDamper, &ServerFfbFriction, &ServerFfbConstantForce, &ServerFfbSine};
    for (int32 Index = 0; Index < EffectCount; ++Index)
    {
      FServerFfbEffect Effect = MakeEffect();
      if (Index == ConstantEffect)
      {
        Effect.cbTypeSpecificParams = sizeof(Constant);
        Effect.lpvTypeSpecificParams = &Constant;
      }
      else if (Index == RumbleEffect)
      {
        Effect.cbTypeSpecificParams = sizeof(Periodic);
        Effect.lpvTypeSpecificParams = &Periodic;
      }
      else
      {
        Effect.cbTypeSpecificParams = sizeof(Condition);
        Effect.lpvTypeSpecificParams = &Condition;
      }
      Result = ServerFfbCreateEffect(Device, *Guids[Index], Effect, &Effects[Index]);
      if (Result < 0)
      {
        Effects[Index] = nullptr;
        UE_LOG(LogCarlaServerForceFeedback, Warning, TEXT("Force feedback: %s refused effect %d (0x%08x)"), Instance.tszProductName, Index, Result);
        return false;
      }
      Sent[Index] = 0;
    }
    return StartAll();
  }

  bool StartAll()
  {
    for (FServerFfbCom *Effect : Effects)
    {
      if (Effect == nullptr || ServerFfbStart(Effect) < 0)
      {
        return false;
      }
    }
    return true;
  }

  bool Reacquire()
  {
    if (Device == nullptr || ServerFfbAcquire(Device) < 0)
    {
      return false;
    }
    for (int32 &Value : Sent)
    {
      Value = TNumericLimits<int32>::Min(); // send everything again
    }
    return StartAll();
  }

  void Close()
  {
    if (Device != nullptr)
    {
      ServerFfbCommand(Device, ServerFfbStopAll);
    }
    for (FServerFfbCom *&Effect : Effects)
    {
      ServerFfbRelease(Effect);
    }
    if (Device != nullptr)
    {
      ServerFfbUnacquire(Device);
      ServerFfbRelease(Device);
    }
  }

  FServerFfbEffect MakeEffect()
  {
    FServerFfbEffect Effect{};
    Effect.dwSize = sizeof(FServerFfbEffect);
    Effect.dwFlags = ServerFfbEffectFlags;
    Effect.dwDuration = ServerFfbInfinite;
    Effect.dwGain = ServerFfbMax;
    Effect.dwTriggerButton = ServerFfbInfinite;
    Effect.cAxes = 1;
    Effect.rgdwAxes = &Axis;
    Effect.rglDirection = &Direction;
    return Effect;
  }

  bool SetCondition(FServerFfbCom *Effect, int32 &Last, int32 Level)
  {
    if (Level == Last)
    {
      return true;
    }
    FServerFfbCondition Condition{0, Level, Level, static_cast<uint32>(ServerFfbMax), static_cast<uint32>(ServerFfbMax), 0};
    FServerFfbEffect Parameters = MakeEffect();
    Parameters.cbTypeSpecificParams = sizeof(Condition);
    Parameters.lpvTypeSpecificParams = &Condition;
    if (ServerFfbSetParameters(Effect, Parameters, ServerFfbTypeSpecificParams) < 0)
    {
      return false;
    }
    Last = Level;
    return true;
  }

  bool SetConstant(int32 Level)
  {
    if (Level == Sent[ConstantEffect])
    {
      return true;
    }
    int32 Constant = Level;
    FServerFfbEffect Parameters = MakeEffect();
    Parameters.cbTypeSpecificParams = sizeof(Constant);
    Parameters.lpvTypeSpecificParams = &Constant;
    if (ServerFfbSetParameters(Effects[ConstantEffect], Parameters, ServerFfbTypeSpecificParams) < 0)
    {
      return false;
    }
    Sent[ConstantEffect] = Level;
    return true;
  }

  bool SetPeriodic(int32 Level)
  {
    if (Level == Sent[RumbleEffect])
    {
      return true;
    }
    FServerFfbPeriodic Periodic{static_cast<uint32>(FMath::Abs(Level)), 0, 0, ServerFfbRumblePeriod};
    FServerFfbEffect Parameters = MakeEffect();
    Parameters.cbTypeSpecificParams = sizeof(Periodic);
    Parameters.lpvTypeSpecificParams = &Periodic;
    if (ServerFfbSetParameters(Effects[RumbleEffect], Parameters, ServerFfbTypeSpecificParams) < 0)
    {
      return false;
    }
    Sent[RumbleEffect] = Level;
    return true;
  }

  void Reload()
  {
    ConfigStamp = IFileManager::Get().GetTimeStamp(*ConfigPath);
    FServerFfbSettings Loaded;
    FString Text;
    TSharedPtr<FJsonObject> Root;
    if (FFileHelper::LoadFileToString(Text, *ConfigPath) &&
        FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text), Root) && Root.IsValid())
    {
      FString Vendor;
      if (Root->TryGetStringField(TEXT("vendor"), Vendor))
      {
        Loaded.Vendor = Vendor.IsEmpty() ? 0u : FParse::HexNumber(*Vendor);
      }
      const TSharedPtr<FJsonObject> *Object = nullptr;
      if (Root->TryGetObjectField(TEXT("force_feedback"), Object))
      {
        double Number = 0.0;
        (*Object)->TryGetBoolField(TEXT("enabled"), Loaded.bEnabled);
        if ((*Object)->TryGetNumberField(TEXT("strength"), Number)) { Loaded.Strength = static_cast<float>(Number); }
        if ((*Object)->TryGetNumberField(TEXT("centering"), Number)) { Loaded.Centering = static_cast<float>(Number); }
        if ((*Object)->TryGetNumberField(TEXT("damping"), Number)) { Loaded.Damping = static_cast<float>(Number); }
        if ((*Object)->TryGetNumberField(TEXT("collisions"), Number)) { Loaded.Collisions = static_cast<float>(Number); }
      }
    }
    if (Loaded.Vendor != Settings.Vendor)
    {
      Close(); // a different device
    }
    Settings = Loaded;
    UE_LOG(LogCarlaServerForceFeedback, Log, TEXT("Force feedback: %s, strength %.2f, centring %.2f, damping %.2f, collisions %.2f"),
        Settings.bEnabled ? TEXT("on") : TEXT("off"), Settings.Strength, Settings.Centering, Settings.Damping, Settings.Collisions);
  }

  FString ConfigPath;
  FDateTime ConfigStamp;
  double LastReloadCheck = 0.0;
  double LastScan = -ServerFfbRescanSeconds;
  FServerFfbSettings Settings;

  FServerFfbCom *DirectInput = nullptr;
  FServerFfbCom *Device = nullptr;
  FServerFfbCom *Effects[EffectCount] = {};
  int32 Sent[EffectCount] = {};
  uint32 Axis = 0; // the X axis, at offset 0 of the data format
  int32 Direction = 1; // along +X: the sign of a force is its direction

  float Jolt = 0.0f;
  float Rumble = 0.0f;
  int32 Failures = 0;
};

FServerForceFeedback::FServerForceFeedback() : Impl(MakeUnique<FImpl>()) {}

FServerForceFeedback::~FServerForceFeedback() = default;

void FServerForceFeedback::Tick(float DeltaSeconds, float SpeedMetersPerSecond, bool bDriving)
{
  Impl->Tick(DeltaSeconds, SpeedMetersPerSecond, bDriving);
}

void FServerForceFeedback::AddImpact(float Impulse, float Side)
{
  Impl->AddImpact(Impulse, Side);
}

bool FServerForceFeedback::IsActive() const
{
  return Impl->IsActive();
}
