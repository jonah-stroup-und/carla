// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "Carla/Game/ServerWheel.h"
#include "Carla.h"

#include <util/ue-header-guard-begin.h>
#include "Async/Async.h"
#include "Dom/JsonObject.h"
#include "HAL/FileManager.h"
#include "HAL/PlatformTime.h"
#include "Misc/CommandLine.h"
#include "Misc/FileHelper.h"
#include "Misc/Parse.h"
#include "Misc/Paths.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "HAL/PlatformProcess.h"
#include <util/ue-header-guard-end.h>

#include <atomic>
#include <thread>

DEFINE_LOG_CATEGORY_STATIC(LogCarlaServerWheel, Log, All);

// winmm's joystick API, loaded at run time. Including <windows.h> and
// <mmsystem.h> here would leak into the unity build: Hide-
// WindowsPlatformTypes.h undefines TRUE/FALSE, and the boost.asio files
// that follow in the same unity file need them.
namespace
{
  constexpr uint32 WinmmNoError = 0;      // JOYERR_NOERROR
  constexpr uint32 WinmmReturnAll = 0xFF; // JOY_RETURNALL

  struct FWinmmJoyCaps // JOYCAPSW
  {
    uint16 wMid, wPid;
    TCHAR szPname[32];
    uint32 wXmin, wXmax, wYmin, wYmax, wZmin, wZmax;
    uint32 wNumButtons, wPeriodMin, wPeriodMax;
    uint32 wRmin, wRmax, wUmin, wUmax, wVmin, wVmax;
    uint32 wCaps, wMaxAxes, wNumAxes, wMaxButtons;
    TCHAR szRegKey[32];
    TCHAR szOEMVxD[260];
  };

  struct FWinmmJoyInfo // JOYINFOEX
  {
    uint32 dwSize, dwFlags;
    uint32 dwXpos, dwYpos, dwZpos, dwRpos, dwUpos, dwVpos;
    uint32 dwButtons, dwButtonNumber, dwPOV, dwReserved1, dwReserved2;
  };

  static_assert(sizeof(FWinmmJoyCaps) == 728 && sizeof(FWinmmJoyInfo) == 52, "winmm struct layout");

  struct FWinmm
  {
    using FGetNumDevs = uint32 (__stdcall *)();
    using FGetDevCaps = uint32 (__stdcall *)(uintptr_t, FWinmmJoyCaps *, uint32);
    using FGetPosEx = uint32 (__stdcall *)(uint32, FWinmmJoyInfo *);
    FGetNumDevs GetNumDevs = nullptr;
    FGetDevCaps GetDevCaps = nullptr;
    FGetPosEx GetPosEx = nullptr;

    FWinmm()
    {
#if PLATFORM_WINDOWS
      if (void *Dll = FPlatformProcess::GetDllHandle(TEXT("winmm.dll")))
      {
        GetNumDevs = reinterpret_cast<FGetNumDevs>(FPlatformProcess::GetDllExport(Dll, TEXT("joyGetNumDevs")));
        GetDevCaps = reinterpret_cast<FGetDevCaps>(FPlatformProcess::GetDllExport(Dll, TEXT("joyGetDevCapsW")));
        GetPosEx = reinterpret_cast<FGetPosEx>(FPlatformProcess::GetDllExport(Dll, TEXT("joyGetPosEx")));
      }
#endif
    }

    bool IsValid() const { return GetNumDevs && GetDevCaps && GetPosEx; }

    static const FWinmm &Get()
    {
      static const FWinmm Instance;
      return Instance;
    }
  };
} // namespace

namespace
{
  constexpr double WheelReloadCheckSeconds = 0.5;
  /// How often to look for a wheel while none is connected.
  constexpr double WheelRescanSeconds = 3.0;
  /// How long the HUD shows the last control pressed.
  constexpr double WheelLastPressedSeconds = 4.0;
  /// How often the poll thread reads the buttons.
  constexpr float WheelButtonPollSeconds = 0.002f;

  constexpr int32 WheelAxisCount = 6;
  const TCHAR *const WheelAxisNames[WheelAxisCount] = {
      TEXT("x"), TEXT("y"), TEXT("z"), TEXT("r"), TEXT("u"), TEXT("v")};

  constexpr int32 WheelButtonCount = 32;
  /// Control ids: buttons are 1..32, the d-pad directions follow.
  constexpr int32 WheelPovUp = 101;
  constexpr int32 WheelPovRight = 102;
  constexpr int32 WheelPovDown = 103;
  constexpr int32 WheelPovLeft = 104;

  const TCHAR *const WheelActionNames[static_cast<int32>(EServerWheelAction::Count)] = {
      TEXT("signal_left"), TEXT("signal_right"), TEXT("gear"), TEXT("camera"), TEXT("reset"),
      TEXT("handbrake"), TEXT("blind_spot"), TEXT("pillars"), TEXT("mission"), TEXT("restart")};

  struct FServerWheelAxisMap
  {
    int32 Axis = -1; // index into WheelAxisNames; -1 for none
    bool bInvert = false;
    float DeadZone = 0.0f;
    float Gamma = 1.0f;
    float Gain = 1.0f;
  };

  struct FServerWheelMapping
  {
    uint32 Vendor = 0x0EB7;
    FServerWheelAxisMap Steer;
    FServerWheelAxisMap Throttle;
    FServerWheelAxisMap Brake;
    TArray<int32> Buttons[static_cast<int32>(EServerWheelAction::Count)];

    FServerWheelMapping()
    {
      // A Fanatec base with pedals on it (a Podium DD2, measured 2026-11-24).
      Steer.Axis = 0;
      Throttle = {2, true, 0.02f, 1.0f, 1.0f};
      Brake = {3, true, 0.02f, 1.0f, 1.0f};
      Buttons[static_cast<int32>(EServerWheelAction::SignalLeft)] = {WheelPovLeft};
      Buttons[static_cast<int32>(EServerWheelAction::SignalRight)] = {WheelPovRight};
      Buttons[static_cast<int32>(EServerWheelAction::Camera)] = {WheelPovUp};
      Buttons[static_cast<int32>(EServerWheelAction::Gear)] = {WheelPovDown};
    }
  };

  /// A joystick found by a scan.
  struct FServerWheelJoystick
  {
    uint32 Id = 0;
    uint32 Vendor = 0;
    uint32 Product = 0;
    uint32 Min[WheelAxisCount] = {};
    uint32 Max[WheelAxisCount] = {};
  };

  struct FServerWheelReading
  {
    uint32 Axes[WheelAxisCount] = {};
    uint32 Buttons = 0;
    uint32 Pov = 0xFFFF;
  };

  FString WheelControlName(int32 Control)
  {
    switch (Control)
    {
      case WheelPovUp:    return TEXT("pov_up");
      case WheelPovRight: return TEXT("pov_right");
      case WheelPovDown:  return TEXT("pov_down");
      case WheelPovLeft:  return TEXT("pov_left");
      default:            return FString::Printf(TEXT("button %d"), Control);
    }
  }

  int32 ParseWheelControl(const TSharedPtr<FJsonValue> &Value)
  {
    double Number = 0.0;
    if (Value->TryGetNumber(Number))
    {
      const int32 Button = FMath::RoundToInt32(Number);
      return Button >= 1 && Button <= WheelButtonCount ? Button : 0;
    }
    FString Text;
    if (!Value->TryGetString(Text))
    {
      return 0;
    }
    Text.TrimStartAndEndInline();
    if (Text.Equals(TEXT("pov_up"), ESearchCase::IgnoreCase))    return WheelPovUp;
    if (Text.Equals(TEXT("pov_right"), ESearchCase::IgnoreCase)) return WheelPovRight;
    if (Text.Equals(TEXT("pov_down"), ESearchCase::IgnoreCase))  return WheelPovDown;
    if (Text.Equals(TEXT("pov_left"), ESearchCase::IgnoreCase))  return WheelPovLeft;
    const int32 Button = FCString::Atoi(*Text);
    return Button >= 1 && Button <= WheelButtonCount ? Button : 0;
  }

  void ReadWheelAxis(const TSharedPtr<FJsonObject> &Root, const TCHAR *Field, FServerWheelAxisMap &Out)
  {
    const TSharedPtr<FJsonObject> *Object = nullptr;
    if (!Root->TryGetObjectField(Field, Object))
    {
      return;
    }
    FString Axis;
    if ((*Object)->TryGetStringField(TEXT("axis"), Axis))
    {
      Out.Axis = -1;
      for (int32 Index = 0; Index < WheelAxisCount; ++Index)
      {
        if (Axis.Equals(WheelAxisNames[Index], ESearchCase::IgnoreCase))
        {
          Out.Axis = Index;
        }
      }
    }
    (*Object)->TryGetBoolField(TEXT("invert"), Out.bInvert);
    double Number = 0.0;
    if ((*Object)->TryGetNumberField(TEXT("deadzone"), Number))
    {
      Out.DeadZone = FMath::Clamp(static_cast<float>(Number), 0.0f, 0.9f);
    }
    if ((*Object)->TryGetNumberField(TEXT("gamma"), Number))
    {
      Out.Gamma = FMath::Clamp(static_cast<float>(Number), 0.1f, 10.0f);
    }
    if ((*Object)->TryGetNumberField(TEXT("gain"), Number))
    {
      Out.Gain = FMath::Clamp(static_cast<float>(Number), 0.1f, 10.0f);
    }
  }

  TArray<FServerWheelJoystick> ScanWheels(uint32 Vendor)
  {
    TArray<FServerWheelJoystick> Found;
#if PLATFORM_WINDOWS
    const FWinmm &Winmm = FWinmm::Get();
    const uint32 Count = Winmm.IsValid() ? Winmm.GetNumDevs() : 0u;
    for (uint32 Id = 0; Id < Count; ++Id)
    {
      FWinmmJoyCaps Caps = {};
      if (Winmm.GetDevCaps(Id, &Caps, sizeof(Caps)) != WinmmNoError)
      {
        continue;
      }
      if (Vendor != 0 && Caps.wMid != Vendor)
      {
        continue;
      }
      FWinmmJoyInfo Info = {};
      Info.dwSize = sizeof(Info);
      Info.dwFlags = WinmmReturnAll;
      if (Winmm.GetPosEx(Id, &Info) != WinmmNoError)
      {
        continue; // a slot the device is not plugged into
      }
      FServerWheelJoystick Device;
      Device.Id = Id;
      Device.Vendor = Caps.wMid;
      Device.Product = Caps.wPid;
      const uint32 Mins[WheelAxisCount] = {Caps.wXmin, Caps.wYmin, Caps.wZmin, Caps.wRmin, Caps.wUmin, Caps.wVmin};
      const uint32 Maxs[WheelAxisCount] = {Caps.wXmax, Caps.wYmax, Caps.wZmax, Caps.wRmax, Caps.wUmax, Caps.wVmax};
      for (int32 Axis = 0; Axis < WheelAxisCount; ++Axis)
      {
        Device.Min[Axis] = Mins[Axis];
        Device.Max[Axis] = Maxs[Axis] > Mins[Axis] ? Maxs[Axis] : Mins[Axis] + 65535u;
      }
      Found.Add(Device);
    }
#endif
    return Found;
  }

  /// The d-pad as bits: up, right, down, left (diagonals set two).
  uint32 PovDirections(uint32 Pov)
  {
    if (Pov > 36000)
    {
      return 0; // centred
    }
    return ((Pov >= 31500 || Pov <= 4500) ? 1u : 0u) |
        ((Pov >= 4500 && Pov <= 13500) ? 2u : 0u) |
        ((Pov >= 13500 && Pov <= 22500) ? 4u : 0u) |
        ((Pov >= 22500 && Pov <= 31500) ? 8u : 0u);
  }

  bool ReadWheel(uint32 Id, FServerWheelReading &Out)
  {
#if PLATFORM_WINDOWS
    FWinmmJoyInfo Info = {};
    Info.dwSize = sizeof(Info);
    Info.dwFlags = WinmmReturnAll;
    const FWinmm &Winmm = FWinmm::Get();
    if (!Winmm.IsValid() || Winmm.GetPosEx(Id, &Info) != WinmmNoError)
    {
      return false;
    }
    Out.Axes[0] = Info.dwXpos;
    Out.Axes[1] = Info.dwYpos;
    Out.Axes[2] = Info.dwZpos;
    Out.Axes[3] = Info.dwRpos;
    Out.Axes[4] = Info.dwUpos;
    Out.Axes[5] = Info.dwVpos;
    Out.Buttons = Info.dwButtons;
    Out.Pov = Info.dwPOV;
    return true;
#else
    return false;
#endif
  }
} // namespace

class FServerWheel::FImpl
{
public:

  FImpl()
    : ConfigPath(GetConfigPath())
  {
    Reload();
    if (!ScanResult.IsValid())
    {
      StartScan();
    }
    PollThread = std::thread([this]() { PollButtons(); });
  }

  ~FImpl()
  {
    bStopPolling.store(true);
    if (PollThread.joinable())
    {
      PollThread.join();
    }
    if (ScanResult.IsValid())
    {
      ScanResult.Wait();
    }
  }

  /// Polls the buttons every WheelButtonPollSeconds and latches each press
  /// until Poll() takes it: a quick paddle tap can be shorter than a frame.
  void PollButtons()
  {
    int32 PreviousId = INDEX_NONE;
    uint32 PreviousButtons = 0;
    uint32 PreviousPov = 0;
    while (!bStopPolling.load())
    {
      const int32 Id = PollId.load();
      FServerWheelReading Read;
      if (Id != INDEX_NONE && ReadWheel(static_cast<uint32>(Id), Read))
      {
        const uint32 Pov = PovDirections(Read.Pov);
        if (Id == PreviousId)
        {
          PressedButtons.fetch_or(Read.Buttons & ~PreviousButtons);
          PressedPov.fetch_or(Pov & ~PreviousPov);
        }
        PreviousId = Id;
        PreviousButtons = Read.Buttons;
        PreviousPov = Pov;
      }
      else
      {
        PreviousId = INDEX_NONE;
      }
      FPlatformProcess::SleepNoStats(WheelButtonPollSeconds);
    }
  }

  void Poll()
  {
    const double Now = FPlatformTime::Seconds();
    if (Now - LastReloadCheck >= WheelReloadCheckSeconds)
    {
      LastReloadCheck = Now;
      if (IFileManager::Get().GetTimeStamp(*ConfigPath) != ConfigStamp)
      {
        Reload();
      }
    }

    if (ScanResult.IsValid() && ScanResult.IsReady())
    {
      Adopt(ScanResult.Get());
      ScanResult.Reset();
    }
    if (Devices.Num() == 0)
    {
      if (!ScanResult.IsValid() && Now - LastScan >= WheelRescanSeconds)
      {
        StartScan();
      }
      ClearInput();
      return;
    }

    // A wheel can show up as several joysticks with only one of them live;
    // use the first that has changed since it was found.
    for (int32 Index = 0; Index < Devices.Num(); ++Index)
    {
      FServerWheelReading Read;
      if (!ReadWheel(Devices[Index].Device.Id, Read))
      {
        UE_LOG(LogCarlaServerWheel, Warning, TEXT("Wheel: %s disconnected"), *Describe(Devices[Index].Device));
        Devices.Reset();
        Active = INDEX_NONE;
        PollId.store(INDEX_NONE);
        ClearInput();
        StartScan();
        return;
      }
      FServerWheelTracked &Tracked = Devices[Index];
      if (!Tracked.bChanged && FMemory::Memcmp(&Read, &Tracked.First, sizeof(Read)) != 0)
      {
        Tracked.bChanged = true;
      }
      for (int32 Axis = 0; Axis < WheelAxisCount; ++Axis)
      {
        // Until a control is first touched the base reports every axis at
        // its midpoint, which would be a half-pressed pedal.
        const uint32 Mid = (Tracked.Device.Min[Axis] + Tracked.Device.Max[Axis]) / 2;
        Tracked.bAxisLive[Axis] |= Read.Axes[Axis] != Mid;
      }
      Tracked.State = Read;
    }
    int32 Chosen = 0;
    for (int32 Index = 0; Index < Devices.Num(); ++Index)
    {
      if (Devices[Index].bChanged)
      {
        Chosen = Index;
        break;
      }
    }
    if (Chosen != Active)
    {
      Active = Chosen;
      UE_LOG(LogCarlaServerWheel, Log, TEXT("Wheel: reading %s"), *Describe(Devices[Active].Device));
    }
    const FServerWheelTracked &Wheel = Devices[Active];

    Steer = ReadAxis(Wheel, Config.Steer, true);
    Throttle = ReadAxis(Wheel, Config.Throttle, false);
    Brake = ReadAxis(Wheel, Config.Brake, false);

    PollId.store(static_cast<int32>(Wheel.Device.Id));

    // Held state from this frame's read (the handbrake); presses from the
    // poll thread, which sees taps shorter than a frame.
    const uint32 NewButtons = PressedButtons.exchange(0);
    const uint32 NewPov = PressedPov.exchange(0);
    const uint32 PovDown = PovDirections(Wheel.State.Pov);
    bool ControlDown[WheelPovLeft + 1] = {};
    bool ControlPressed[WheelPovLeft + 1] = {};
    for (int32 Button = 1; Button <= WheelButtonCount; ++Button)
    {
      ControlDown[Button] = (Wheel.State.Buttons & (1u << (Button - 1))) != 0;
      ControlPressed[Button] = (NewButtons & (1u << (Button - 1))) != 0;
    }
    for (int32 Direction = 0; Direction < 4; ++Direction)
    {
      ControlDown[WheelPovUp + Direction] = (PovDown & (1u << Direction)) != 0;
      ControlPressed[WheelPovUp + Direction] = (NewPov & (1u << Direction)) != 0;
    }
    for (int32 Control = 1; Control <= WheelPovLeft; ++Control)
    {
      if (ControlPressed[Control])
      {
        LastPressed = WheelControlName(Control);
        LastPressedTime = Now;
        UE_LOG(LogCarlaServerWheel, Log, TEXT("Wheel: pressed %s"), *LastPressed);
      }
    }
    for (int32 Action = 0; Action < static_cast<int32>(EServerWheelAction::Count); ++Action)
    {
      bool bDown = false;
      bool bPressed = false;
      for (int32 Control : Config.Buttons[Action])
      {
        if (Control > 0 && Control <= WheelPovLeft)
        {
          bDown |= ControlDown[Control];
          bPressed |= ControlPressed[Control];
        }
      }
      bActionPressed[Action] = bPressed;
      bActionDown[Action] = bDown;
    }
  }

  bool IsConnected() const { return Active != INDEX_NONE; }

  float GetSteer() const { return Steer; }
  float GetThrottle() const { return Throttle; }
  float GetBrake() const { return Brake; }

  bool IsDown(EServerWheelAction Action) const { return bActionDown[static_cast<int32>(Action)]; }
  bool WasPressed(EServerWheelAction Action) const { return bActionPressed[static_cast<int32>(Action)]; }

  FString GetLastPressed() const
  {
    return FPlatformTime::Seconds() - LastPressedTime <= WheelLastPressedSeconds ? LastPressed : FString();
  }

  FString GetDescription() const
  {
    return IsConnected() ? Describe(Devices[Active].Device) : FString();
  }

private:

  struct FServerWheelTracked
  {
    FServerWheelJoystick Device;
    FServerWheelReading First;
    FServerWheelReading State;
    bool bChanged = false;
    bool bAxisLive[WheelAxisCount] = {};
  };

  static FString Describe(const FServerWheelJoystick &Device)
  {
    return FString::Printf(TEXT("joystick %u (%04X:%04X)"), Device.Id, Device.Vendor, Device.Product);
  }

  float ReadAxis(const FServerWheelTracked &Wheel, const FServerWheelAxisMap &Axis, bool bCentred) const
  {
    if (Axis.Axis < 0 || !Wheel.bAxisLive[Axis.Axis])
    {
      return 0.0f;
    }
    const float Min = static_cast<float>(Wheel.Device.Min[Axis.Axis]);
    const float Max = static_cast<float>(Wheel.Device.Max[Axis.Axis]);
    const float Unit = FMath::Clamp((static_cast<float>(Wheel.State.Axes[Axis.Axis]) - Min) / (Max - Min), 0.0f, 1.0f);
    if (bCentred)
    {
      float Value = Unit * 2.0f - 1.0f;
      Value = Axis.bInvert ? -Value : Value;
      // Gamma > 1 softens the centre and keeps full lock at the ends.
      const float Magnitude = FMath::Pow(FMath::Max(0.0f, FMath::Abs(Value) - Axis.DeadZone) / (1.0f - Axis.DeadZone), Axis.Gamma);
      return FMath::Clamp(FMath::Sign(Value) * Magnitude * Axis.Gain, -1.0f, 1.0f);
    }
    float Value = Axis.bInvert ? 1.0f - Unit : Unit;
    Value = FMath::Max(0.0f, Value - Axis.DeadZone) / (1.0f - Axis.DeadZone);
    return FMath::Clamp(FMath::Pow(Value, Axis.Gamma) * Axis.Gain, 0.0f, 1.0f);
  }

  void ClearInput()
  {
    Steer = 0.0f;
    Throttle = 0.0f;
    Brake = 0.0f;
    for (int32 Action = 0; Action < static_cast<int32>(EServerWheelAction::Count); ++Action)
    {
      bActionDown[Action] = false;
      bActionPressed[Action] = false;
    }
  }

  void StartScan()
  {
    LastScan = FPlatformTime::Seconds();
    const uint32 Vendor = Config.Vendor;
    ScanResult = Async(EAsyncExecution::ThreadPool, [Vendor]() { return ScanWheels(Vendor); });
  }

  void Adopt(const TArray<FServerWheelJoystick> &Found)
  {
    Devices.Reset();
    Active = INDEX_NONE;
    PollId.store(INDEX_NONE);
    for (const FServerWheelJoystick &Device : Found)
    {
      FServerWheelTracked Tracked;
      Tracked.Device = Device;
      if (ReadWheel(Device.Id, Tracked.First))
      {
        Tracked.State = Tracked.First;
        Devices.Add(Tracked);
      }
    }
    if (Devices.Num() > 0)
    {
      UE_LOG(LogCarlaServerWheel, Log, TEXT("Wheel: found %d joystick(s), first %s; mapping from %s"),
          Devices.Num(), *Describe(Devices[0].Device), *ConfigPath);
    }
  }

  void Reload()
  {
    ConfigStamp = IFileManager::Get().GetTimeStamp(*ConfigPath);
    FServerWheelMapping Loaded;
    FString Text;
    if (!FFileHelper::LoadFileToString(Text, *ConfigPath))
    {
      UE_LOG(LogCarlaServerWheel, Log, TEXT("Wheel: no config at %s; using the Fanatec defaults"), *ConfigPath);
      ApplyConfig(Loaded);
      return;
    }
    TSharedPtr<FJsonObject> Root;
    if (!FJsonSerializer::Deserialize(TJsonReaderFactory<>::Create(Text), Root) || !Root.IsValid())
    {
      UE_LOG(LogCarlaServerWheel, Warning, TEXT("Wheel: %s is not valid JSON; keeping the current mapping"), *ConfigPath);
      return;
    }
    FString Vendor;
    if (Root->TryGetStringField(TEXT("vendor"), Vendor))
    {
      Loaded.Vendor = Vendor.IsEmpty() ? 0u : FParse::HexNumber(*Vendor);
    }
    ReadWheelAxis(Root, TEXT("steer"), Loaded.Steer);
    ReadWheelAxis(Root, TEXT("throttle"), Loaded.Throttle);
    ReadWheelAxis(Root, TEXT("brake"), Loaded.Brake);
    const TSharedPtr<FJsonObject> *Buttons = nullptr;
    if (Root->TryGetObjectField(TEXT("buttons"), Buttons))
    {
      for (int32 Action = 0; Action < static_cast<int32>(EServerWheelAction::Count); ++Action)
      {
        const TArray<TSharedPtr<FJsonValue>> *Values = nullptr;
        if (!(*Buttons)->TryGetArrayField(WheelActionNames[Action], Values))
        {
          continue; // keep the default
        }
        Loaded.Buttons[Action].Reset();
        for (const TSharedPtr<FJsonValue> &Value : *Values)
        {
          if (const int32 Control = ParseWheelControl(Value))
          {
            Loaded.Buttons[Action].Add(Control);
          }
          else
          {
            UE_LOG(LogCarlaServerWheel, Warning, TEXT("Wheel: %s: unknown control for %s"), *ConfigPath, WheelActionNames[Action]);
          }
        }
      }
    }
    UE_LOG(LogCarlaServerWheel, Log, TEXT("Wheel: mapping loaded from %s"), *ConfigPath);
    ApplyConfig(Loaded);
  }

  void ApplyConfig(const FServerWheelMapping &Loaded)
  {
    const bool bVendorChanged = Loaded.Vendor != Config.Vendor;
    Config = Loaded;
    if (bVendorChanged && !ScanResult.IsValid())
    {
      Devices.Reset();
      Active = INDEX_NONE;
      PollId.store(INDEX_NONE);
      StartScan();
    }
  }

  FString ConfigPath;
  FDateTime ConfigStamp;
  double LastReloadCheck = 0.0;
  FServerWheelMapping Config;

  TFuture<TArray<FServerWheelJoystick>> ScanResult;
  double LastScan = 0.0;
  TArray<FServerWheelTracked> Devices;
  int32 Active = INDEX_NONE;

  std::thread PollThread;
  std::atomic<bool> bStopPolling{false};
  /// The joystick the poll thread reads; INDEX_NONE for none.
  std::atomic<int32> PollId{INDEX_NONE};
  /// Presses since Poll() last took them: button bits, d-pad bits.
  std::atomic<uint32> PressedButtons{0};
  std::atomic<uint32> PressedPov{0};

  float Steer = 0.0f;
  float Throttle = 0.0f;
  float Brake = 0.0f;
  bool bActionDown[static_cast<int32>(EServerWheelAction::Count)] = {};
  bool bActionPressed[static_cast<int32>(EServerWheelAction::Count)] = {};
  FString LastPressed;
  double LastPressedTime = -1000.0;
};

FString FServerWheel::GetConfigPath()
{
  FString Path;
  if (FParse::Value(FCommandLine::Get(), TEXT("-server-wheel="), Path) && !Path.IsEmpty())
  {
    return FPaths::ConvertRelativePathToFull(Path);
  }
  return FPaths::ConvertRelativePathToFull(FPaths::ProjectConfigDir() / TEXT("ServerWheel.json"));
}

FServerWheel::FServerWheel() : Impl(MakeUnique<FImpl>()) {}

FServerWheel::~FServerWheel() = default;

void FServerWheel::Poll() { Impl->Poll(); }

bool FServerWheel::IsConnected() const { return Impl->IsConnected(); }

float FServerWheel::GetSteer() const { return Impl->GetSteer(); }

float FServerWheel::GetThrottle() const { return Impl->GetThrottle(); }

float FServerWheel::GetBrake() const { return Impl->GetBrake(); }

bool FServerWheel::IsDown(EServerWheelAction Action) const { return Impl->IsDown(Action); }

bool FServerWheel::WasPressed(EServerWheelAction Action) const { return Impl->WasPressed(Action); }

FString FServerWheel::GetLastPressed() const { return Impl->GetLastPressed(); }

FString FServerWheel::GetDescription() const { return Impl->GetDescription(); }
