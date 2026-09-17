// clang-format off
/*
** Copyright 2021 Robotic Systems Lab - ETH Zurich:
** Linghao Zhang, Jonas Junger, Lennart Nachtigall
**
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
**
** 1. Redistributions of source code must retain the above copyright notice,
**    this list of conditions and the following disclaimer.
**
** 2. Redistributions in binary form must reproduce the above copyright notice,
**    this list of conditions and the following disclaimer in the documentation
**    and/or other materials provided with the distribution.
**
** 3. Neither the name of the copyright holder nor the names of its contributors
**    may be used to endorse or promote products derived from this software without
**    specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
** AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
** IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
** DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
** FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
** DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
** SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
** CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
** OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
// clang-format on

#pragma once

#include <yaml-cpp/yaml.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <ethercat_sdk_master/EthercatDevice.hpp>
#include <mutex>
#include <optional>
#include <string>
#include <functional>

#include "maxon_epos_ethercat_sdk/Command.hpp"
#include "maxon_epos_ethercat_sdk/Controlword.hpp"
#include "maxon_epos_ethercat_sdk/DriveState.hpp"
#include "maxon_epos_ethercat_sdk/Reading.hpp"

namespace maxon {
// Decodes an EPOS4 error code (object 0x603F) into a human-readable string.
// Free function so both the SDK and application code can use it.
std::string errorCodeToString(uint16_t code);

class Maxon : public ecat_master::EthercatDevice {
 public:
  typedef std::shared_ptr<Maxon> SharedPtr;

  // create Maxon Drive from setup file
  static SharedPtr deviceFromFile(const std::string& configFile,
                                  const std::string& name,
                                  const uint32_t address);
  // constructor
  Maxon() = default;
  Maxon(const std::string& name, const uint32_t address);

  // pure virtual overwrites
 public:
  bool startup() override;
  void preShutdown() override;
  void shutdown() override;
  void updateWrite() override;
  void updateRead() override;
  bool putIntoOperation() {
    bool success;
    bus_->setState(soem_interface_rsl::ETHERCAT_SM_STATE::OPERATIONAL,
                   getAddress());
    success = bus_->waitForState(
        soem_interface_rsl::ETHERCAT_SM_STATE::OPERATIONAL, getAddress());
    return success;
  }
  PdoInfo getCurrentPdoInfo() const override { return pdoInfo_; }

 public:
  void stageCommand(const Command& command);
  Reading getReading() const;
  void getReading(Reading& reading) const;

  bool loadConfigFile(const std::string& fileName);
  bool loadConfigNode(YAML::Node configNode);
  bool loadConfiguration(const Configuration& configuration);
  Configuration getConfiguration() const;

  // SDO
 public:
  bool getStatuswordViaSdo(Statusword& statusword);
  bool setControlwordViaSdo(Controlword& controlword);
  bool setDriveStateViaSdo(const DriveState& driveState);
  // De-energize over the mailbox. From Fault a fault reset (transition 15)
  // lands in SwitchOnDisabled; from every other state, and when the
  // statusword cannot be read, the CiA-402 "Disable voltage" controlword
  // (0x6040 = 0x0000) is written blind. True only when a re-read statusword
  // confirms SwitchOnDisabled. Blocking SDO: not while PDOs cycle.
  bool disableVoltageViaSdo();
  bool resetDefaultViaSdo();
  bool readSIUnitSDO();
  bool readMaxSystemSpeedSDO();
  bool readMaxProfileVelocitySDO();
  bool readVelocityControllerGainSDO();
  bool readJLVPTControllerGainSDO();
  // Read back and log, in one line per drive, the gains actually stored in the
  // drive: current (0x30A0), velocity (0x30A2) and JVPT (0x34C6). Diagnostic.
  bool logControllerGainsSDO();
  bool readMotorDataSDO();
  double readJointStateSDO();
  bool setJointPositionTargetSDO(double jointpos);
  double getHomeReferenceStateSDO();
  bool getSoftLimitsSDO();
  bool getFollowErrorSDO();
  bool readAccelerationLimitsSDO();
  bool readPositionLimitsSDO();
  bool getTemperatureStateSDO();
  bool readVoltageDataSDO();
  bool getConfigurationSDO();

  //homing

  bool storeParam();

  struct PersistentZeroResult {
    enum class Reference { Unchanged, Applied, Unknown } reference{Reference::Unknown};
    enum class Persistence { NotAttempted, Persisted, Failed, Unknown } persistence{Persistence::NotAttempted};
    uint32_t serial{0};
    // Home Reference Position (0x30B5:01) read back after an applied reference:
    // the value the drive keeps in RAM until save-all or a power cycle.
    std::optional<int32_t> homeReference;
    std::string detail;
  };

  // Deliberate maintenance operations. Method 37 performs no commanded motion:
  // it makes the current physical position the reference, verifies a fresh
  // homing completion edge and the position readback, and leaves the drive in
  // the CiA-402 state it started in. A SwitchOnDisabled drive stays disabled in
  // HomingMode; an OperationEnabled drive is returned to CyclicJVPTMode at the
  // reference it now reports (its power stage stays on throughout). The
  // reference lives in RAM until persistReferencedZero() issues CiA-301
  // save-all, which needs a SwitchOnDisabled drive and no cyclic PDO traffic.
  // One call is one attempt; callers must not retry an indeterminate result.
  PersistentZeroResult referenceCurrentPositionAsZero(
      const std::function<bool()>& cancelled = [] { return false; });
  PersistentZeroResult referenceCurrentPositionAs(
      double positionRad, const std::function<bool()>& cancelled = [] { return false; });
  PersistentZeroResult persistReferencedZero(
      const std::function<bool()>& cancelled = [] { return false; });
  bool readDeviceSerialNumber(uint32_t& serial);
  bool readHomeReference(int32_t& homeReference);

 protected:
  // Narrow hardware boundary for the persistent-zero protocol. Production uses
  // the real SDO/PDO path; tests override only these external observations while
  // executing the same transaction algorithm.
  virtual bool persistentZeroReadSerial(uint32_t& serial);
  virtual bool persistentZeroVerifyMethod(int8_t method);
  virtual bool persistentZeroVerifyHomePosition(int32_t position);
  virtual bool persistentZeroReadDisplayedMode(int8_t& mode);
  virtual bool persistentZeroReadActualPosition(int32_t& position);
  virtual bool persistentZeroReadHomeReference(int32_t& homeReference);
  virtual bool persistentZeroVerifyJvptGain(uint8_t subindex, uint32_t value);
  virtual bool persistentZeroStoreParameters();
  virtual Reading persistentZeroReading() const;
  virtual void persistentZeroStageCommand(const Command& command);
  virtual void persistentZeroSetHomingStart(bool start);
  virtual void persistentZeroSleepFor(std::chrono::milliseconds duration);

  bool stateTransitionViaSdo(const StateTransition& stateTransition);
  // Re-reads the statusword a bounded number of times until it shows
  // driveState. False on the first failed read.
  bool confirmDriveStateViaSdo(const DriveState& driveState);

  // PDO
 public:
  bool setDriveStateViaPdo(const DriveState& driveState,
                           const bool waitForState);
  bool lastPdoStateChangeSuccessful() const { return stateChangeSuccessful_; }

 protected:
  void engagePdoStateMachine();
  bool mapPdos(RxPdoTypeEnum rxPdoTypeEnum, TxPdoTypeEnum txPdoTypeEnum);
  // True when the drive already holds exactly this PDO assignment and mapping,
  // so mapPdos can skip rewriting it. Reads only.
  bool pdoMappingIsCurrent(uint16_t assignment, uint16_t mapping,
                           const uint32_t* objects, uint8_t count);
  bool configParam();
  Controlword getNextStateTransitionControlword(
      const DriveState& requestedDriveState,
      const DriveState& currentDriveState);
  void autoConfigurePdoSizes();

  // Software position limit — command-frame enforcement for JVPT mode.
  // Clamps a commanded joint position (raw encoder increments) to the configured
  // soft window [softMinPosLimitSI, softMaxPosLimitSI]. JVPT is a manufacturer-
  // specific mode (ModeOfOperationEnum = -64) whose honoring of the drive's
  // CiA-402 software position limit (0x607D) is undocumented, so we enforce here
  // in the SDK, in the same joint-radian frame the limits are authored in (which
  // also sidesteps the post-homing 0x30B0 frame shift). A degenerate window
  // (max <= min, e.g. the [0,0] default) means "disabled" and the target passes
  // through unchanged. NOTE: this gates the position setpoint only — the JVPT
  // feedforward torque is not limited here.
  int32_t clampJointPositionToSoftLimits(int32_t targetJointPositionRaw);

  uint16_t getTxPdoSize();
  uint16_t getRxPdoSize();

  bool isAllowedModeCombination(const std::vector<ModeOfOperationEnum> modes);
  std::pair<RxPdoTypeEnum, TxPdoTypeEnum> getMixedPdoType(
      std::vector<ModeOfOperationEnum> modes);

  // Errors
 protected:
  void addErrorToReading(const ErrorType& errorType);

 public:
  // SDO-read the EPOS4 error code (0x603F) into `code` and record it in the
  // reading's fault history. No logging; false when the read fails. Blocks
  // the mailbox: call from a NON-RT thread only.
  bool readErrorCode(uint16_t& code);
  void printErrorCode();
  void printDiagnosis();
  // Call repeatedly from the executor; queues/polls the fault-code SDO without
  // waiting on the network or holding the cyclic device mutex.
  void processPendingFaultLog();

 public:
  Configuration configuration_;

 protected:
  Command stagedCommand_;
  Reading reading_;
  RxPdoTypeEnum rxPdoTypeEnum_{RxPdoTypeEnum::NA};
  TxPdoTypeEnum txPdoTypeEnum_{TxPdoTypeEnum::NA};
  Controlword controlword_;
  PdoInfo pdoInfo_;
  bool hasRead_{false};
  bool conductStateChange_{false};
  DriveState targetDriveState_{DriveState::NA};
  DriveState lastLoggedFaultState_{DriveState::NA};
  std::chrono::time_point<std::chrono::steady_clock> driveStateChangeTimePoint_;
  uint16_t numberOfSuccessfulTargetStateReadings_{0};
  std::atomic<bool> stateChangeSuccessful_{false};
  // RT writer, executor consumer. A new edge queues another diagnostic read.
  std::atomic<bool> faultEdgePending_{false};
  soem_interface_rsl::MailboxRequest::Ptr faultCodeRequest_; // executor-owned

  // Rising/falling-edge latch for clampJointPositionToSoftLimits() so the [WARN]
  // fires once when the command enters the clamped region and once when it
  // returns inside the window — not every RT cycle.
  bool softLimitClampActive_{false};
  // Latch so a non-finite soft-limit config is reported once, not every cycle.
  bool softLimitConfigInvalidLogged_{false};

  // Configurable parameters
 protected:
  bool allowModeChange_{false};
  ModeOfOperationEnum modeOfOperation_{ModeOfOperationEnum::NA};

 protected:
  mutable std::recursive_mutex stagedCommandMutex_;  // TODO required?
  mutable std::recursive_mutex readingMutex_;        // TODO required?
  mutable std::recursive_mutex mutex_;               // TODO: change name!!!!
  std::mutex persistentZeroMutex_;
};
}  // namespace maxon
