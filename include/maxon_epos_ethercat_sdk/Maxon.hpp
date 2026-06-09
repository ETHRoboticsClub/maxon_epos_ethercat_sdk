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
#include <string>

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
  // True if the drive is persistently referenced (Absolute Home Reference State
  // 0x30B5-02 == 1). Used by bring-up to skip homing on an absolute encoder
  // whose zero was already calibrated and stored to NVM.
  bool isAbsoluteReferenced();
  bool getSoftLimitsSDO();
  bool getFollowErrorSDO();
  bool readAccelerationLimitsSDO();
  bool readPositionLimitsSDO();
  bool getTemperatureStateSDO();
  bool readVoltageDataSDO();
  bool getConfigurationSDO();

  //homing

  bool doHoming();
  bool storeParam();


 protected:
  bool stateTransitionViaSdo(const StateTransition& stateTransition);

  // PDO
 public:
  bool setDriveStateViaPdo(const DriveState& driveState,
                           const bool waitForState);
  bool lastPdoStateChangeSuccessful() const { return stateChangeSuccessful_; }

 protected:
  void engagePdoStateMachine();
  bool mapPdos(RxPdoTypeEnum rxPdoTypeEnum, TxPdoTypeEnum txPdoTypeEnum);
  bool configParam();
  Controlword getNextStateTransitionControlword(
      const DriveState& requestedDriveState,
      const DriveState& currentDriveState);
  void autoConfigurePdoSizes();

  uint16_t getTxPdoSize();
  uint16_t getRxPdoSize();

  bool isAllowedModeCombination(const std::vector<ModeOfOperationEnum> modes);
  std::pair<RxPdoTypeEnum, TxPdoTypeEnum> getMixedPdoType(
      std::vector<ModeOfOperationEnum> modes);

  // Errors
 protected:
  void addErrorToReading(const ErrorType& errorType);

 public:
  void printErrorCode();
  void printDiagnosis();
  // Consume the fault-edge flag set by updateRead() and do the SDO read of
  // 0x603F off the RT path. Idempotent: no-op when no fault edge has been
  // observed since the last call. Call from a NON-RT thread (the executor /
  // diagnostics thread) — the underlying SDO blocks the EtherCAT mailbox, and
  // at 500 Hz one inline SDO blows the 2 ms cycle budget. See updateRead().
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
  // Set by updateRead() on a non-Fault → Fault transition. Consumed by
  // processPendingFaultLog() from a non-RT thread, which does the SDO read of
  // 0x603F and populates reading_.lastFault_. Atomic because writer (RT
  // worker) and reader (executor) are different threads.
  std::atomic<bool> faultEdgePending_{false};

  // Configurable parameters
 protected:
  bool allowModeChange_{false};
  ModeOfOperationEnum modeOfOperation_{ModeOfOperationEnum::NA};

 protected:
  mutable std::recursive_mutex stagedCommandMutex_;  // TODO required?
  mutable std::recursive_mutex readingMutex_;        // TODO required?
  mutable std::recursive_mutex mutex_;               // TODO: change name!!!!
};
}  // namespace maxon
