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

#include "maxon_epos_ethercat_sdk/Maxon.hpp"

#include <chrono>
#include <cmath>
#include <map>
#include <thread>
#include <algorithm>

#include "maxon_epos_ethercat_sdk/ConfigurationParser.hpp"
#include "maxon_epos_ethercat_sdk/JointUnits.hpp"
#include "maxon_epos_ethercat_sdk/ObjectDictionary.hpp"
#include "maxon_epos_ethercat_sdk/RxPdo.hpp"
#include "maxon_epos_ethercat_sdk/TxPdo.hpp"

namespace maxon {
std::string binstring(uint16_t var) {
  std::string s = "0000000000000000";
  for (int i = 0; i < 16; i++) {
    if (var & (1 << (15 - i))) {
      s[i] = '1';
    }
  }
  return s;
}
std::string binstring(int8_t var) {
  std::string s = "00000000";
  for (int i = 0; i < 8; i++) {
    if (var & (1 << (7 - i))) {
      s[i] = '1';
    }
  }
  return s;
}

Maxon::SharedPtr Maxon::deviceFromFile(const std::string& configFile,
                                       const std::string& name,
                                       const uint32_t address) {
  auto maxon = std::make_shared<Maxon>(name, address);
  maxon->loadConfigFile(configFile);
  return maxon;
}

Maxon::Maxon(const std::string& name, const uint32_t address) {
  address_ = address;
  name_ = name;
}

bool Maxon::startup() {
  bool success = true;
  success &= bus_->waitForState(soem_interface_rsl::ETHERCAT_SM_STATE::PRE_OP,
                                address_);
  // bus_->syncDistributedClock0(address_, true, timeStep_, timeStep_ / 2.f); //
  // Might not need
  //
  // No settle sleep here or at the end: every SDO below is verified by a
  // read-back, and a drive not ready for mailbox traffic fails that loudly
  // ("hardware configuration ... not successful") rather than silently. The two
  // 100 ms sleeps this replaces cost 4.4 s of the 22-drive serial start.

  // PDO mapping
  success &= mapPdos(rxPdoTypeEnum_, txPdoTypeEnum_);

  // Set Interpolation Time Period (0x60C2). The drive's internal interpolator
  // bridges between cyclic setpoints over this window — it MUST equal the
  // master's actual cyclic period or inner loops over/undershoot between
  // setpoints. Previously hardcoded to 0 ms (silent bug); now derived from
  // the EthercatDevice base class's timeStep_ (set by EthercatMaster from
  // its own YAML time_step) so it always matches the master cycle and the
  // DC SYNC0 period above. Note: we use timeStep_ (base member) NOT
  // configuration_.timeStep — maxon::Configuration has no timeStep field.
  // Encoded as value (sub 0x01, uint8) × 10^exponent (sub 0x02, int8 base 10),
  // exponent fixed at -3 → milliseconds. Clamped to [1, 255] ms (uint8 range)
  // with a [WARN] per CLAUDE.md §5 if the configured timeStep falls outside.
  const double timeStepMs = timeStep_ * 1000.0;
  const long roundedMs = std::lround(timeStepMs);
  // CLAUDE.md §5: no "reasonable fallback" exists for timeStep <= 0 — refuse to
  // start. For timeStep > 255 ms (exotic bench rates) clamp + WARN is OK.
  if (roundedMs < 1) {
    MELO_ERROR_STREAM("[maxon_epos_ethercat_sdk:Maxon::startup] '" << name_
        << "': expected timeStep > 0 for 0x60C2 encoding, got " << timeStepMs
        << " ms. Refusing to start; fix ethercat_master_s.time_step in YAML.");
    addErrorToReading(ErrorType::ConfigurationError);
    return false;
  }
  if (roundedMs > 255) {
    MELO_WARN_STREAM("[maxon_epos_ethercat_sdk:Maxon::startup] '" << name_
        << "': expected timeStep <= 255 ms for 0x60C2 uint8 encoding, got "
        << timeStepMs << " ms, fallback=255 ms. Drive interpolator window will "
        "be shorter than master cycle.");
  }
  const uint8_t periodValue =
      static_cast<uint8_t>(std::clamp<long>(roundedMs, 1, 255));
  success &= sdoVerifyWrite(OD_INDEX_INTERPOLATION_TIME_PERIOD, 0x01, false,
                            periodValue,
                            configuration_.configRunSdoVerifyTimeout);
  success &= sdoVerifyWrite(OD_INDEX_INTERPOLATION_TIME_PERIOD, 0x02, false,
                            static_cast<int8_t>(-3),
                            configuration_.configRunSdoVerifyTimeout);
  MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::startup] '" << name_
                   << "' 0x60C2 interpolation period set to "
                   << static_cast<int>(periodValue) << " ms "
                   << "(timeStep_=" << timeStepMs << " ms)");


  // Set initial mode of operation
  success &=
      sdoVerifyWrite(OD_INDEX_MODES_OF_OPERATION, 0x00, false,
                     static_cast<int8_t>(configuration_.modesOfOperation[0]),
                     configuration_.configRunSdoVerifyTimeout);

  // To be on the safe side: set currect PDO sizes
  autoConfigurePdoSizes();

  // write the configuration parameters via Sdo
  success &= configParam();

  if (!success) {
    MELO_ERROR_STREAM(
        "[maxon_epos_ethercat_sdk:Maxon::preStartupOnlineConfiguration] "
        "hardware configuration of '"
        << name_ << "' not successful!");
    addErrorToReading(ErrorType::ConfigurationError);
  }
  return success;
}

void Maxon::preShutdown() {
  if (!disableVoltageViaSdo()) {
    MELO_ERROR_STREAM("[maxon_epos_ethercat_sdk:Maxon::preShutdown] '"
                      << name_ << "' NOT confirmed SwitchOnDisabled");
  }
}

void Maxon::shutdown() {
  bus_->setState(soem_interface_rsl::ETHERCAT_SM_STATE::INIT, address_);
}

void Maxon::updateWrite() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  /*
  ** Check if the Mode of Operation has been set properly
  */
  if (modeOfOperation_ == ModeOfOperationEnum::NA) {
    reading_.addError(ErrorType::ModeOfOperationError);
    MELO_ERROR_STREAM(
        "[maxon_epos_ethercat_sdk:Maxon::updateWrite]"
        " Mode of operation for '"
        << name_ << "' has not been set.");
    return;
  }

  /*!
   * engage the state machine if a state change is requested
   */
  if (conductStateChange_ && hasRead_) {
    engagePdoStateMachine();
  }

  switch (rxPdoTypeEnum_) {
    case RxPdoTypeEnum::RxPdoStandard: {
      RxPdoStandard rxPdo{};
      rxPdo.modeOfOperation_ = static_cast<int8_t>(modeOfOperation_);
      rxPdo.controlWord_ = controlword_.getRawControlword();

      // actually writing to the hardware
      bus_->writeRxPdo(address_, rxPdo);
      break;
    }

    case RxPdoTypeEnum::RxPdoCST: {
      RxPdoCST rxPdo{};
      {
        std::lock_guard<std::recursive_mutex> lock(stagedCommandMutex_);
        rxPdo.targetTorque_ = stagedCommand_.getTargetTorqueRaw();
        rxPdo.torqueOffset_ = stagedCommand_.getTorqueOffsetRaw();

        // Extra data
        rxPdo.controlWord_ = controlword_.getRawControlword();
        rxPdo.modeOfOperation_ = static_cast<int8_t>(modeOfOperation_);
      }

      // actually writing to the hardware
      bus_->writeRxPdo(address_, rxPdo);
      break;
    }

    case RxPdoTypeEnum::RxPdoPVM: {
      RxPdoPVM rxPdo{};
      {
        std::lock_guard<std::recursive_mutex> lock(stagedCommandMutex_);
        rxPdo.controlWord_ = controlword_.getRawControlword();
        rxPdo.targetVelocity_ = stagedCommand_.getTargetVelocityRaw();
        rxPdo.profileAccel_ = stagedCommand_.getProfileAccelRaw();
        rxPdo.profileDeccel_ = stagedCommand_.getProfileDeccelRaw();
        rxPdo.motionProfileType_ = stagedCommand_.getMotionProfileType();
        // MELO_WARN_STREAM("Target Velocity: " << rxPdo.targetVelocity_);
      }
      bus_->writeRxPdo(address_, rxPdo);
      break;
    }
        case RxPdoTypeEnum::RxPdoJVPT: {
      RxPdoJVPT rxPdo{};
      {
        std::lock_guard<std::recursive_mutex> lock(stagedCommandMutex_);
        rxPdo.controlWord_ = controlword_.getRawControlword();
        rxPdo.targetJointPosition_ = stagedCommand_.getTargetJointPositionRaw();
        rxPdo.targetJointVelocity_ = stagedCommand_.getTargetJointVelocityRaw();
        rxPdo.targetJointTorque_ = stagedCommand_.getTargetJointTorqueRaw();
        rxPdo.modeOfOperation_ = static_cast<int8_t>(modeOfOperation_);

      }
      // Software position limit: enforce the configured soft window on the
      // commanded joint position before it reaches the drive. No-op when the
      // window is disabled (max <= min). See clampJointPositionToSoftLimits.
      rxPdo.targetJointPosition_ =
          clampJointPositionToSoftLimits(rxPdo.targetJointPosition_);
      // actually writing to the hardware
      bus_->writeRxPdo(address_, rxPdo);
      break;
    }

      case RxPdoTypeEnum::RxPdoFreeze: {
      RxPdoFreeze rxPdo{};
      {
        std::lock_guard<std::recursive_mutex> lock(stagedCommandMutex_);
        rxPdo.controlWord_ = controlword_.getRawControlword();
        rxPdo.targetJointPosition_ = stagedCommand_.getTargetJointPositionRaw();
        rxPdo.targetJointVelocity_ = stagedCommand_.getTargetJointVelocityRaw();
        rxPdo.targetJointTorque_ = stagedCommand_.getTargetJointTorqueRaw();
        rxPdo.modeOfOperation_ = static_cast<int8_t>(modeOfOperation_);

      }
      // actually writing to the hardware
      bus_->writeRxPdo(address_, rxPdo);
      break;
    }
    default:
      MELO_ERROR_STREAM(
          "[maxon_epos_ethercat_sdk:Maxon::updateWrite] "
          " Unsupported Rx Pdo type for '"
          << name_ << "'");
      addErrorToReading(ErrorType::RxPdoTypeError);
  }
}

int32_t Maxon::clampJointPositionToSoftLimits(int32_t targetJointPositionRaw) {
  const double softMinSI = configuration_.softMinPosLimitSI;
  const double softMaxSI = configuration_.softMaxPosLimitSI;

  // Reject a non-finite config (NaN/inf from a bad YAML) BEFORE the cast below:
  // casting a non-finite double to int is undefined behaviour, and this runs on
  // the RT path. Treat as "disabled" and pass through, logged once (no silent
  // fallback — CLAUDE.md §5).
  if (!std::isfinite(softMinSI) || !std::isfinite(softMaxSI)) {
    if (!softLimitConfigInvalidLogged_) {
      MELO_WARN_STREAM(
          "[maxon_epos_ethercat_sdk:Maxon::clampJointPositionToSoftLimits] '"
          << name_ << "': non-finite soft position limit (min=" << softMinSI
          << ", max=" << softMaxSI << "); soft limit DISABLED for this drive.");
      softLimitConfigInvalidLogged_ = true;
    }
    return targetJointPositionRaw;
  }

  // Convert the configured soft window (joint radians) to raw encoder increments
  // with the SAME factor the command path uses (positionEncoderResolution / 2*pi,
  // see stageCommand()), so the limit and the commanded position share one frame.
  const double positionFactorRadToInteger =
      static_cast<double>(configuration_.positionEncoderResolution) / (2.0 * M_PI);

  // Saturate to int32 range before casting (out-of-range double -> int is also
  // UB). A window wider than int32 increments is effectively unbounded.
  constexpr double kInt32MaxD = 2147483647.0;
  constexpr double kInt32MinD = -2147483648.0;
  auto toSaturatedInc = [&](double radValue) -> int32_t {
    double inc = radValue * positionFactorRadToInteger;
    if (inc > kInt32MaxD) inc = kInt32MaxD;
    else if (inc < kInt32MinD) inc = kInt32MinD;
    return static_cast<int32_t>(inc);
  };
  const int32_t minInc = toSaturatedInc(softMinSI);
  const int32_t maxInc = toSaturatedInc(softMaxSI);

  // Degenerate / disabled window (covers the [0,0] default): pass through.
  if (maxInc <= minInc) {
    return targetJointPositionRaw;
  }

  int32_t clamped = targetJointPositionRaw;
  if (clamped > maxInc) {
    clamped = maxInc;
  } else if (clamped < minInc) {
    clamped = minInc;
  }

  const bool clamping = (clamped != targetJointPositionRaw);
  if (clamping && !softLimitClampActive_) {
    MELO_WARN_STREAM(
        "[maxon_epos_ethercat_sdk:Maxon::clampJointPositionToSoftLimits] '"
        << name_ << "': commanded joint position " << targetJointPositionRaw
        << " inc left soft window [" << minInc << ", " << maxInc
        << "] inc; clamping to " << clamped << " inc.");
  } else if (!clamping && softLimitClampActive_) {
    MELO_INFO_STREAM(
        "[maxon_epos_ethercat_sdk:Maxon::clampJointPositionToSoftLimits] '"
        << name_ << "': commanded joint position back inside soft window ["
        << minInc << ", " << maxInc << "] inc.");
  }
  softLimitClampActive_ = clamping;
  return clamped;
}

void Maxon::updateRead() {
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  // TODO(duboisf): implement some sort of time stamp
  switch (txPdoTypeEnum_) {
    case TxPdoTypeEnum::TxPdoStandard: {
      TxPdoStandard txPdo{};
      // reading from the bus
      bus_->readTxPdo(address_, txPdo);
      reading_.setStatusword(txPdo.statusword_);
      break;
    }

    case TxPdoTypeEnum::TxPdoCST: {
      TxPdoCST txPdo{};
      // reading from the bus
      bus_->readTxPdo(address_, txPdo);
      {
        std::lock_guard<std::recursive_mutex> lock(readingMutex_);
        reading_.setStatusword(txPdo.statusword_);
        reading_.setActualCurrent(txPdo.actualTorque_);
        reading_.setActualVelocity(txPdo.actualVelocity_);
        reading_.setActualPosition(txPdo.actualPosition_);
      }
      break;
    }

    case TxPdoTypeEnum::TxPdoPVM: {
      TxPdoPVM txPdo{};
      // reading from the bus
      bus_->readTxPdo(address_, txPdo);
      {
        std::lock_guard<std::recursive_mutex> lock(readingMutex_);
        reading_.setDemandVelocity(txPdo.demandVelocity_);
        reading_.setStatusword(txPdo.statusword_);
        reading_.setActualVelocity(txPdo.actualVelocity_);
        // MELO_WARN_STREAM("Demand Velocity: " << txPdo.demandVelocity_);
        // MELO_WARN_STREAM("Actual Velocity: " << txPdo.actualVelocity_);
      }
      break;
    }

    case TxPdoTypeEnum::TxPdoJVPT: {
      TxPdoJVPT txPdo{};

      //reading from the bus
      bus_->readTxPdo(address_, txPdo);
      {
      //get the bus mutex lock for reading, prevents multiple calls accesing the bus at the same time
      std::lock_guard<std::recursive_mutex>lock(readingMutex_);
      //from the TxPDOJVPT configuration read the required values from the actuators
      reading_.setStatusword(txPdo.statusword_);
      reading_.setActualJointPositionRAW(txPdo.actualJointPosition_);
      reading_.setTimePointNow();  // bus calls updateRead only after a successful WKC
      reading_.setActualJointVelocityRAW(txPdo.actualJointVelocity_);
      reading_.setActualJointCurrentRAW(txPdo.actualJointCurrent_);
      reading_.setEstJointTorqueRAW(txPdo.estJointTorque_);
      // Motor temperature is on the cyclic PDO (6 objects total). Order MUST
      // match the TxPdoJVPT struct + the mapping array in
      // ConfigureParameters.cpp. Power-stage (psu) temperature was dropped from
      // the cyclic PDO (read via SDO if needed).
      reading_.setMotorTemperatureRAW(txPdo.temeperature_motor);
      // reading_.setPsuTemperatureRAW(txPdo.temeperature_psu);  // psu temp off cyclic PDO
      // These diagnostics remain off the cyclic PDO (read via SDO if needed).
      // Their Reading getters return defaults (0) until read via SDO.
      // reading_.setDemandedJointCurrentRAW(txPdo.currentDemand);
      // reading_.setDemandedJointVelocityRAW(txPdo.velocityDemand);
      // reading_.setI2tMotorRAW(txPdo.i2tmotor);
      // reading_.setI2tPSURAW(txPdo.i2tpsu);
      // reading_.setPositionDemand(txPdo.positionDemand);

      }

      break;
    }

      case TxPdoTypeEnum::TxPdoFreeze: {
      TxPdoFreeze txPdo{};

      //reading from the bus
      bus_->readTxPdo(address_, txPdo);
      {
      //get the bus mutex lock for reading, prevents multiple calls accesing the bus at the same time
      std::lock_guard<std::recursive_mutex>lock(readingMutex_);
      //from the TxPDOJVPT configuration read the required values from the actuators
      reading_.setStatusword(txPdo.statusword_);
      reading_.setActualJointPositionRAW(txPdo.actualJointPosition_);
      reading_.setTimePointNow();
      reading_.setActualJointVelocityRAW(txPdo.actualJointVelocity_);
      reading_.setActualJointCurrentRAW(txPdo.actualJointCurrent_);
      reading_.setDemandedJointCurrentRAW(txPdo.currentDemand);
      reading_.setDemandedJointVelocityRAW(txPdo.velocityDemand);
      reading_.setMotorTemperatureRAW(txPdo.temeperature_motor);
      reading_.setI2tMotorRAW(txPdo.i2tmotor);
      reading_.setPsuTemperatureRAW(txPdo.temeperature_psu);
      reading_.setI2tPSURAW(txPdo.i2tpsu);
      reading_.setEstJointTorqueRAW(txPdo.estJointTorque_);
      reading_.setPositionDemand(txPdo.positionDemand);
      
      }

      break;
    }

    default:
      MELO_ERROR_STREAM(
          "[maxon_epos_ethercat_sdk:Maxon::updateRead] Unsupported Tx Pdo "
          "type for '"
          << name_ << "'");
      reading_.addError(ErrorType::TxPdoTypeError);
  }

  // set the hasRead_ variable to true since a nes reading was read
  if (!hasRead_) {
    hasRead_ = true;
  }

  // Print warning if drive is in FaultReactionActive state.
  const DriveState currentDriveState = reading_.getDriveState();
  if (currentDriveState == DriveState::FaultReactionActive) {
    MELO_ERROR_STREAM("[maxon_epos_ethercat_sdk:Maxon::updateRead] '"
                      << name_ << "' is in drive state 'FaultReactionActive'");
  }

  // Print warning if drive is in Fault state.
  if (currentDriveState == DriveState::Fault) {
    // Throttled: this runs every cycle while faulted; at high bus rates the
    // unthrottled version floods the log and buries the one-shot decoded
    // error-code line emitted by printErrorCode() on the fault edge below.
    MELO_ERROR_THROTTLE_STREAM(1.0, "[maxon_epos_ethercat_sdk:Maxon::updateRead] '"
                      << name_ << "' is in drive state 'Fault'");
    // Edge-triggered: FLAG the fault transition; the SDO read of 0x603F that
    // populates lastFault_ happens off the RT path in processPendingFaultLog(),
    // invoked by the executor's monitorFaultTransitions(). This moves the SDO
    // from "every fault-edge cycle on the RT worker" to "once per fault edge
    // on the 50 Hz executor". Note: on a fault edge the worker is still
    // briefly coupled to the executor's SDO via SOEM's bus contextMutex_ for
    // the mailbox round-trip (1-3 ms); this is one event per fault, not per
    // cycle, so the deadline budget at 500 Hz is no longer perpetually blown
    // when a drive is faulted. The statusword log below is local-only.
    if (lastLoggedFaultState_ != DriveState::Fault) {
      MELO_ERROR_STREAM("[maxon_epos_ethercat_sdk:Maxon::updateRead] '"
                        << name_ << "' fault statusword=0x" << std::hex
                        << reading_.getRawStatusword() << std::dec);
      faultEdgePending_.store(true, std::memory_order_release);
    }
  }
  lastLoggedFaultState_ = currentDriveState;
}
//this is a lock safe function already usibg the mutex lock
void Maxon::stageCommand(const Command& command) {
  std::lock_guard<std::recursive_mutex> deviceLock(mutex_);
  std::lock_guard<std::recursive_mutex> lock(stagedCommandMutex_);
  stagedCommand_ = command;
  // MELO_WARN_STREAM("Staged Command: " << stagedCommand_.getTargetVelocity());
  stagedCommand_.setPositionFactorRadToInteger(
      static_cast<double>(configuration_.positionEncoderResolution) /
      (2.0 * M_PI));

  double currentFactorAToInt = 1000.0 / configuration_.nominalCurrentA;
  stagedCommand_.setCurrentFactorAToInteger(currentFactorAToInt);
  stagedCommand_.setTorqueFactorNmToInteger(
      1000.0 /
      (configuration_.nominalCurrentA * configuration_.torqueConstantNmA));
  stagedCommand_.setVelocityFactorToRadPerS(configuration_.velocityFactorConfiguredUnitToRadPerSec);

  stagedCommand_.setUseRawCommands(configuration_.useRawCommands);

  stagedCommand_.doUnitConversion();
  // MELO_WARN_STREAM("Staged Command: " << stagedCommand_.getTargetVelocityRaw());

  const auto targetMode = command.getModeOfOperation();
  if (std::find(configuration_.modesOfOperation.begin(),
                configuration_.modesOfOperation.end(),
                targetMode) != configuration_.modesOfOperation.end()) {
    modeOfOperation_ = targetMode;
  } else {
    MELO_ERROR_STREAM(
        "[maxon_epos_ethercat_sdk:Maxon::stageCommand] "
        "Target mode of operation '"
        << targetMode << "' for device '" << name_ << "' not allowed");
  }
}

Reading Maxon::getReading() const {
  std::lock_guard<std::recursive_mutex> lock(readingMutex_);
  return reading_;
}

void Maxon::getReading(Reading& reading) const {
  std::lock_guard<std::recursive_mutex> lock(readingMutex_);
  reading = reading_;
}

bool Maxon::loadConfigFile(const std::string& fileName) {
  ConfigurationParser configurationParser(fileName);
  return loadConfiguration(configurationParser.getConfiguration());
}

bool Maxon::loadConfigNode(YAML::Node configNode) {
  ConfigurationParser configurationParser(configNode);
  return loadConfiguration(configurationParser.getConfiguration());
}

bool Maxon::loadConfiguration(const Configuration& configuration) {
  reading_.configureReading(configuration);
  modeOfOperation_ = configuration.modesOfOperation[0];
  const auto pdoTypeSolution = configuration.getPdoTypeSolution();
  rxPdoTypeEnum_ = pdoTypeSolution.first;
  txPdoTypeEnum_ = pdoTypeSolution.second;
  configuration_ = configuration;

  MELO_INFO_STREAM("[maxon_epos_ethercat_sdk] '" << name_
                                                  << "' gear_ratio="
                                                  << configuration_.gearRatio);

  MELO_INFO_STREAM("[maxon_epos_ethercat_sdk] Sanity check for '" << name_
                                                                  << "':");
  return configuration.sanityCheck();
}

Configuration Maxon::getConfiguration() const { return configuration_; }

bool Maxon::getStatuswordViaSdo(Statusword& statusword) {
  uint16_t statuswordValue = 0;
  bool success = sendSdoRead(OD_INDEX_STATUSWORD, 0, false, statuswordValue);
  statusword.setFromRawStatusword(statuswordValue);
  return success;
}

bool Maxon::setControlwordViaSdo(Controlword& controlword) {
  return sendSdoWrite(OD_INDEX_CONTROLWORD, 0, false,
                      controlword.getRawControlword());
}

bool Maxon::resetDefaultViaSdo() {
  bool success = true;
  success &=sendSdoWrite(OD_INDEX_RESET_DEFAULT_PARAMETERS, 0x01, true, 0x64);
  success &=sendSdoWrite(OD_INDEX_RESET_DEFAULT_PARAMETERS, 0x02, true, 0x61);
  success &=sendSdoWrite(OD_INDEX_RESET_DEFAULT_PARAMETERS, 0x03, true, 0x6F);
  success &=sendSdoWrite(OD_INDEX_RESET_DEFAULT_PARAMETERS, 0x04, true, 0x6C);

  return success;
}



bool Maxon::readMaxSystemSpeedSDO() {
  uint32_t maxSystemSpeed;
  bool success =
      sendSdoRead(OD_INDEX_MAX_SYSTEM_SPEED, 6, false, maxSystemSpeed);
  MELO_INFO_STREAM("Max System Speed: " << maxSystemSpeed);
  return success;
}

bool Maxon::readMaxProfileVelocitySDO() {
  uint32_t maxProfileVelocity;
  bool success =
      sendSdoRead(OD_INDEX_MAX_PROFILE_VELOCITY, 0, false, maxProfileVelocity);
  MELO_INFO_STREAM("Max Profile Velocity: " << maxProfileVelocity);
  return success;
}


bool Maxon::readVelocityControllerGainSDO(){
  bool success = true;
  uint32_t p_gain;
  uint32_t i_gain;

  success &= sendSdoRead(OD_INDEX_VELOCITY_CONTROL_PARAM, 0x01, false, p_gain);
  success &= sendSdoRead(OD_INDEX_VELOCITY_CONTROL_PARAM, 0x02, false, i_gain);
  MELO_INFO_STREAM("P Gain: " << p_gain);
  MELO_INFO_STREAM("I Gain: " << i_gain);

  return success;
}

bool Maxon::readJLVPTControllerGainSDO(){

  bool success = true;
  uint32_t p_gain;
  uint32_t i_gain;
  uint32_t d_gain;
  uint32_t i_max;
  success &= sendSdoRead(OD_INDEX_JVPT_PARAMETERS, 0x01, false, p_gain);
  success &= sendSdoRead(OD_INDEX_JVPT_PARAMETERS, 0x02, false, i_gain);
  success &= sendSdoRead(OD_INDEX_JVPT_PARAMETERS, 0x03, false, d_gain);
  success &= sendSdoRead(OD_INDEX_JVPT_PARAMETERS, 0x04, false, i_max);
  MELO_INFO_STREAM("P Gain: " << p_gain);
  MELO_INFO_STREAM("I Gain: " << i_gain);
  MELO_INFO_STREAM("D Gain: " << d_gain);
  MELO_INFO_STREAM("I Max: " << i_max);

  return success;
}

bool Maxon::logControllerGainsSDO() {
  // One-line, per-drive read-back of the gains actually stored in the drive,
  // so the live tuning of every joint is visible at boot. Reminder: the SDK
  // only WRITES current (0x30A0) and JVPT (0x34C6); the velocity loop (0x30A2)
  // is never written by this stack, so its values here come straight from the
  // drive's NVM. Read straight from the OD via SDO (post-configuration).
  uint32_t cur_p = 0, cur_i = 0;
  uint32_t vel_p = 0, vel_i = 0;
  uint32_t jvpt_p = 0, jvpt_i = 0, jvpt_d = 0, jvpt_imax = 0;
  bool success = true;
  success &= sendSdoRead(OD_INDEX_CURRENT_CONTROL_PARAM, 0x01, false, cur_p);
  success &= sendSdoRead(OD_INDEX_CURRENT_CONTROL_PARAM, 0x02, false, cur_i);
  success &= sendSdoRead(OD_INDEX_VELOCITY_CONTROL_PARAM, 0x01, false, vel_p);
  success &= sendSdoRead(OD_INDEX_VELOCITY_CONTROL_PARAM, 0x02, false, vel_i);
  success &= sendSdoRead(OD_INDEX_JVPT_PARAMETERS, 0x01, false, jvpt_p);
  success &= sendSdoRead(OD_INDEX_JVPT_PARAMETERS, 0x02, false, jvpt_i);
  success &= sendSdoRead(OD_INDEX_JVPT_PARAMETERS, 0x03, false, jvpt_d);
  success &= sendSdoRead(OD_INDEX_JVPT_PARAMETERS, 0x04, false, jvpt_imax);

  MELO_INFO_STREAM(
      "[GainDump] '" << name_ << "' read-back from drive:"
      << "  Current(0x30A0) P=" << cur_p << " I=" << cur_i
      << " | Velocity(0x30A2) P=" << vel_p << " I=" << vel_i
      << " | JVPT(0x34C6) P=" << jvpt_p << " I=" << jvpt_i
      << " D=" << jvpt_d << " Imax=" << jvpt_imax);

  if (!success) {
    MELO_WARN_STREAM("[GainDump] '" << name_ << "': one or more gain SDO reads "
        "failed; values above may be stale/zero. Verify the OD addresses "
        "(current 0x30A0, velocity 0x30A2, JVPT 0x34C6) against the firmware spec.");
  }
  return success;
}

bool Maxon::readMotorDataSDO() {
  uint32_t nominalCurrent;
  uint32_t outputcurrentlimit;
  uint32_t motor_torque_constant;
  uint32_t control_data;
  bool success = true;
  success &=
      sendSdoRead(OD_INDEX_MOTOR_DATA, 0x01, false, nominalCurrent);
  success &=
      sendSdoRead(OD_INDEX_MOTOR_DATA, 0x02, false, outputcurrentlimit);
  success &=
      sendSdoRead(OD_INDEX_MOTOR_DATA, 0x05, false, motor_torque_constant);
  success &=
      sendSdoRead(0x3000, 0x02, false, control_data);
    
  MELO_INFO_STREAM("Nominal Current: " << nominalCurrent);
  MELO_INFO_STREAM("Output Current Limit: " << outputcurrentlimit);
  MELO_INFO_STREAM("Motor Torque Constant: " << motor_torque_constant);
  MELO_INFO_STREAM("Control Data: " << std::hex << control_data);
  return success;
}

double Maxon::readJointStateSDO() {
  int32_t jointposraw;
  double jointpos;
  sendSdoRead(OD_INDEX_JOINT_POSITION_ACTUAL, 0x00, false, jointposraw);
  jointpos = static_cast<double>(jointposraw) / 1000.0;
  MELO_INFO_STREAM("Joint Position: " << jointpos);
  return jointpos;
}

bool Maxon::setJointPositionTargetSDO(double jointpos) {
  int32_t jointposraw = static_cast<int32_t>(jointpos * 1000.0);
  bool success = sendSdoWrite(OD_INDEX_TARGET_JOINT_POSITION, 0x00, false, jointposraw);
  stagedCommand_.setTargetJointPosition(jointpos);
  return success;
}

double Maxon::getHomeReferenceStateSDO() {
  uint8_t homeref = 0x00;
  int32_t homereference;
  int32_t home_position;
  int8_t homing_method;
  int32_t homing_offset;
  bool succes = sendSdoRead(OD_INDEX_HOME_REFERENCE_STATE, 0x02, false, homeref);
  succes &= sendSdoRead(OD_INDEX_HOME_REFERENCE_STATE, 0x01, false, homereference);
  succes &= sendSdoRead(OD_INDEX_HOME_POSITION, 0x00, false, home_position);
  succes &= sendSdoRead(OD_INDEX_HOME_METHOD, 0x00, false, homing_method);
  succes &= sendSdoRead(OD_INDEX_HOME_OFFSET, 0x00, false, homing_offset);
  MELO_INFO_STREAM("Homing Method: " << homing_method);
  MELO_INFO_STREAM("Home Reference: " << static_cast<int>(homeref));
  MELO_INFO_STREAM("Home Reference Position: " << homereference);
  MELO_INFO_STREAM("Home Offset: " << homing_offset);
  MELO_INFO_STREAM("Home Position: " << home_position);
  return homereference /
         static_cast<double>(configuration_.positionEncoderResolution);
}

bool Maxon::getSoftLimitsSDO(){
  int32_t min_limit;
  int32_t max_limit;
  bool success = true;
  success &= sendSdoRead(OD_INDEX_SOFT_LIMIT, 0x01, false, min_limit);
  success &= sendSdoRead(OD_INDEX_SOFT_LIMIT, 0x02, false, max_limit);
  MELO_INFO_STREAM("Min Limit in inc: " << min_limit);
  MELO_INFO_STREAM("Max Limit in inc: " << max_limit);
  return success;
}

bool Maxon::getFollowErrorSDO(){
  uint32_t follow_error;
  bool success = sendSdoRead(OD_INDEX_FOLLOW_ERROR_WINDOW, 0x00, false, follow_error);
  MELO_INFO_STREAM("Follow Error: " << follow_error);
  return success;

}

bool Maxon::readSIUnitSDO() {
  uint32_t siUnitPos;
  uint32_t siUnitVel;
  uint32_t siUnitAcc;
  bool success = true;
  
  success &= sendSdoRead(OD_INDEX_SI_UNIT_POSITION, 0, false, siUnitPos);
  MELO_INFO_STREAM("SI Unit Position: " << std::hex << siUnitPos);

  success &= sendSdoRead(OD_INDEX_SI_UNIT_ACCELERATION, 0, false, siUnitAcc);
  MELO_INFO_STREAM("SI Unit Acceleration: " << std::hex << siUnitAcc);

  success &= sendSdoRead(OD_INDEX_SI_UNIT_VELOCITY, 0, false, siUnitVel);
  MELO_INFO_STREAM("SI Unit Velocity: " << std::hex << siUnitVel << std::dec);
  // kJointVelocityMilliRpmToRadPerSec assumes this drive reports velocity in
  // milli-RPM. If it does not, every joint velocity it publishes is off by the
  // ratio of the two units and nothing downstream can tell -- the field is
  // display/logging only, so there is no controller to diverge and give it
  // away. Say so at bring-up instead.
  if (siUnitVel != OD_VALUE_SI_UNIT_VELOCITY_MILLI_RPM) {
    MELO_ERROR_STREAM("[maxon_epos_ethercat_sdk:Maxon::readSIUnitSDO] '" << name_
        << "': expected SI unit velocity (0x60A9) = 0x" << std::hex
        << OD_VALUE_SI_UNIT_VELOCITY_MILLI_RPM << " (milli-RPM), got 0x"
        << siUnitVel << std::dec << ", fallback=converting as milli-RPM anyway. "
        "Joint velocity from this drive is scaled wrong until the drive is "
        "reflashed or JointUnits.hpp is made unit-aware.");
  }

  return success;
}

bool Maxon::readAccelerationLimitsSDO(){
  bool success = true;
  uint32_t max_acceleration;
  uint32_t max_profile_acceleration;
  uint32_t max_profile_deceleration;
  uint32_t quick_stop_deceleration;

  success &= sendSdoRead(OD_INDEX_MAX_ACCELERATION, 0, false, max_acceleration);
  success &= sendSdoRead(OD_INDEX_PROFILE_ACCELERATION, 0, false, max_profile_acceleration);
  success &= sendSdoRead(OD_INDEX_PROFILE_DECELERATION, 0, false, max_profile_deceleration);
  success &= sendSdoRead(OD_INDEX_QUICKSTOP_DECELERATION, 0, false, quick_stop_deceleration);

  MELO_INFO_STREAM("Max Acceleration: " << max_acceleration);
  MELO_INFO_STREAM("Max Profile Acceleration: " << max_profile_acceleration);
  MELO_INFO_STREAM("Max Profile Deceleration: " << max_profile_deceleration);
  MELO_INFO_STREAM("Quick Stop Deceleration: " << quick_stop_deceleration);

  return success;
}

bool Maxon::readPositionLimitsSDO(){
  bool success = true;
  int32_t max_position_range_limit;
  int32_t min_position_range_limit;
  int32_t max_soft_pos_limit;
  int32_t min_soft_pos_limit;

  success &= sendSdoRead(OD_INDEX_POSITION_RANGE_LIMIT, 0x01, false, min_position_range_limit);
  success &= sendSdoRead(OD_INDEX_POSITION_RANGE_LIMIT, 0x02, false, max_position_range_limit);
  success &= sendSdoRead(OD_INDEX_SOFTWARE_POSITION_LIMIT, 0x01, false, min_soft_pos_limit);
  success &= sendSdoRead(OD_INDEX_SOFTWARE_POSITION_LIMIT, 0x02, false, max_soft_pos_limit);

  // Read and then discarded until now, which made this the one silent member of
  // the getConfigurationSDO() dump. The soft limits matter in particular: equal
  // values (0/0) mean the drive enforces no software position limit of its own.
  MELO_INFO_STREAM("Position Range Limit: [" << min_position_range_limit << ", "
                   << max_position_range_limit << "] (raw)");
  MELO_INFO_STREAM("Software Position Limit: [" << min_soft_pos_limit << ", "
                   << max_soft_pos_limit << "] (raw; equal values = limit disabled)");

  return success;
}

bool Maxon::getTemperatureStateSDO(){
  bool success = true;
  int16_t temperature_power_stage;
  int16_t temperature_motor;

  success &= sendSdoRead(OD_INDEX_TEMPERATURE, 0x01, false, temperature_power_stage);
  success &= sendSdoRead(OD_INDEX_TEMPERATURE, 0x02, false, temperature_motor);

  MELO_INFO_STREAM("Temperature Power Stage: " << temperature_power_stage);
  MELO_INFO_STREAM("Temperature Motor: " << temperature_motor);


  return success;
}

bool Maxon::readVoltageDataSDO() {  
  bool success = true;
  uint16_t psu_voltage;

  success &= sendSdoRead(OD_INDEX_PSU_VOLTAGE, 0x01, false, psu_voltage);

  MELO_INFO_STREAM("PSU Voltage: " << psu_voltage / 10.0);
  return success;
}


bool Maxon::getConfigurationSDO(){
  //First we get the units
  readSIUnitSDO();
  //First we read the motor Data
  readMotorDataSDO();
  //Then we read voltage data
  readVoltageDataSDO();
  //Then we read the velocity controller gain only used for the freeze controller
  readVelocityControllerGainSDO();
  //Then we read the Joint Velocity Position Torque controller gain
  readJLVPTControllerGainSDO();
  //Then we read the speed limits
  readMaxSystemSpeedSDO();
  readMaxProfileVelocitySDO();
  //Then we read the acceleration limits
  readAccelerationLimitsSDO();
  //Then we read the position limits
  readPositionLimitsSDO();
  //Then we read the homing reference and state
  getHomeReferenceStateSDO();
  //Then we read the position follow error limit
  getFollowErrorSDO();
  //Then we read the temperature
  getTemperatureStateSDO();
  //get soft position limits
  getSoftLimitsSDO();

  return true;
}


bool Maxon::storeParam() {
  // CiA301 "save all parameters": write the ASCII "save" signature (0x65766173)
  // to 0x1010:01. Use a PLAIN SDO write — NOT sdoVerifyWrite — because reading
  // 0x1010:01 back returns the save-capability bitfield (bit0=1, i.e. 0x1), never
  // the signature, so verify-by-readback always reports a false failure even when
  // the save succeeded. The drive withholds the SDO download response until the
  // NVM write completes, so a successful sendSdoWrite means the save is done.
  const uint32_t signature = static_cast<uint32_t>(0x65766173);
  return sendSdoWrite(OD_STORE_PARAM, 0x01, false, signature);
}

bool Maxon::readDeviceSerialNumber(uint32_t& serial) {
  return sendSdoRead(OD_IDENTITY_OBJECT, 0x04, false, serial);
}

bool Maxon::persistentZeroSdo(uint16_t index, uint8_t subindex, uint8_t size, bool write, uint32_t& value) {
  using soem_interface_rsl::MailboxStatus;
  const auto request = requestSdo(index, subindex, size, write, value);
  // The mailbox owner completes or times out every accepted request itself;
  // the margin only covers the tick that publishes the verdict.
  const auto deadline = std::chrono::steady_clock::now() +
      soem_interface_rsl::AsyncMailbox::kTimeout + std::chrono::milliseconds(300);
  auto status = request->status.load(std::memory_order_acquire);
  while (status == MailboxStatus::Pending && std::chrono::steady_clock::now() < deadline) {
    persistentZeroSleepFor(std::chrono::milliseconds(1));
    status = request->status.load(std::memory_order_acquire);
  }
  if (status == MailboxStatus::Success) {
    if (!write) value = request->value;
    return true;
  }
  if (status != MailboxStatus::Unavailable) {
    MELO_ERROR_STREAM("[maxon_epos_ethercat_sdk:Maxon::persistentZeroSdo] '" << name_ << "' 0x" << std::hex
                      << index << ":" << static_cast<int>(subindex) << std::dec << (write ? " write " : " read ")
                      << soem_interface_rsl::mailboxStatusName(status));
    return false;
  }
  // Unavailable: the bus is not in OP, so the mailbox is idle and a synchronous
  // transfer is the one that works.
  switch (size) {
    case 1: { uint8_t v = static_cast<uint8_t>(value);
      const bool ok = write ? sendSdoWrite(index, subindex, false, v) : sendSdoRead(index, subindex, false, v);
      if (ok && !write) value = v; return ok; }
    case 2: { uint16_t v = static_cast<uint16_t>(value);
      const bool ok = write ? sendSdoWrite(index, subindex, false, v) : sendSdoRead(index, subindex, false, v);
      if (ok && !write) value = v; return ok; }
    default: {
      const bool ok = write ? sendSdoWrite(index, subindex, false, value) : sendSdoRead(index, subindex, false, value);
      return ok; }
  }
}

bool Maxon::persistentZeroReadSerial(uint32_t& serial) {
  return persistentZeroSdoRead(OD_IDENTITY_OBJECT, 0x04, serial);
}

bool Maxon::persistentZeroVerifyMethod(int8_t method) {
  return persistentZeroSdoVerifyWrite(OD_INDEX_HOME_METHOD, 0x00, method);
}

bool Maxon::persistentZeroVerifyHomePosition(int32_t position) {
  return persistentZeroSdoVerifyWrite(OD_INDEX_HOME_POSITION, 0x00, position);
}

bool Maxon::persistentZeroReadHomePosition(int32_t& position) {
  return persistentZeroSdoRead(OD_INDEX_HOME_POSITION, 0x00, position);
}

bool Maxon::persistentZeroReadDisplayedMode(int8_t& mode) {
  return persistentZeroSdoRead(OD_INDEX_MODES_OF_OPERATION_DISPLAY, 0x00, mode);
}

bool Maxon::persistentZeroReadActualPosition(int32_t& position) {
  return persistentZeroSdoRead(OD_INDEX_JOINT_POSITION_ACTUAL, 0x00, position);
}

bool Maxon::persistentZeroReadHomeReference(int32_t& homeReference) {
  return persistentZeroSdoRead(OD_INDEX_HOME_REFERENCE_STATE, 0x01, homeReference);
}

bool Maxon::readHomeReference(int32_t& homeReference) {
  // Read in either bus state: a caller settling a transaction reads it in OP.
  return persistentZeroSdoRead(OD_INDEX_HOME_REFERENCE_STATE, 0x01, homeReference);
}

bool Maxon::persistentZeroVerifyJvptGain(uint8_t subindex, uint32_t value) {
  return persistentZeroSdoVerifyWrite(OD_INDEX_JVPT_PARAMETERS, subindex, value);
}

bool Maxon::persistentZeroStoreParameters() { return storeParam(); }

Reading Maxon::persistentZeroReading() const { return getReading(); }

void Maxon::persistentZeroStageCommand(const Command& command) {
  stageCommand(command);
}

void Maxon::persistentZeroSetHomingStart(bool start) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  controlword_.homingOperationStart_ = start;
}

void Maxon::persistentZeroSleepFor(std::chrono::milliseconds duration) {
  std::this_thread::sleep_for(duration);
}

Maxon::PersistentZeroResult Maxon::referenceCurrentPositionAsZero(
    const std::function<bool()>& cancelled, double readbackToleranceRad) {
  return referenceCurrentPositionAs(0.0, cancelled, readbackToleranceRad);
}

Maxon::PersistentZeroResult Maxon::referenceCurrentPositionAs(
    double positionRad, const std::function<bool()>& cancelled, double readbackToleranceRad) {
  std::lock_guard<std::mutex> operationLock(persistentZeroMutex_);
  PersistentZeroResult out;
  const double increments = positionRad *
      static_cast<double>(configuration_.positionEncoderResolution) / (2.0 * M_PI);
  if (!std::isfinite(increments) || configuration_.positionEncoderResolution <= 0 ||
      increments < -2147483648.0 || increments > 2147483647.0) {
    out.reference = PersistentZeroResult::Reference::Unchanged;
    out.detail = "reference position cannot be represented in native encoder increments";
    return out;
  }
  const auto homePosition = static_cast<int32_t>(std::llround(increments));
  const auto stageJvptAt = [this](double target) {
    Command safe;
    safe.setModeOfOperation(ModeOfOperationEnum::CyclicJVPTMode);
    safe.setTargetJointPosition(target);
    safe.setTargetJointVelocity(0.0);
    safe.setTargetJointTorque(0.0);
    persistentZeroStageCommand(safe);
  };
  // Leaves HomingMode without moving: the target is what the drive reports now.
  const auto restoreSafeJvpt = [this, &stageJvptAt]() {
    stageJvptAt(persistentZeroReading().getActualJointPosition());
  };
  if (cancelled()) {
    out.reference = PersistentZeroResult::Reference::Unchanged;
    out.detail = "cancelled before mutation";
    return out;
  }
  if (!persistentZeroReadSerial(out.serial) || out.serial == 0) {
    out.reference = PersistentZeroResult::Reference::Unchanged;
    out.detail = "drive serial identity (0x1018:04) is unavailable";
    return out;
  }

  // Method 37 needs no power stage, but it runs on an enabled drive too: the
  // homing-mode position loop keeps the joint where it is and the frame shift
  // is applied without motion. Any other CiA-402 state is refused.
  const DriveState entryState = persistentZeroReading().getDriveState();
  const bool enabled = entryState == DriveState::OperationEnabled;
  if (!enabled && entryState != DriveState::SwitchOnDisabled) {
    out.reference = PersistentZeroResult::Reference::Unchanged;
    out.detail = "drive was neither SwitchOnDisabled nor OperationEnabled before Method-37 preparation";
    return out;
  }
  const std::string entryStateName = enabled ? "OperationEnabled" : "SwitchOnDisabled";

  const int8_t method = 37;
  if (cancelled()) {
    out.reference = PersistentZeroResult::Reference::Unchanged;
    out.detail = "cancelled before Method-37 configuration";
    return out;
  }
  bool configured = persistentZeroVerifyMethod(method);
  if (cancelled()) {
    out.reference = PersistentZeroResult::Reference::Unchanged;
    out.detail = "cancelled after method selection but before Home Position mutation";
    return out;
  }
  configured &= persistentZeroVerifyHomePosition(homePosition);
  if (!configured) {
    out.detail = "failed to configure Method 37 with the requested Home Position";
    return out;
  }

  if (cancelled()) {
    out.reference = PersistentZeroResult::Reference::Unchanged;
    out.detail = "cancelled before Method-37 trigger";
    return out;
  }

  Command command;
  command.setModeOfOperation(ModeOfOperationEnum::HomingMode);
  if (cancelled()) {
    out.reference = PersistentZeroResult::Reference::Unchanged;
    out.detail = "cancelled before HomingMode mutation";
    return out;
  }
  persistentZeroStageCommand(command);
  const auto displayedModeIs = [this](ModeOfOperationEnum expected) {
    for (unsigned i = 0; i < 20; ++i) {
      int8_t displayedMode = 0;
      if (persistentZeroReadDisplayedMode(displayedMode) &&
          displayedMode == static_cast<int8_t>(expected)) {
        return true;
      }
      persistentZeroSleepFor(std::chrono::milliseconds(10));
    }
    return false;
  };
  if (!displayedModeIs(ModeOfOperationEnum::HomingMode)) {
    restoreSafeJvpt();
    out.reference = PersistentZeroResult::Reference::Unchanged;
    out.detail = "drive did not confirm HomingMode on 0x6061";
    return out;
  }

  // Require a new completion edge. A pre-existing attained bit cannot prove
  // this operation ran, and must never be accepted as success.
  persistentZeroSetHomingStart(false);
  bool sawClear = false;
  for (unsigned i = 0; i < 20; ++i) {
    if (!persistentZeroReading().getStatusword().homingFinished()) {
      sawClear = true;
      break;
    }
    persistentZeroSleepFor(std::chrono::milliseconds(10));
  }
  if (!sawClear) {
    restoreSafeJvpt();
    out.reference = PersistentZeroResult::Reference::Unchanged;
    out.detail = "homing-attained did not clear; no fresh Method-37 edge";
    return out;
  }

  if (cancelled()) {
    restoreSafeJvpt();
    out.reference = PersistentZeroResult::Reference::Unchanged;
    out.detail = "cancelled before Method-37 start edge";
    return out;
  }

  const auto sampleBeforeTrigger = persistentZeroReading().getLastReadingTimePoint();
  persistentZeroSetHomingStart(true);
  bool finished = false;
  for (unsigned i = 0; i < 100; ++i) {
    if (cancelled()) {
      persistentZeroSetHomingStart(false);
      if (enabled) restoreSafeJvpt();
      out.reference = PersistentZeroResult::Reference::Unknown;
      out.detail = "cancelled after Method-37 trigger; reference outcome is unknown";
      return out;
    }
    const auto reading = persistentZeroReading();
    if (reading.getDriveState() != entryState) {
      persistentZeroSetHomingStart(false);
      out.reference = PersistentZeroResult::Reference::Unknown;
      out.detail = "drive left " + entryStateName + " during Method 37";
      return out;
    }
    if ((reading.getRawStatusword() & (1u << 13)) != 0) {
      persistentZeroSetHomingStart(false);
      if (enabled) restoreSafeJvpt();
      out.reference = PersistentZeroResult::Reference::Unknown;
      out.detail = "drive reported a Method-37 homing error";
      return out;
    }
    const bool newerThanTrigger =
        reading.getLastReadingTimePoint() > sampleBeforeTrigger;
    const bool referenced = (reading.getRawStatusword() & (1u << 15)) != 0;
    if (newerThanTrigger && reading.getStatusword().homingFinished() && referenced) {
      finished = true;
      break;
    }
    persistentZeroSleepFor(std::chrono::milliseconds(20));
  }
  persistentZeroSetHomingStart(false);
  if (!finished) {
    if (enabled) restoreSafeJvpt();
    out.reference = PersistentZeroResult::Reference::Unknown;
    out.persistence = PersistentZeroResult::Persistence::NotAttempted;
    out.detail = "Method-37 completion was not observed before the deadline";
    return out;
  }

  const auto feedback = persistentZeroReading();
  const double ageUs = feedback.getAgeOfLastReadingInMicroseconds();
  int32_t reportedPosition = 1;
  if (!std::isfinite(ageUs) || ageUs < 0.0 || ageUs > 100000.0 ||
      !persistentZeroReadActualPosition(reportedPosition)) {
    if (enabled) restoreSafeJvpt();
    out.reference = PersistentZeroResult::Reference::Unknown;
    out.persistence = PersistentZeroResult::Persistence::NotAttempted;
    out.detail = "fresh resulting position feedback could not be verified";
    return out;
  }
  // The drive set 0x6064 := Home Position at the homing instant. The frame is
  // proven by the PDO sample of the completion cycle, the closest observation
  // to that instant; the later SDO readback of 0x6064 confirms the channel and
  // shows how far a torque-free joint has moved since. Within the caller's
  // stationarity budget that is motion, not a wrong frame.
  // The register Method 37 consumed: Home Position must still be the value
  // the drive was told to take at the homing instant.
  int32_t homePositionNow = 0;
  if (!persistentZeroReadHomePosition(homePositionNow) || homePositionNow != homePosition) {
    if (enabled) restoreSafeJvpt();
    out.reference = PersistentZeroResult::Reference::Unknown;
    out.persistence = PersistentZeroResult::Persistence::NotAttempted;
    out.detail = "Method 37 completed but Home Position (0x30B0) reads " + std::to_string(homePositionNow) +
        " against requested " + std::to_string(homePosition);
    MELO_WARN_STREAM("[maxon_epos_ethercat_sdk:Maxon::referenceCurrentPositionAs] '" << name_ << "' " << out.detail);
    return out;
  }
  out.frameSet = true;
  const double toleranceRad = std::isfinite(readbackToleranceRad) ? std::max(0.0, readbackToleranceRad) : 0.0;
  const int64_t toleranceCounts = std::max<int64_t>(kMinReadbackToleranceCounts, static_cast<int64_t>(
      toleranceRad * static_cast<double>(configuration_.positionEncoderResolution) / (2.0 * M_PI)));
  const int32_t pdoPosition = feedback.getActualJointPositionRAW();
  const int64_t delta = static_cast<int64_t>(pdoPosition) - static_cast<int64_t>(homePosition);
  const int64_t sdoDelta = static_cast<int64_t>(reportedPosition) - static_cast<int64_t>(homePosition);
  if (std::llabs(delta) > toleranceCounts) {
    if (enabled) restoreSafeJvpt();
    out.reference = PersistentZeroResult::Reference::Unknown;
    out.persistence = PersistentZeroResult::Persistence::NotAttempted;
    out.detail = "Method 37 completed but 0x6064 at completion was " + std::to_string(pdoPosition) +
        " against requested " + std::to_string(homePosition) + " (delta " + std::to_string(delta) +
        " counts, tolerance " + std::to_string(toleranceCounts) + "; SDO readback " +
        std::to_string(reportedPosition) + ", delta " + std::to_string(sdoDelta) + ")";
    MELO_WARN_STREAM("[maxon_epos_ethercat_sdk:Maxon::referenceCurrentPositionAs] '" << name_ << "' " << out.detail);
    return out;
  }
  if (delta != 0 || sdoDelta != 0) {
    MELO_WARN_STREAM("[maxon_epos_ethercat_sdk:Maxon::referenceCurrentPositionAs] '" << name_
                     << "' reference accepted: 0x6064 was " << delta << " count(s) off at completion (tolerance "
                     << toleranceCounts << ") and " << sdoDelta << " at the SDO readback; the joint moved, the frame did not");
  }
  int32_t homeReference = 0;
  if (persistentZeroReadHomeReference(homeReference)) out.homeReference = homeReference;
  out.reference = PersistentZeroResult::Reference::Applied;
  if (enabled) {
    // The drive now reports the requested reference at this physical spot, so
    // holding that value in the position loop is a no-motion handover.
    stageJvptAt(positionRad);
    if (!displayedModeIs(ModeOfOperationEnum::CyclicJVPTMode)) {
      out.reference = PersistentZeroResult::Reference::Unknown;
      out.detail = "reference read back but the drive did not confirm CyclicJVPTMode on 0x6061";
      return out;
    }
  }
  if (persistentZeroReading().getDriveState() != entryState) {
    out.reference = PersistentZeroResult::Reference::Unknown;
    out.detail = "reference read back but the drive did not stay " + entryStateName;
    return out;
  }
  out.detail = enabled
      ? "requested position reference verified in RAM; drive restored to CyclicJVPTMode at the reference"
      : "requested position reference verified in RAM; drive returned to SwitchOnDisabled";
  return out;
}

Maxon::PersistentZeroResult Maxon::persistReferencedZero(
    const std::function<bool()>& cancelled) {
  std::lock_guard<std::mutex> operationLock(persistentZeroMutex_);
  PersistentZeroResult out;
  if (!persistentZeroReadSerial(out.serial) || out.serial == 0) {
    out.persistence = PersistentZeroResult::Persistence::NotAttempted;
    out.detail = "drive serial identity (0x1018:04) is unavailable";
    return out;
  }
  if (cancelled() || persistentZeroReading().getDriveState() != DriveState::SwitchOnDisabled) {
    out.persistence = PersistentZeroResult::Persistence::NotAttempted;
    out.detail = "save-all requires an uncancelled SwitchOnDisabled drive";
    return out;
  }

  // Save-all includes controller parameters. Reassert and verify the configured
  // baseline immediately before 0x1010 so a temporary damping/policy gain can
  // never become the next boot's baseline.
  const uint32_t baselineP = static_cast<uint32_t>(configuration_.jvptPGain);
  const uint32_t baselineD = static_cast<uint32_t>(configuration_.jvptDGain);
  if (cancelled()) {
    out.persistence = PersistentZeroResult::Persistence::NotAttempted;
    out.detail = "cancelled before configured gain baseline restoration";
    return out;
  }
  bool baseline = persistentZeroVerifyJvptGain(0x01, baselineP);
  if (cancelled()) {
    out.persistence = PersistentZeroResult::Persistence::NotAttempted;
    out.detail = "cancelled after P baseline restoration; save-all not attempted";
    return out;
  }
  baseline &= persistentZeroVerifyJvptGain(0x03, baselineD);
  if (!baseline) {
    out.persistence = PersistentZeroResult::Persistence::NotAttempted;
    out.detail = "configured JVPT baseline could not be restored; save-all not attempted";
    return out;
  }
  if (cancelled()) {
    out.persistence = PersistentZeroResult::Persistence::NotAttempted;
    out.detail = "cancelled after baseline restoration; save-all not attempted";
    return out;
  }
  if (!persistentZeroStoreParameters()) {
    out.persistence = PersistentZeroResult::Persistence::Unknown;
    out.detail = "reference applied in RAM; save-all acknowledgement was not received";
    return out;
  }
  out.persistence = PersistentZeroResult::Persistence::Persisted;
  out.detail = "referenced zero persisted with configured gains";
  return out;
}


// A mailbox controlword takes effect within the drive's next state-machine
// cycle; a few re-reads cover it without an open-ended wait. A read that fails
// outright ends the confirmation: a dead mailbox does not recover in 10 ms and
// every failed read costs the SOEM mailbox timeout.
static constexpr unsigned kSdoStateConfirmReads = 5;
static constexpr std::chrono::milliseconds kSdoStateConfirmInterval{10};

bool Maxon::disableVoltageViaSdo() {
  Statusword statusword;
  if (getStatuswordViaSdo(statusword)) {
    const DriveState current = statusword.getDriveState();
    if (current == DriveState::SwitchOnDisabled) return true;
    if (current == DriveState::Fault) {
      if (!stateTransitionViaSdo(StateTransition::_15)) return false;
      return confirmDriveStateViaSdo(DriveState::SwitchOnDisabled);
    }
  } else {
    MELO_WARN_STREAM("[maxon_epos_ethercat_sdk:Maxon::disableVoltageViaSdo] '"
                     << name_ << "': statusword unreadable; writing Disable voltage blind");
  }
  // Transitions 7, 9, 10 and 12 all encode "Disable voltage" (bits 1 and 7
  // clear); 0x0000 carries none of the profile-mode flags Controlword adds.
  const uint16_t disableVoltage = 0x0000;
  if (!sendSdoWrite(OD_INDEX_CONTROLWORD, 0x00, false, disableVoltage)) {
    MELO_ERROR_STREAM("[maxon_epos_ethercat_sdk:Maxon::disableVoltageViaSdo] '"
                      << name_ << "': controlword write failed");
    addErrorToReading(ErrorType::SdoStateTransitionError);
    return false;
  }
  return confirmDriveStateViaSdo(DriveState::SwitchOnDisabled);
}

bool Maxon::confirmDriveStateViaSdo(const DriveState& driveState) {
  for (unsigned attempt = 0; attempt < kSdoStateConfirmReads; ++attempt) {
    if (attempt > 0) std::this_thread::sleep_for(kSdoStateConfirmInterval);
    Statusword statusword;
    if (!getStatuswordViaSdo(statusword)) {
      MELO_ERROR_STREAM("[maxon_epos_ethercat_sdk:Maxon::confirmDriveStateViaSdo] '"
                        << name_ << "': statusword unreadable after the transition; "
                        << driveState << " NOT confirmed");
      addErrorToReading(ErrorType::SdoStateTransitionError);
      return false;
    }
    if (statusword.getDriveState() == driveState) return true;
  }
  MELO_ERROR_STREAM("[maxon_epos_ethercat_sdk:Maxon::confirmDriveStateViaSdo] '"
                    << name_ << "': " << driveState << " NOT reached after the transition");
  addErrorToReading(ErrorType::SdoStateTransitionError);
  return false;
}

bool Maxon::setDriveStateViaSdo(const DriveState& driveState) {
  // The lowest state reachable over EtherCAT and the de-energize target: it
  // must not depend on a readable statusword.
  if (driveState == DriveState::SwitchOnDisabled) return disableVoltageViaSdo();

  bool success = true;
  Statusword currentStatusword;
  if (!getStatuswordViaSdo(currentStatusword)) {
    MELO_ERROR_STREAM("[maxon_epos_ethercat_sdk:Maxon::setDriveStateViaSdo] '"
                      << name_ << "': statusword unreadable; no transition sequenced");
    addErrorToReading(ErrorType::SdoStateTransitionError);
    return false;
  }
  DriveState currentDriveState = currentStatusword.getDriveState();

  // do the adequate state changes (via sdo) depending on the requested and
  // current drive states
  switch (driveState) {
    case DriveState::ReadyToSwitchOn:
      switch (currentDriveState) {
        case DriveState::SwitchOnDisabled:
          success &= stateTransitionViaSdo(StateTransition::_2);
          break;
        case DriveState::ReadyToSwitchOn:
          success &= true;
          break;
        case DriveState::SwitchedOn:
          success &= stateTransitionViaSdo(StateTransition::_6);
          break;
        case DriveState::OperationEnabled:
          success &= stateTransitionViaSdo(StateTransition::_8);
          break;
        case DriveState::QuickStopActive:
          success &= stateTransitionViaSdo(StateTransition::_12);
          success &= stateTransitionViaSdo(StateTransition::_2);
          break;
        case DriveState::Fault:
          success &= stateTransitionViaSdo(StateTransition::_15);
          success &= stateTransitionViaSdo(StateTransition::_2);
          break;
        default:
          MELO_ERROR_STREAM(
              "[maxon_epos_ethercat_sdk:Maxon::setDriveStateViaSdo] State "
              "Transition not implemented");
          addErrorToReading(ErrorType::SdoStateTransitionError);
          success = false;
      }
      break;

    case DriveState::SwitchedOn:
      switch (currentDriveState) {
        case DriveState::SwitchOnDisabled:
          success &= stateTransitionViaSdo(StateTransition::_2);
          success &= stateTransitionViaSdo(StateTransition::_3);
          break;
        case DriveState::ReadyToSwitchOn:
          success &= stateTransitionViaSdo(StateTransition::_3);
          break;
        case DriveState::SwitchedOn:
          success &= true;
          break;
        case DriveState::OperationEnabled:
          success &= stateTransitionViaSdo(StateTransition::_5);
          break;
        case DriveState::QuickStopActive:
          success &= stateTransitionViaSdo(StateTransition::_12);
          success &= stateTransitionViaSdo(StateTransition::_2);
          success &= stateTransitionViaSdo(StateTransition::_3);
          break;
        case DriveState::Fault:
          success &= stateTransitionViaSdo(StateTransition::_15);
          success &= stateTransitionViaSdo(StateTransition::_2);
          success &= stateTransitionViaSdo(StateTransition::_3);
          break;
        default:
          MELO_ERROR_STREAM(
              "[maxon_epos_ethercat_sdk:Maxon::setDriveStateViaSdo] State "
              "Transition not implemented");
          addErrorToReading(ErrorType::SdoStateTransitionError);
          success = false;
      }
      break;

    case DriveState::OperationEnabled:
      switch (currentDriveState) {
        case DriveState::SwitchOnDisabled:
          success &= stateTransitionViaSdo(StateTransition::_2);
          success &= stateTransitionViaSdo(StateTransition::_3);
          success &= stateTransitionViaSdo(StateTransition::_4);
          break;
        case DriveState::ReadyToSwitchOn:
          success &= stateTransitionViaSdo(StateTransition::_3);
          success &= stateTransitionViaSdo(StateTransition::_4);
          break;
        case DriveState::SwitchedOn:
          success &= stateTransitionViaSdo(StateTransition::_4);
          break;
        case DriveState::OperationEnabled:
          success &= true;
          break;
        case DriveState::QuickStopActive:
          success &= stateTransitionViaSdo(StateTransition::_12);
          success &= stateTransitionViaSdo(StateTransition::_2);
          success &= stateTransitionViaSdo(StateTransition::_3);
          success &= stateTransitionViaSdo(StateTransition::_4);
          break;
        case DriveState::Fault:
          success &= stateTransitionViaSdo(StateTransition::_15);
          success &= stateTransitionViaSdo(StateTransition::_2);
          success &= stateTransitionViaSdo(StateTransition::_3);
          success &= stateTransitionViaSdo(StateTransition::_4);
          break;
        default:
          MELO_ERROR_STREAM(
              "[maxon_epos_ethercat_sdk:Maxon::setDriveStateViaSdo] State "
              "Transition not implemented");
          addErrorToReading(ErrorType::SdoStateTransitionError);
          success = false;
      }
      break;

    case DriveState::QuickStopActive:
      switch (currentDriveState) {
        case DriveState::SwitchOnDisabled:
          success &= stateTransitionViaSdo(StateTransition::_2);
          success &= stateTransitionViaSdo(StateTransition::_3);
          success &= stateTransitionViaSdo(StateTransition::_4);
          success &= stateTransitionViaSdo(StateTransition::_11);
          break;
        case DriveState::ReadyToSwitchOn:
          success &= stateTransitionViaSdo(StateTransition::_3);
          success &= stateTransitionViaSdo(StateTransition::_4);
          success &= stateTransitionViaSdo(StateTransition::_11);
          break;
        case DriveState::SwitchedOn:
          success &= stateTransitionViaSdo(StateTransition::_4);
          success &= stateTransitionViaSdo(StateTransition::_11);
          break;
        case DriveState::OperationEnabled:
          success &= stateTransitionViaSdo(StateTransition::_11);
          break;
        case DriveState::QuickStopActive:
          success &= true;
          break;
        case DriveState::Fault:
          success &= stateTransitionViaSdo(StateTransition::_15);
          success &= stateTransitionViaSdo(StateTransition::_2);
          success &= stateTransitionViaSdo(StateTransition::_3);
          success &= stateTransitionViaSdo(StateTransition::_4);
          success &= stateTransitionViaSdo(StateTransition::_11);
          break;
        default:
          MELO_ERROR_STREAM(
              "[maxon_epos_ethercat_sdk:Maxon::setDriveStateViaSdo] State "
              "Transition not implemented");
          addErrorToReading(ErrorType::SdoStateTransitionError);
          success = false;
      }
      break;

    default:
      MELO_ERROR_STREAM(
          "[maxon_epos_ethercat_sdk:Maxon::setDriveStateViaSdo] State "
          "Transition not implemented");
      addErrorToReading(ErrorType::SdoStateTransitionError);
      success = false;
  }
  return success;
}

bool Maxon::stateTransitionViaSdo(const StateTransition& stateTransition) {
  Controlword controlword;
  switch (stateTransition) {
    case StateTransition::_2:
      controlword.setStateTransition2();
      return setControlwordViaSdo(controlword);
      break;
    case StateTransition::_3:
      controlword.setStateTransition3();
      return setControlwordViaSdo(controlword);
      break;
    case StateTransition::_4:
      controlword.setStateTransition4();
      return setControlwordViaSdo(controlword);
      break;
    case StateTransition::_5:
      controlword.setStateTransition5();
      return setControlwordViaSdo(controlword);
      break;
    case StateTransition::_6:
      controlword.setStateTransition6();
      return setControlwordViaSdo(controlword);
      break;
    case StateTransition::_7:
      controlword.setStateTransition7();
      return setControlwordViaSdo(controlword);
      break;
    case StateTransition::_8:
      controlword.setStateTransition8();
      return setControlwordViaSdo(controlword);
      break;
    case StateTransition::_9:
      controlword.setStateTransition9();
      return setControlwordViaSdo(controlword);
      break;
    case StateTransition::_10:
      controlword.setStateTransition10();
      return setControlwordViaSdo(controlword);
      break;
    case StateTransition::_11:
      controlword.setStateTransition11();
      return setControlwordViaSdo(controlword);
      break;
    case StateTransition::_12:
      controlword.setStateTransition12();
      return setControlwordViaSdo(controlword);
      break;
    case StateTransition::_15:
      controlword.setStateTransition15();
      return setControlwordViaSdo(controlword);
      break;
    default:
      MELO_ERROR_STREAM(
          "[maxon_epos_ethercat_sdk:Maxon::stateTransitionViaSdo] State "
          "Transition not implemented");
      addErrorToReading(ErrorType::SdoStateTransitionError);
      return false;
  }
}

bool Maxon::setDriveStateViaPdo(const DriveState& driveState,
                                const bool waitForState) {
  bool success = false;
  /*
  ** locking the mutex_
  ** This is not done with a lock_guard here because during the waiting time the
  ** mutex_ must be unlocked periodically such that PDO writing (and thus state
  ** changes) may occur at all!
  */
  mutex_.lock();

  // reset the "stateChangeSuccessful_" flag to false such that a new successful
  // state change can be detected
  stateChangeSuccessful_ = false;

  // make the state machine realize that a state change will have to happen
  conductStateChange_ = true;

  // overwrite the target drive state
  targetDriveState_ = driveState;

  // set the hasRead flag to false such that at least one new reading will be
  // available when starting the state change
  hasRead_ = false;

  // set the time point of the last pdo change to now
  driveStateChangeTimePoint_ = std::chrono::steady_clock::now();

  // set a temporary time point to prevent getting caught in an infinite loop
  auto driveStateChangeStartTimePoint = std::chrono::steady_clock::now();

  // return if no waiting is requested
  if (!waitForState) {
    // unlock the mutex
    mutex_.unlock();
    // return true if no waiting is requested
    return true;
  }

  // Wait for the state change to be successful
  // during the waiting time the mutex MUST be unlocked!

  while (true) {
    // break loop as soon as the state change was successful
    if (stateChangeSuccessful_) {
      success = true;
      break;
    }

    // break the loop if the state change takes too long
    // this prevents a freezing of the end user's program if the hardware is not
    // able to change it's state.
    if ((std::chrono::duration_cast<std::chrono::microseconds>(
             std::chrono::steady_clock::now() - driveStateChangeStartTimePoint))
            .count() > configuration_.driveStateChangeMaxTimeout) {
      break;
    }
    // unlock the mutex during sleep time
    mutex_.unlock();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    // lock the mutex to be able to check the success flag
    mutex_.lock();
  }
  // unlock the mutex one last time
  mutex_.unlock();
  return success;
}

Controlword Maxon::getNextStateTransitionControlword(
    const DriveState& requestedDriveState,
    const DriveState& currentDriveState) {
  Controlword controlword;
  controlword.setAllFalse();
  switch (requestedDriveState) {
    case DriveState::SwitchOnDisabled:
      switch (currentDriveState) {
        case DriveState::SwitchOnDisabled:
          MELO_ERROR_STREAM(
              "[maxon_epos_ethercat_sdk:Maxon::"
              "getNextStateTransitionControlword] "
              << "drive state has already been reached for '" << name_ << "'");
          addErrorToReading(ErrorType::PdoStateTransitionError);
          break;
        case DriveState::ReadyToSwitchOn:
          controlword.setStateTransition7();
          break;
        case DriveState::SwitchedOn:
          controlword.setStateTransition10();
          break;
        case DriveState::OperationEnabled:
          controlword.setStateTransition9();
          break;
        case DriveState::QuickStopActive:
          controlword.setStateTransition12();
          break;
        case DriveState::Fault:
          controlword.setStateTransition15();
          break;
        default:
          MELO_ERROR_STREAM(
              "[maxon_epos_ethercat_sdk:Maxon::"
              "getNextStateTransitionControlword] "
              << "PDO state transition not implemented for '" << name_ << "'\n"
              << "Current: " << currentDriveState << "\n"
              << "Requested: " << requestedDriveState);
          addErrorToReading(ErrorType::PdoStateTransitionError);
      }
      break;

    case DriveState::ReadyToSwitchOn:
      switch (currentDriveState) {
        case DriveState::SwitchOnDisabled:
          controlword.setStateTransition2();
          break;
        case DriveState::ReadyToSwitchOn:
          MELO_ERROR_STREAM(
              "[maxon_epos_ethercat_sdk:Maxon::"
              "getNextStateTransitionControlword] "
              << "drive state has already been reached for '" << name_ << "'");
          addErrorToReading(ErrorType::PdoStateTransitionError);
          break;
        case DriveState::SwitchedOn:
          controlword.setStateTransition6();
          break;
        case DriveState::OperationEnabled:
          controlword.setStateTransition8();
          break;
        case DriveState::QuickStopActive:
          controlword.setStateTransition12();
          break;
        case DriveState::Fault:
          controlword.setStateTransition15();
          break;
        default:
          MELO_ERROR_STREAM(
              "[maxon_epos_ethercat_sdk:Maxon::"
              "getNextStateTransitionControlword] "
              << "PDO state transition not implemented for '" << name_ << "'\n"
              << "Current: " << currentDriveState << "\n"
              << "Requested: " << requestedDriveState);
          addErrorToReading(ErrorType::PdoStateTransitionError);
      }
      break;

    case DriveState::SwitchedOn:
      switch (currentDriveState) {
        case DriveState::SwitchOnDisabled:
          controlword.setStateTransition2();
          break;
        case DriveState::ReadyToSwitchOn:
          controlword.setStateTransition3();
          break;
        case DriveState::SwitchedOn:
          MELO_ERROR_STREAM(
              "[maxon_epos_ethercat_sdk:Maxon::"
              "getNextStateTransitionControlword] "
              << "drive state has already been reached for '" << name_ << "'");
          addErrorToReading(ErrorType::PdoStateTransitionError);
          break;
        case DriveState::OperationEnabled:
          controlword.setStateTransition5();
          break;
        case DriveState::QuickStopActive:
          controlword.setStateTransition12();
          break;
        case DriveState::Fault:
          controlword.setStateTransition15();
          break;
        default:
          MELO_ERROR_STREAM(
              "[maxon_epos_ethercat_sdk:Maxon::"
              "getNextStateTransitionControlword] "
              << "PDO state transition not implemented for '" << name_ << "'\n"
              << "Current: " << currentDriveState << "\n"
              << "Requested: " << requestedDriveState);
          addErrorToReading(ErrorType::PdoStateTransitionError);
      }
      break;

    case DriveState::OperationEnabled:
      switch (currentDriveState) {
        case DriveState::SwitchOnDisabled:
          controlword.setStateTransition2();
          break;
        case DriveState::ReadyToSwitchOn:
          controlword.setStateTransition3();
          break;
        case DriveState::SwitchedOn:
          controlword.setStateTransition4();
          break;
        case DriveState::OperationEnabled:
          MELO_ERROR_STREAM(
              "[maxon_epos_ethercat_sdk:Maxon::"
              "getNextStateTransitionControlword] "
              << "drive state has already been reached for '" << name_ << "'");
          addErrorToReading(ErrorType::PdoStateTransitionError);
          break;
        case DriveState::QuickStopActive:
          controlword.setStateTransition12();
          break;
        case DriveState::Fault:
          controlword.setStateTransition15();
          break;
        default:
          MELO_ERROR_STREAM(
              "[maxon_epos_ethercat_sdk:Maxon::"
              "getNextStateTransitionControlword] "
              << "PDO state transition not implemented for '" << name_ << "'\n"
              << "Current: " << currentDriveState << "\n"
              << "Requested: " << requestedDriveState);
          addErrorToReading(ErrorType::PdoStateTransitionError);
      }
      break;

    case DriveState::QuickStopActive:
      switch (currentDriveState) {
        case DriveState::SwitchOnDisabled:
          controlword.setStateTransition2();
          break;
        case DriveState::ReadyToSwitchOn:
          controlword.setStateTransition3();
          break;
        case DriveState::SwitchedOn:
          controlword.setStateTransition4();
          break;
        case DriveState::OperationEnabled:
          controlword.setStateTransition11();
          break;
        case DriveState::QuickStopActive:
          MELO_ERROR_STREAM(
              "[maxon_epos_ethercat_sdk:Maxon::"
              "getNextStateTransitionControlword] "
              << "drive state has already been reached for '" << name_ << "'");
          addErrorToReading(ErrorType::PdoStateTransitionError);
          break;
        case DriveState::Fault:
          controlword.setStateTransition15();
          break;
        default:
          MELO_ERROR_STREAM(
              "[maxon_epos_ethercat_sdk:Maxon::"
              "getNextStateTransitionControlword] "
              << "PDO state transition not implemented for '" << name_ << "'\n"
              << "Current: " << currentDriveState << "\n"
              << "Requested: " << requestedDriveState);
          addErrorToReading(ErrorType::PdoStateTransitionError);
      }
      break;

    default:
      MELO_ERROR_STREAM(
          "[maxon_epos_ethercat_sdk:Maxon::getNextStateTransitionControlword] "
          << "PDO state cannot be reached for '" << name_ << "'");
      addErrorToReading(ErrorType::PdoStateTransitionError);
  }

  return controlword;
}

void Maxon::autoConfigurePdoSizes() {
  auto pdoSizes = bus_->getHardwarePdoSizes(static_cast<uint16_t>(address_));
  pdoInfo_.rxPdoSize_ = pdoSizes.first;
  pdoInfo_.txPdoSize_ = pdoSizes.second;
}

uint16_t Maxon::getTxPdoSize() { return pdoInfo_.txPdoSize_; }

uint16_t Maxon::getRxPdoSize() { return pdoInfo_.rxPdoSize_; }

void Maxon::engagePdoStateMachine() {
  // locking the mutex
  std::lock_guard<std::recursive_mutex> lock(mutex_);

  // elapsed time since the last new controlword
  auto microsecondsSinceChange =
      (std::chrono::duration_cast<std::chrono::microseconds>(
           std::chrono::steady_clock::now() - driveStateChangeTimePoint_))
          .count();

  // get the current state
  // since we wait until "hasRead" is true, this is guaranteed to be a newly
  // read value
  const DriveState currentDriveState = reading_.getDriveState();
  // check if the state change already was successful:
  if (currentDriveState == targetDriveState_) {
    numberOfSuccessfulTargetStateReadings_++;
    if (numberOfSuccessfulTargetStateReadings_ >=
        configuration_.minNumberOfSuccessfulTargetStateReadings) {
      // disable the state machine
      conductStateChange_ = false;
      numberOfSuccessfulTargetStateReadings_ = 0;
      stateChangeSuccessful_ = true;
      return;
    }
  } else if (microsecondsSinceChange >
             configuration_.driveStateChangeMinTimeout) {
    // get the next controlword from the state machine
    controlword_ =
        getNextStateTransitionControlword(targetDriveState_, currentDriveState);
    driveStateChangeTimePoint_ = std::chrono::steady_clock::now();
  }

  // set the "hasRead" variable to false such that there will definitely be a
  // new reading when this method is called again
  hasRead_ = false;
}
}  // namespace maxon
