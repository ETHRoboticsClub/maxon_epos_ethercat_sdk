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
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // PDO mapping
  success &= mapPdos(rxPdoTypeEnum_, txPdoTypeEnum_);

  // Set Interpolation
  success &= sdoVerifyWrite(OD_INDEX_INTERPOLATION_TIME_PERIOD, 0x01, false,
                            static_cast<uint8_t>(0),
                            configuration_.configRunSdoVerifyTimeout);

  success &= sdoVerifyWrite(OD_INDEX_INTERPOLATION_TIME_PERIOD, 0x02, false,
                            static_cast<int8_t>(-3),
                            configuration_.configRunSdoVerifyTimeout);

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
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  return success;
}

void Maxon::preShutdown() {
  // setDriveStateViaSdo(DriveState::QuickStopActive);
  setDriveStateViaSdo(DriveState::SwitchOnDisabled);
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
  if (reading_.getDriveState() == DriveState::FaultReactionActive) {
    MELO_ERROR_STREAM("[maxon_epos_ethercat_sdk:Maxon::updateRead] '"
                      << name_ << "' is in drive state 'FaultReactionActive'");
  }

  // Print warning if drive is in Fault state.
  if (reading_.getDriveState() == DriveState::Fault) {
    MELO_ERROR_STREAM("[maxon_epos_ethercat_sdk:Maxon::updateRead] '"
                      << name_ << "' is in drive state 'Fault'");
  }
}
//this is a lock safe function already usibg the mutex lock
void Maxon::stageCommand(const Command& command) {
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
  return homereference / 4096.0;
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
  MELO_INFO_STREAM("SI Unit Velocity: " << std::hex << siUnitVel);

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
  //the signature for saving is 0x65766173
  uint32_t signature = static_cast<uint32_t>(0x65766173);
  return sdoVerifyWrite(OD_STORE_PARAM, 0x01, false, signature);
}


bool Maxon::doHoming() {
  bool success = true;

    // set the homing mwthod via sdo
    success &= sdoVerifyWrite(OD_HOMING_METHOD, 0x00, false, static_cast<int8_t>(37), configuration_.configRunSdoVerifyTimeout);

    // change the operation mode to homing
    // start the homing process by setting the controlword
    // for homing operation start controlword bit 4 -> 1
    Command command;
    command.setModeOfOperation(maxon::ModeOfOperationEnum::HomingMode);
    stageCommand(command);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    controlword_.homingOperationStart_ = true;

    /// wait for the homing to finish 

    bool homing_finished = false;
    uint count = 0;

    while ( !homing_finished && (count < 20)) {
      MELO_INFO_STREAM("homing in progress");
      Reading reading = getReading();
      Statusword status = reading.getStatusword();
      MELO_INFO_STREAM("Statusword:" << status);
      homing_finished = status.homingFinished();
      count ++;
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }


    if(homing_finished) {
      MELO_INFO_STREAM("homing finished");
      return true;
    } else {
      MELO_ERROR_STREAM("Maximum number of retries reached");
      return false;
    }
  return false;
}

bool Maxon::setDriveStateViaSdo(const DriveState& driveState) {
  bool success = true;
  Statusword currentStatusword;
  success &= getStatuswordViaSdo(currentStatusword);
  DriveState currentDriveState = currentStatusword.getDriveState();

  // do the adequate state changes (via sdo) depending on the requested and
  // current drive states
  switch (driveState) {
    // Target: switch on disabled
    // This is the lowest state in which the state machine can be brought over
    // EtherCAT
    case DriveState::SwitchOnDisabled:
      switch (currentDriveState) {
        case DriveState::SwitchOnDisabled:
          success &= true;
          break;
        case DriveState::ReadyToSwitchOn:
          success &= stateTransitionViaSdo(StateTransition::_7);
          break;
        case DriveState::SwitchedOn:
          success &= stateTransitionViaSdo(StateTransition::_10);
          break;
        case DriveState::OperationEnabled:
          success &= stateTransitionViaSdo(StateTransition::_9);
          break;
        case DriveState::QuickStopActive:
          success &= stateTransitionViaSdo(StateTransition::_12);
          break;
        case DriveState::Fault:
          success &= stateTransitionViaSdo(StateTransition::_15);
          break;
        default:
          MELO_ERROR_STREAM(
              "[maxon_epos_ethercat_sdk:Maxon::setDriveStateViaSdo] State "
              "Transition not implemented");
          addErrorToReading(ErrorType::SdoStateTransitionError);
          success = false;
      }
      break;

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
