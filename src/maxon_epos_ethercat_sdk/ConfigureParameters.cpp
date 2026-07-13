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

#include <array>
#include <thread>

#include "maxon_epos_ethercat_sdk/Maxon.hpp"
#include "maxon_epos_ethercat_sdk/ObjectDictionary.hpp"

namespace maxon {
bool Maxon::mapPdos(RxPdoTypeEnum rxPdoTypeEnum, TxPdoTypeEnum txPdoTypeEnum) {
  uint8_t subIndex;

  bool rxSuccess = true;
  switch (rxPdoTypeEnum) {
    case RxPdoTypeEnum::RxPdoStandard: {
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Rx Pdo: "
                       << "Standard Mode");

      // Disable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x01, false,
                                  OD_INDEX_RX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write number of objects
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }

    case RxPdoTypeEnum::RxPdoCST: {
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Rx Pdo: "
                       << "Cyclic Synchronous Troque Mode");

      // Disable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x01, false,
                                  OD_INDEX_RX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects...
      std::array<uint32_t, 4> objects{
          (OD_INDEX_TARGET_TORQUE << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_OFFSET_TORQUE << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_CONTROLWORD << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_MODES_OF_OPERATION << 16) | (0x00 << 8) |
              sizeof(int8_t) * 8,
      };

      subIndex = 0;
      for (const auto& objectIndex : objects) {
        subIndex += 1;
        rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, subIndex, false,
                                    objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }

      // Write number of objects
      rxSuccess &=
          sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false, subIndex,
                         configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }
    
    case RxPdoTypeEnum::RxPdoPVM: {
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Rx Pdo: "
                       << "Profile Velocity Mode");

      // Disable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x01, false,
                                  OD_INDEX_RX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects...
      std::array<uint32_t, 5> objects{
          (OD_INDEX_CONTROLWORD << 16) | (0x00 << 8) | sizeof(uint16_t) * 8,
          (OD_INDEX_TARGET_VELOCITY << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_PROFILE_ACCELERATION << 16) | (0x00 << 8) |
              sizeof(uint32_t) * 8,
          (OD_INDEX_PROFILE_DECELERATION << 16) | (0x00 << 8) |
              sizeof(uint32_t) * 8,
          (OD_INDEX_MOTION_PROFILE_TYPE << 16) | (0x00 << 8) |
              sizeof(int16_t) * 8,
      };

      subIndex = 0;
      for (const auto& objectIndex : objects) {
        subIndex += 1;
        rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, subIndex, false,
                                    objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }

      // Write number of objects
      rxSuccess &=
          sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false, subIndex,
                         configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }

    case RxPdoTypeEnum::RxPdoJVPT:{
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Rx Pdo: "
                       << "Cyclic Joint Velocity Position Torque Mode");

      // Disable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);
      
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x01, false,
                                  OD_INDEX_RX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects
      std::array<uint32_t, 5> objects{
          (OD_INDEX_TARGET_JOINT_TORQUE << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_TARGET_JOINT_POSITION << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_TARGET_JOINT_VELOCITY << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_CONTROLWORD << 16) | (0x00 << 8) | sizeof(uint16_t) * 8,
          (OD_INDEX_MODES_OF_OPERATION << 16) | (0x00 << 8) | sizeof(int8_t) * 8,
      };

      subIndex = 0;
      for (const auto& objectIndex : objects) {
        subIndex += 1;
        rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, subIndex, false,
                                    objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }
      //Write number of objects
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false, subIndex,
                         configuration_.configRunSdoVerifyTimeout);
      
      // Enable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }


    case RxPdoTypeEnum::RxPdoFreeze:{
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Rx Pdo: "
                       << "Cyclic Joint Freeze Mode");

      // Disable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);
      
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x01, false,
                                  OD_INDEX_RX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects
      std::array<uint32_t, 5> objects{
          (OD_INDEX_TARGET_JOINT_TORQUE << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_TARGET_JOINT_POSITION << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_TARGET_JOINT_VELOCITY << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_CONTROLWORD << 16) | (0x00 << 8) | sizeof(uint16_t) * 8,
          (OD_INDEX_MODES_OF_OPERATION << 16) | (0x00 << 8) | sizeof(int8_t) * 8,
      };

      subIndex = 0;
      for (const auto& objectIndex : objects) {
        subIndex += 1;
        rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, subIndex, false,
                                    objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }
      //Write number of objects
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_MAPPING_3, 0x00, false, subIndex,
                         configuration_.configRunSdoVerifyTimeout);
      
      // Enable PDO
      rxSuccess &= sdoVerifyWrite(OD_INDEX_RX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }

    case RxPdoTypeEnum::NA:
      MELO_ERROR_STREAM(
          "[maxon_epos_ethercat_sdk:Maxon::mapPdos] Cannot map "
          "RxPdoTypeEnum::NA, PdoType not configured properly");
      addErrorToReading(ErrorType::PdoMappingError);
      rxSuccess = false;
      break;
    default:  // Non-implemented type
      MELO_ERROR_STREAM(
          "[maxon_epos_ethercat_sdk:Maxon::mapPdos] Cannot map unimplemented "
          "RxPdo, PdoType not configured properly");
      addErrorToReading(ErrorType::PdoMappingError);
      rxSuccess = false;
      break;
  }

  bool txSuccess = true;
  switch (txPdoTypeEnum) {
    case TxPdoTypeEnum::TxPdoStandard: {
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Tx Pdo: "
                       << "Standard Mode");

      // Disable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x01, false,
                                  OD_INDEX_TX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write number of objects
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }

    case TxPdoTypeEnum::TxPdoCST: {
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Tx Pdo: "
                       << "Cyclic Synchronous Torque Mode");

      // Disable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x01, false,
                                  OD_INDEX_TX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects...
      std::array<uint32_t, 4> objects{
          (OD_INDEX_STATUSWORD << 16) | (0x00 << 8) | sizeof(uint16_t) * 8,
          (OD_INDEX_TORQUE_ACTUAL << 16) | (0x00 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_VELOCITY_ACTUAL << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_POSITION_ACTUAL << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
      };

      subIndex = 0;
      for (const auto& objectIndex : objects) {
        subIndex += 1;
        txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, subIndex, false,
                                    objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }

      // Write number of objects
      txSuccess &=
          sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false, subIndex,
                         configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }

    case TxPdoTypeEnum::TxPdoPVM: {
      // (OD_INDEX_TORQUE_ACTUAL << 16) | (0x01 << 8) | sizeof(int16_t) * 8

      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Tx Pdo: "
                       << "Profile Velocity Mode");

      // Disable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x01, false,
                                  OD_INDEX_TX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects...
      std::array<uint32_t, 3> objects{
          (OD_INDEX_STATUSWORD << 16) | (0x00 << 8) | sizeof(uint16_t) * 8,
          (OD_INDEX_VELOCITY_DEMAND << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_VELOCITY_ACTUAL << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
      };

      subIndex = 0;
      for (const auto& objectIndex : objects) {
        subIndex += 1;
        txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, subIndex, false,
                                    objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }

      // Write number of objects
      txSuccess &=
          sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false, subIndex,
                         configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }

    case TxPdoTypeEnum::TxPdoJVPT:{
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Tx Pdo: "
                       << "Cyclic Joint Velocity Position Torque Mode");

      // Disable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x01, false,
                                  OD_INDEX_TX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects
      // TxPDO is 7 mapped objects: the 5 core feedbacks + the two drive
      // temperatures (motor=0x3201/0x02, power-stage=0x3201/0x01). 7 <= 8, so it
      // respects the "<=8 PDOs per direction" bound that motivated the original
      // trim from 12. Array SIZE and order MUST match TxPdoJVPT (TxPdo.hpp) and
      // the parse in Maxon.cpp::updateRead. NB: array<7> => exactly 7 entries
      // written and the object count set to 7.
      std::array<uint32_t, 7> objects{
          (OD_INDEX_STATUSWORD << 16) | (0x00 << 8) | sizeof(uint16_t) * 8,
          (OD_INDEX_JOINT_TORQUE_EST << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_JOINT_VELOCITY_ACTUAL << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_JOINT_POSITION_ACTUAL << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_JOINT_CURRENT_ACTUAL << 16) | (0x01 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_TEMPERATURE << 16) | (0x02 << 8) | sizeof(int16_t) * 8,  // motor temp
          (OD_INDEX_TEMPERATURE << 16) | (0x01 << 8) | sizeof(int16_t) * 8,  // power-stage temp
          // (OD_INDEX_VELOCITY_DEMAND << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          // (OD_INDEX_CURRENT_DEMAND << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          // (OD_INDEX_POSITION_DEMAND << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          // (OD_INDEX_I2T << 16) | (0x01 << 8) | sizeof(uint16_t) * 8,
          // (OD_INDEX_I2T << 16) | (0x02 << 8) | sizeof(uint16_t) * 8,
      };

      subIndex = 0;
      for (const auto& objectIndex : objects) {
        subIndex += 1;
        txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, subIndex, false,
                                    objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }

      // Write number of objects
      txSuccess &=
          sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false, subIndex,
                         configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }

    case TxPdoTypeEnum::TxPdoFreeze:{
      MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::mapPdos] Tx Pdo: "
                       << "Cyclic Freeze Mode");

      // Disable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false,
                                  static_cast<uint8_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);

      // Write mapping
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x01, false,
                                  OD_INDEX_TX_PDO_MAPPING_3,
                                  configuration_.configRunSdoVerifyTimeout);

      // Write objects
      std::array<uint32_t, 12> objects{
          (OD_INDEX_STATUSWORD << 16) | (0x00 << 8) | sizeof(uint16_t) * 8,
          (OD_INDEX_JOINT_TORQUE_EST << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_JOINT_VELOCITY_ACTUAL << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_JOINT_POSITION_ACTUAL << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_JOINT_CURRENT_ACTUAL << 16) | (0x01 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_VELOCITY_DEMAND << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_CURRENT_DEMAND << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_POSITION_DEMAND << 16) | (0x00 << 8) | sizeof(int32_t) * 8,
          (OD_INDEX_TEMPERATURE << 16) | (0x02 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_TEMPERATURE << 16) | (0x01 << 8) | sizeof(int16_t) * 8,
          (OD_INDEX_I2T << 16) | (0x01 << 8) | sizeof(uint16_t) * 8,
          (OD_INDEX_I2T << 16) | (0x02 << 8) | sizeof(uint16_t) * 8,
      };

      subIndex = 0;
      for (const auto& objectIndex : objects) {
        subIndex += 1;
        txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, subIndex, false,
                                    objectIndex,
                                    configuration_.configRunSdoVerifyTimeout);
      }

      // Write number of objects
      txSuccess &=
          sdoVerifyWrite(OD_INDEX_TX_PDO_MAPPING_3, 0x00, false, subIndex,
                         configuration_.configRunSdoVerifyTimeout);

      // Enable PDO
      txSuccess &= sdoVerifyWrite(OD_INDEX_TX_PDO_ASSIGNMENT, 0x00, false,
                                  static_cast<uint8_t>(1),
                                  configuration_.configRunSdoVerifyTimeout);

      break;
    }


    case TxPdoTypeEnum::NA:
      MELO_ERROR_STREAM(
          "[maxon_epos_ethercat_sdk:Maxon::mapPdos] Cannot map "
          "TxPdoTypeEnum::NA, PdoType not configured properly");
      addErrorToReading(ErrorType::TxPdoMappingError);
      txSuccess = false;
      break;
    default:  // if any case was forgotten
      MELO_ERROR_STREAM(
          "[maxon_epos_ethercat_sdk:Maxon::mapPdos] Cannot map undefined "
          "TxPdo, PdoType not configured properly");
      addErrorToReading(ErrorType::TxPdoMappingError);
      txSuccess = false;
      break;
  }

  return (txSuccess && rxSuccess);
}

bool Maxon::configParam() {
  bool configSuccess = true;
  // Values to update during the configuration from the loaded yaml file.
  double position_to_inc =
      static_cast<double>(configuration_.positionEncoderResolution) /
      (2 * M_PI);

  MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::configParam] '" << name_
                   << "' using position_encoder_resolution="
                   << configuration_.positionEncoderResolution
                   << ", position_to_inc=" << position_to_inc);
  
  //JVPT related parameters

  uint32_t jvpt_p_gain = static_cast<uint32_t> (configuration_.jvptPGain);
  uint32_t jvpt_i_gain = static_cast<uint32_t> (configuration_.jvptIGain);
  uint32_t jvpt_d_gain = static_cast<uint32_t> (configuration_.jvptDGain);

  uint32_t jvpt_maximal_integral_value = static_cast<uint32_t> (configuration_.maxTorqueSI * 1000); //This value determines the output torque limit of the motor 1mNm is the unit

  // Effective DAMPING e-stop D gain (applied at runtime with P->0). Logged so
  // the per-joint choice is visible at boot even when the YAML key is absent
  // (struct default 100 then stands).
  MELO_INFO_STREAM("[maxon_epos_ethercat_sdk:Maxon::configParam] '" << name_
                   << "' DAMPING e-stop will use JVPT D="
                   << static_cast<uint32_t>(configuration_.jvptDampingDGain)
                   << " (raw 0x34C6:03), P=0; normal JVPT P=" << jvpt_p_gain
                   << " D=" << jvpt_d_gain);

  configSuccess &= sdoVerifyWrite(OD_INDEX_JVPT_PARAMETERS, 0x01, false,
                                  jvpt_p_gain,
                                  configuration_.configRunSdoVerifyTimeout);
  
  configSuccess &= sdoVerifyWrite(OD_INDEX_JVPT_PARAMETERS, 0x02, false,
                                  jvpt_i_gain,
                                  configuration_.configRunSdoVerifyTimeout);    

  configSuccess &= sdoVerifyWrite(OD_INDEX_JVPT_PARAMETERS, 0x03, false,  
                                  jvpt_d_gain,
                                  configuration_.configRunSdoVerifyTimeout);

  configSuccess &= sdoVerifyWrite(OD_INDEX_JVPT_PARAMETERS, 0x04, false,
                                  jvpt_maximal_integral_value,
                                  configuration_.configRunSdoVerifyTimeout);

  // Current controller gains. Config holds SI units (V/A and V/(A*s)); the
  // EPOS4 firmware spec stores them as UNSIGNED32 scaled by 1e6 (P, 0x30A0:01)
  // and 1e3 (I, 0x30A0:02). Defaults 1.17188 V/A and 3906.25 V/(A*s) map to
  // the spec defaults 1'171'880 and 3'906'250.
  uint32_t current_p_gain =
      static_cast<uint32_t>(configuration_.currentPGainSI * 1.0e6);
  uint32_t current_i_gain =
      static_cast<uint32_t>(configuration_.currentIGainSI * 1.0e3);

  configSuccess &= sdoVerifyWrite(OD_INDEX_CURRENT_CONTROL_PARAM, 0x01, false,
                                  current_p_gain,
                                  configuration_.configRunSdoVerifyTimeout);

  configSuccess &= sdoVerifyWrite(OD_INDEX_CURRENT_CONTROL_PARAM, 0x02, false,
                                  current_i_gain,
                                  configuration_.configRunSdoVerifyTimeout);


  //Limit Related Parameters

    int32_t soft_max_pos_limit = static_cast<int32_t>(
      configuration_.softMaxPosLimitSI * position_to_inc);
    int32_t soft_min_pos_limit = static_cast<int32_t>(
      configuration_.softMinPosLimitSI * position_to_inc);

  configSuccess &= sdoVerifyWrite(OD_INDEX_SOFT_LIMIT, 0x02, false,
                                  soft_max_pos_limit,
                                  configuration_.configRunSdoVerifyTimeout);

  configSuccess &= sdoVerifyWrite(OD_INDEX_SOFT_LIMIT, 0x01, false, 
                                  soft_min_pos_limit,
                                  configuration_.configRunSdoVerifyTimeout);

  // Homing parameters (must be written in pre-op/config phase)
  configSuccess &= sdoVerifyWrite(OD_HOMING_METHOD, 0x00, false,
                                  configuration_.homingMethod,
                                  configuration_.configRunSdoVerifyTimeout);
  configSuccess &= sdoVerifyWrite(OD_INDEX_HOME_POSITION, 0x00, false,
                                  configuration_.homingPosition,
                                  configuration_.configRunSdoVerifyTimeout);
  // Home Offset (0x3673) is deliberately forced to ZERO. The URDF zero is now
  // applied per-boot in doHoming() via Method 37 / Home Position (0x30B0 =
  // current_raw - homing_offset) — the only path this firmware honours. A
  // nonzero 0x3673 (e.g. one previously persisted to NVM via MAXON_STORE_PARAMS)
  // would stack a second shift on top of that and corrupt the zero, so it must
  // stay neutral. To flush an old NVM value: run once with MAXON_STORE_PARAMS=1,
  // then power-cycle.
  configSuccess &= sdoVerifyWrite(OD_INDEX_HOME_OFFSET, 0x00, false,
                                  static_cast<int32_t>(0),
                                  configuration_.configRunSdoVerifyTimeout);



  if (configSuccess) {
    MELO_INFO("Setting configuration parameters succeeded.");
  } else {
    MELO_ERROR("Setting configuration parameters failed.");
  }

  // Read back and log what the drive actually holds for the current/velocity/
  // JVPT loops (diagnostic; does not affect configSuccess). Lets the operator
  // confirm each joint's live tuning — and exposes the velocity loop, which
  // this stack never writes (drive-NVM only).
  logControllerGainsSDO();

  return configSuccess;
}
}  // namespace maxon
