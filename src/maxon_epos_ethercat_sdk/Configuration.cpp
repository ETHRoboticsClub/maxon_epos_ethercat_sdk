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

#include "maxon_epos_ethercat_sdk/Configuration.hpp"

#include <iomanip>
#include <vector>
#include <map>
#include <algorithm>
#include <utility>

namespace maxon {
std::string modeOfOperationString(ModeOfOperationEnum modeOfOperation_) {
  switch (modeOfOperation_) {
    case ModeOfOperationEnum::NA:
      return "NA";
    case ModeOfOperationEnum::ProfiledVelocityMode:
      return "Profiled Velocity Mode";
    case ModeOfOperationEnum::CyclicSynchronousTorqueMode:
      return "Cyclic Synchronous Torque Mode";
    case ModeOfOperationEnum::CyclicJVPTMode:
      return "Cyclic JVPT Mode";
    case ModeOfOperationEnum::CyclicFreezeMode:
      return "Cyclic Joint Freeze Mode";
    default:
      return "Unsupported Mode of Operation";
  }
}

std::string rxPdoString(RxPdoTypeEnum rxPdo) {
  switch (rxPdo) {
    case RxPdoTypeEnum::NA:
      return "NA";
    case RxPdoTypeEnum::RxPdoStandard:
      return "Rx PDO Standard";
    case RxPdoTypeEnum::RxPdoCST:
      return "Rx PDO CST";
    case RxPdoTypeEnum::RxPdoPVM:
      return "Rx PDO PVM";
    case RxPdoTypeEnum::RxPdoJVPT:
      return "Rx PDO JVPT";
    case RxPdoTypeEnum::RxPdoFreeze:
      return "Rx PDO Freeze";
    default:
      return "Unsupported Type";
  }
}

std::string txPdoString(TxPdoTypeEnum txPdo) {
  switch (txPdo) {
    case TxPdoTypeEnum::NA:
      return "NA";
    case TxPdoTypeEnum::TxPdoCST:
      return "Tx PDO CST";
    case TxPdoTypeEnum::TxPdoPVM:
      return "Tx PDO PVM";
    case TxPdoTypeEnum::TxPdoStandard:
      return "Tx PDO Standard";
    case TxPdoTypeEnum::TxPdoJVPT:
      return "Tx PDO JVPT";
    case TxPdoTypeEnum::TxPdoFreeze:
      return "Tx PDO Freeze";
    default:
      return "Unsupported Type";
  }
}

std::ostream& operator<<(std::ostream& os, const Configuration& configuration) {
  std::string modeOfOperation_ =
      modeOfOperationString(configuration.modesOfOperation[0]);
  unsigned int tmp3 = modeOfOperation_.size();
  unsigned int len2 = tmp3;
  len2++;

  os << std::boolalpha << std::left << std::setw(43) << std::setfill('-') << "|"
     << std::setw(len2 + 2) << "-"
     << "|\n"
     << std::setfill(' ') << std::setw(43 + len2 + 2) << "| Configuration"
     << "|\n"
     << std::setw(43) << std::setfill('-') << "|" << std::setw(len2 + 2) << "+"
     << "|\n"
     << std::setfill(' ') << std::setw(43) << "| 1st Mode of Operation:"
     << "| " << std::setw(len2) << modeOfOperation_ << "|\n"
     << std::setw(43) << "| Config Run SDO verify timeout:"
     << "| " << std::setw(len2) << configuration.configRunSdoVerifyTimeout
     << "|\n"
     << std::setw(43) << "| Print Debug Messages:"
     << "| " << std::setw(len2) << configuration.printDebugMessages << "|\n"
     << std::setw(43) << "| Drive State Change Min Timeout:"
     << "| " << std::setw(len2) << configuration.driveStateChangeMinTimeout
     << "|\n"
     << std::setw(43) << "| Drive State Change Max Timeout:"
     << "| " << std::setw(len2) << configuration.driveStateChangeMaxTimeout
     << "|\n"
     << std::setw(43) << "| Min Successful Target State Readings:"
     << "| " << std::setw(len2)
     << configuration.minNumberOfSuccessfulTargetStateReadings << "|\n"
     << std::setw(43) << "| Force Append Equal Error:"
     << "| " << std::setw(len2) << configuration.forceAppendEqualError << "|\n"
     << std::setw(43) << "| Force Append Equal Fault:"
     << "| " << std::setw(len2) << configuration.forceAppendEqualFault << "|\n"
     << std::setw(43) << "| Error Storage Capacity"
     << "| " << std::setw(len2) << configuration.errorStorageCapacity << "|\n"
     << std::setw(43) << "| Fault Storage Capacity"
     << "| " << std::setw(len2) << configuration.faultStorageCapacity << "|\n"
     << std::setw(43) << std::setfill('-') << "|" << std::setw(len2 + 2) << "+"
     << "|\n"
     << std::setfill(' ') << std::noboolalpha << std::right;
  return os;
}

std::pair<RxPdoTypeEnum, TxPdoTypeEnum> Configuration::getPdoTypeSolution()
    const {
  // clang-format off
  // {ModeOfOperationEnum1, ..., ModeOfOperationEnumN} -> {RxPdoTypeEnum, TxPdoTypeEnum}
  const std::map<std::vector<ModeOfOperationEnum>, std::pair<RxPdoTypeEnum, TxPdoTypeEnum>> modes2PdoTypeMap = {
      {
        { ModeOfOperationEnum::CyclicSynchronousTorqueMode },
        { RxPdoTypeEnum::RxPdoCST, TxPdoTypeEnum::TxPdoCST }
      },
      {
        { ModeOfOperationEnum::ProfiledVelocityMode },
        { RxPdoTypeEnum::RxPdoPVM, TxPdoTypeEnum::TxPdoPVM }
      },
      {
        { ModeOfOperationEnum::CyclicJVPTMode },
        { RxPdoTypeEnum::RxPdoJVPT, TxPdoTypeEnum::TxPdoJVPT }
      },
      {
        { ModeOfOperationEnum::CyclicJVPTMode, ModeOfOperationEnum::CyclicFreezeMode, ModeOfOperationEnum::HomingMode},
        { RxPdoTypeEnum::RxPdoJVPT, TxPdoTypeEnum::TxPdoJVPT }
      },
      {
        { ModeOfOperationEnum::NA },
        { RxPdoTypeEnum::NA, TxPdoTypeEnum::NA }
      },
  };
  // clang-format on

  bool setsAreEqual;
  for (const auto& modes2PdoTypeEntry : modes2PdoTypeMap) {
    setsAreEqual = true;
    for (const auto& modeOfOperation : modesOfOperation)
      setsAreEqual &=
          std::find(modes2PdoTypeEntry.first.begin(),
                    modes2PdoTypeEntry.first.end(),
                    modeOfOperation) != modes2PdoTypeEntry.first.end();
    for (const auto& modeOfOperation : modes2PdoTypeEntry.first)
      setsAreEqual &=
          std::find(modesOfOperation.begin(), modesOfOperation.end(),
                    modeOfOperation) != modesOfOperation.end();
    if (setsAreEqual) return modes2PdoTypeEntry.second;
  }
  return std::pair<RxPdoTypeEnum, TxPdoTypeEnum>{RxPdoTypeEnum::NA,
                                                 TxPdoTypeEnum::NA};
}

std::vector<std::string> Configuration::configurationFaults() const {
  std::vector<std::string> faults;
  const auto pdoTypePair = getPdoTypeSolution();
  if (pdoTypePair.first == RxPdoTypeEnum::NA || pdoTypePair.second == TxPdoTypeEnum::NA)
    faults.push_back("modes_of_operation (" + std::to_string(modesOfOperation.size()) +
                     " configured) map onto no supported PDO pair");
  if (driveStateChangeMinTimeout > driveStateChangeMaxTimeout)
    faults.push_back("drive_state_change_min_timeout exceeds drive_state_change_max_timeout");
  // Divisors of the torque/current scaling (Maxon::loadConfiguration): zero
  // makes every command's unit conversion undefined.
  if (!(nominalCurrentA > 0.0 && torqueConstantNmA > 0.0))
    faults.push_back("nominal_current and torque_constant must both be > 0");
  // Written raw to 0x34C6:03 as UNSIGNED32 by the DAMPING e-stop: a negative
  // value wraps to a huge gain. The cap is 100x a typical D of 1e4.
  if (!(jvptDampingDGain >= 0.0 && jvptDampingDGain <= 1.0e6))
    faults.push_back("JVPT_damping_D_gain must be in [0, 1e6]");
  return faults;
}
}  // namespace maxon
