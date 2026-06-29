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

#include <iomanip>

#include "maxon_epos_ethercat_sdk/Maxon.hpp"
#include "maxon_epos_ethercat_sdk/ObjectDictionary.hpp"

namespace maxon {

// Decodes the EPOS4 error code read from object 0x603F. Only codes whose
// meaning was confirmed against the maxon EPOS4 Communication Guide / Firmware
// Specification (ch. 7 "Error Handling") are mapped; anything else returns an
// explicit pointer to the spec rather than a silent/empty string. Add more
// codes here as they are verified against that document.
std::string errorCodeToString(uint16_t code) {
  switch (code) {
    case 0x0000: return "No error";
    case 0x1000: return "Generic error";
    case 0x2310: return "Overcurrent error";
    case 0x3210: return "Overvoltage error";
    case 0x4210: return "Overtemperature error";
    case 0x6320: return "Software parameter error";
    case 0x8110: return "CAN overrun error (object lost)";
    case 0x8120: return "CAN in error-passive mode";
    case 0x8130: return "Life-guard / heartbeat error";
    case 0x8180: return "EtherCAT communication error";
    case 0x8181: return "EtherCAT initialization error";
    case 0x8182: return "EtherCAT Rx queue overflow";
    case 0x8210: return "PDO length error";
    case 0x8250: return "RPDO timeout (drive missed its cyclic command frame)";
    case 0x8280: return "EtherCAT PDO communication error";
    case 0x8281: return "EtherCAT SDO communication error";
    case 0x8611: return "Following error";
    default:
      return "unmapped code - see EPOS4 Firmware Specification ch.7 (Error Handling)";
  }
}

// Print errors
void Maxon::addErrorToReading(const ErrorType& errorType) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  reading_.addError(errorType);
}

/*
** Print error code
** Reads object 0x603F, stores it into the reading (so getLastFault()/
** getFaults() expose the real code to the application - without this addFault()
** is never called and getLastFault() always returns 0) and logs the decoded
** meaning. See firmware documentation for the full code table.
*/
void Maxon::printErrorCode() {
  uint16_t errorcode = 0;
  bool error_read_success =
      sendSdoRead(OD_INDEX_ERROR_CODE, 0x00, false, errorcode);
  if (error_read_success) {
    {
      std::lock_guard<std::recursive_mutex> lock(readingMutex_);
      reading_.addFault(errorcode);
    }
    MELO_ERROR_STREAM("[maxon_epos_ethercat_sdk:Maxon::printErrorCode] '"
                      << name_ << "' error code 0x" << std::hex << std::setw(4)
                      << std::setfill('0') << errorcode << std::dec << " - "
                      << errorCodeToString(errorcode));
  } else {
    MELO_ERROR_STREAM(
        "[maxon_epos_ethercat_sdk:Maxon::printErrorCode] '"
        << name_ << "' reading error code (0x603F) unsuccessful.")
  }
}

/*
** Consume the fault-edge flag set by updateRead() and perform the deferred SDO
** read of 0x603F. Idempotent: only does work the first time it's called after
** a non-Fault → Fault transition. The compare-exchange ensures concurrent
** callers from different threads can't race the SDO call.
**
** Why this exists: printErrorCode() does an SDO read which blocks the EtherCAT
** mailbox until the slave responds. Running it inline in the RT worker at
** 500 Hz means one fault edge consumes most/all of the 2 ms cycle budget and
** several simultaneous faults cascade into WKC errors. By deferring the SDO
** to the executor thread (50 Hz), the worker stays on its deadline and the
** error code still lands in reading_.lastFault_ within at most one executor
** period — fast enough for monitorFaultTransitions() to log it on the same
** fault-edge dump it already produces.
*/
void Maxon::processPendingFaultLog() {
  bool expected = true;
  if (!faultEdgePending_.compare_exchange_strong(expected, false,
                                                 std::memory_order_acq_rel)) {
    return;
  }
  // Match the SDK's existing locking convention: every other SDO-issuing path
  // (configParam, setDriveStateViaSdo) holds mutex_ across the SDO. The bus
  // also has its own contextMutex_ inside SOEM, so this is belt-and-braces,
  // but diverging from convention would be a foot-gun for future maintenance.
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  printErrorCode();
}

/*
 * Print diagnosis messages
 */
void Maxon::printDiagnosis() {
  uint8_t newestIdx = 0;
  uint8_t newMsgAvailable = 0;
  std::array<uint32_t, 4> diagnosisMsg;
  sendSdoRead(OD_INDEX_DIAGNOSIS, 0x04, false, newMsgAvailable);
  if (newMsgAvailable) {
    sendSdoRead(OD_INDEX_DIAGNOSIS, 0x02, false, newestIdx);
    sendSdoRead(OD_INDEX_DIAGNOSIS, newestIdx, false, diagnosisMsg);
    MELO_INFO(
        "[maxon_epos_ethercat_sdk:Maxon::printDiagnosis] Latest diagnostic "
        "message: ");
    for (const auto& s : diagnosisMsg) {
      MELO_INFO_STREAM(std::hex << s);
    }
  } else {
    MELO_INFO(
        "[maxon_epos_ethercat_sdk:Maxon::printDiagnosis] "
        "No diagnostic message available.");
  }
}
}  // namespace maxon
