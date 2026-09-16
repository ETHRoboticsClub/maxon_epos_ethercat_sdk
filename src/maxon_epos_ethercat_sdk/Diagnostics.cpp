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
** Read error code
** Reads object 0x603F into `code` and stores it into the reading (so
** getLastFault()/getFaults() expose the real code to the application - without
** this addFault() is never called and getLastFault() always returns 0).
** Returns false when the SDO read fails; `code` is then untouched. Does not
** log, so the caller decides the severity (see printErrorCode() and the
** boot-fault scan in standalone.cpp).
*/
bool Maxon::readErrorCode(uint16_t& code) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  uint16_t errorcode = 0;
  if (!sendSdoRead(OD_INDEX_ERROR_CODE, 0x00, false, errorcode)) {
    return false;
  }
  {
    std::lock_guard<std::recursive_mutex> readingLock(readingMutex_);
    reading_.addFault(errorcode);
  }
  code = errorcode;
  return true;
}

/*
** Print error code
** readErrorCode() plus a log line with the decoded meaning. See firmware
** documentation for the full code table.
*/
void Maxon::printErrorCode() {
  uint16_t errorcode = 0;
  if (readErrorCode(errorcode)) {
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

void Maxon::processPendingFaultLog() {
  using soem_interface_rsl::MailboxStatus;
  if (faultCodeRequest_) {
    const auto status = faultCodeRequest_->status.load(std::memory_order_acquire);
    if (status == MailboxStatus::Pending) return;
    if (status == MailboxStatus::Success) {
      const auto code = static_cast<uint16_t>(faultCodeRequest_->value);
      {
        std::lock_guard<std::recursive_mutex> lock(readingMutex_);
        reading_.addFault(code);
      }
      MELO_ERROR_STREAM("[" << name_ << "] resolved drive fault 0x603F=0x" << std::hex << code << std::dec << " - " << errorCodeToString(code))
    } else {
      MELO_ERROR_STREAM("[" << name_ << "] fault code unavailable: mailbox status=" << soem_interface_rsl::mailboxStatusName(status)
                        << " abort=" << faultCodeRequest_->abortCode)
    }
    faultCodeRequest_.reset();
  }
  if (faultEdgePending_.exchange(false, std::memory_order_acq_rel)) {
    faultCodeRequest_ = requestSdo(OD_INDEX_ERROR_CODE, 0, 2);
  }
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
