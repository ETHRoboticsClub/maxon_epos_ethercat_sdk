#pragma once

#include <cstdint>
#include <optional>

namespace maxon {

// Controller objects read back from an EPOS4 at startup, in the firmware's own units
// (EPOS4 Firmware Specification rel8234, Nov 2018). An empty value is an object whose
// read failed or was skipped; nothing is defaulted. Gains and observer terms are on
// the motor side.
struct ControllerReadback {
  std::optional<uint32_t> controlStructure;            // 0x3000:02 bits, 6.2.49.2, Table 6-95
  std::optional<uint32_t> nominalCurrentMilliamperes;  // 0x3001:01 mA, 6.2.50.1, Table 6-104
  std::optional<uint32_t> outputCurrentLimitMilliamperes;  // 0x3001:02 mA, 6.2.50.2, Table 6-105
  std::optional<uint32_t> torqueConstantMicronewtonMetresPerAmpere;  // 0x3001:05 uNm/A, 6.2.50.5
  std::optional<uint32_t> currentPGain;  // 0x30A0:01 uV/A, 6.2.58.1
  // 0x30A0:02, 6.2.58.2. rel8234 prints uV/(A s), but its default 3'906'250 against the P
  // default 1'171'880 uV/A then gives an integral time of 0.3 s, three orders above a
  // winding's L/R; the unit is uV/(A ms) (Ti = 0.3 ms), as the SDK's write scaling assumes.
  std::optional<uint32_t> currentIGain;
  std::optional<uint32_t> velocityPGain;                        // 0x30A2:01 uA s/rad, 6.2.60.1
  std::optional<uint32_t> velocityIGain;                        // 0x30A2:02 uA/rad, 6.2.60.2
  std::optional<uint32_t> velocityFeedforwardVelocityGain;      // 0x30A2:03 uA s/rad, 6.2.60.3
  std::optional<uint32_t> velocityFeedforwardAccelerationGain;  // 0x30A2:04 uA s^2/rad, 6.2.60.4
  std::optional<uint32_t> observerPositionCorrectionGain;  // 0x30A3:01 per mille, 6.2.61.1
  std::optional<uint32_t> observerVelocityCorrectionGain;  // 0x30A3:02 mHz, 6.2.61.2
  std::optional<uint32_t> observerLoadCorrectionGain;      // 0x30A3:03 uNm/rad, 6.2.61.3
  std::optional<uint32_t> observerFriction;                // 0x30A3:04 0.001 uNm/rpm, 6.2.61.4
  std::optional<uint32_t> observerInertia;                 // 0x30A3:05 0.001 g cm^2, 6.2.61.5
};

}  // namespace maxon
