#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "maxon_epos_ethercat_sdk/Maxon.hpp"

namespace maxon {
namespace {

constexpr uint16_t kSwitchOnDisabled = 1u << 6;
constexpr uint16_t kHomingAttained = 1u << 12;
constexpr uint16_t kHomingError = 1u << 13;
constexpr uint16_t kPositionReferenced = 1u << 15;

class ProtocolMaxon final : public Maxon {
 public:
  ProtocolMaxon() {
    configuration_.modesOfOperation = {
        ModeOfOperationEnum::HomingMode,
        ModeOfOperationEnum::CyclicJVPTMode,
    };
    configuration_.positionEncoderResolution = 1000;
    configuration_.nominalCurrentA = 1.0;
    configuration_.torqueConstantNmA = 1.0;
    configuration_.jvptPGain = 120;
    configuration_.jvptDGain = 34;
    reading_.setStatusword(kSwitchOnDisabled);
    reading_.setPositionFactorIntegerToRad(0.001);
    reading_.setActualJointPositionRAW(420);
    reading_.setTimePointNow();
  }

  bool displayedModeAccepted{true};
  bool completeOnStart{true};
  bool freshCompletion{true};
  bool includeReferencedBit{true};
  bool includeHomingError{false};
  bool actualPositionReadOk{true};
  int32_t actualPosition{0};
  bool storeOk{true};
  unsigned storeCalls{0};
  unsigned actualPositionReads{0};
  std::vector<std::pair<uint8_t, uint32_t>> gainWrites;
  std::vector<int8_t> methodWrites;
  std::vector<int32_t> homePositionWrites;
  std::vector<ModeOfOperationEnum> stagedModes;
  std::vector<Command> stagedCommands;
  std::vector<bool> homingStartEdges;
  std::vector<uint16_t> controlwords;

  void setStatusword(uint16_t statusword) {
    reading_.setStatusword(statusword);
    reading_.setTimePointNow();
  }

 protected:
  bool persistentZeroReadSerial(uint32_t& serial) override {
    serial = 0x12345678;
    return true;
  }

  bool persistentZeroVerifyMethod(int8_t method) override {
    methodWrites.push_back(method);
    return true;
  }

  bool persistentZeroVerifyHomePosition(int32_t position) override {
    homePositionWrites.push_back(position);
    return true;
  }

  bool persistentZeroReadDisplayedMode(int8_t& mode) override {
    mode = static_cast<int8_t>(displayedModeAccepted
        ? ModeOfOperationEnum::HomingMode
        : ModeOfOperationEnum::CyclicJVPTMode);
    return true;
  }

  bool persistentZeroReadActualPosition(int32_t& position) override {
    ++actualPositionReads;
    position = actualPosition;
    return actualPositionReadOk;
  }

  bool persistentZeroVerifyJvptGain(uint8_t subindex, uint32_t value) override {
    gainWrites.emplace_back(subindex, value);
    return true;
  }

  bool persistentZeroStoreParameters() override {
    ++storeCalls;
    return storeOk;
  }

  Reading persistentZeroReading() const override { return reading_; }

  void persistentZeroStageCommand(const Command& command) override {
    stagedModes.push_back(command.getModeOfOperation());
    stagedCommands.push_back(command);
    Maxon::persistentZeroStageCommand(command);
  }

  void persistentZeroSetHomingStart(bool start) override {
    Maxon::persistentZeroSetHomingStart(start);
    homingStartEdges.push_back(start);
    controlwords.push_back(controlword_.getRawControlword());
    if (!start || !completeOnStart) return;
    uint16_t status = kSwitchOnDisabled | kHomingAttained;
    if (includeReferencedBit) status |= kPositionReferenced;
    if (includeHomingError) status |= kHomingError;
    reading_.setStatusword(status);
    if (freshCompletion) reading_.setTimePointNow();
  }

  void persistentZeroSleepFor(std::chrono::milliseconds) override {}

 private:
  mutable Reading reading_;
};

TEST(PersistentZeroProtocol, ReferencesAtZeroWithoutEnablingTorque) {
  ProtocolMaxon drive;

  const auto result = drive.referenceCurrentPositionAsZero();

  EXPECT_EQ(result.reference, Maxon::PersistentZeroResult::Reference::Applied);
  EXPECT_EQ(result.persistence,
            Maxon::PersistentZeroResult::Persistence::NotAttempted);
  EXPECT_EQ(drive.methodWrites, std::vector<int8_t>({37}));
  EXPECT_EQ(drive.homePositionWrites, std::vector<int32_t>({0}));
  EXPECT_EQ(drive.stagedModes,
            std::vector<ModeOfOperationEnum>({ModeOfOperationEnum::HomingMode}));
  EXPECT_EQ(drive.homingStartEdges, std::vector<bool>({false, true, false}));
  EXPECT_EQ(drive.actualPositionReads, 1u);
  for (const auto controlword : drive.controlwords) {
    EXPECT_EQ(controlword & (1u << 3), 0u) << "Method 37 must not enable operation";
  }
}

TEST(PersistentZeroProtocol, RejectsCompletionWithoutANewerPdoSample) {
  ProtocolMaxon drive;
  drive.freshCompletion = false;

  const auto result = drive.referenceCurrentPositionAsZero();

  EXPECT_EQ(result.reference, Maxon::PersistentZeroResult::Reference::Unknown);
  EXPECT_EQ(drive.actualPositionReads, 0u);
  EXPECT_EQ(drive.homingStartEdges.back(), false);
}

TEST(PersistentZeroProtocol, RejectsCompletionWithoutReferencedBit) {
  ProtocolMaxon drive;
  drive.includeReferencedBit = false;

  const auto result = drive.referenceCurrentPositionAsZero();

  EXPECT_EQ(result.reference, Maxon::PersistentZeroResult::Reference::Unknown);
  EXPECT_EQ(drive.actualPositionReads, 0u);
}

TEST(PersistentZeroProtocol, RejectsHomingErrorEvenWithCompletionBits) {
  ProtocolMaxon drive;
  drive.includeHomingError = true;

  const auto result = drive.referenceCurrentPositionAsZero();

  EXPECT_EQ(result.reference, Maxon::PersistentZeroResult::Reference::Unknown);
  EXPECT_EQ(drive.actualPositionReads, 0u);
  EXPECT_EQ(drive.homingStartEdges.back(), false);
}

TEST(PersistentZeroProtocol, RejectsNonzeroPositionReadback) {
  ProtocolMaxon drive;
  drive.actualPosition = 1;

  const auto result = drive.referenceCurrentPositionAsZero();

  EXPECT_EQ(result.reference, Maxon::PersistentZeroResult::Reference::Unknown);
  EXPECT_EQ(drive.actualPositionReads, 1u);
}

TEST(PersistentZeroProtocol, CancellationBeforeMutationPerformsNoWrites) {
  ProtocolMaxon drive;

  const auto result = drive.referenceCurrentPositionAsZero([] { return true; });

  EXPECT_EQ(result.reference, Maxon::PersistentZeroResult::Reference::Unchanged);
  EXPECT_TRUE(drive.methodWrites.empty());
  EXPECT_TRUE(drive.homePositionWrites.empty());
  EXPECT_TRUE(drive.stagedModes.empty());
  EXPECT_TRUE(drive.homingStartEdges.empty());
}

TEST(PersistentZeroProtocol, FailedModeConfirmationRestoresSafeDisabledTarget) {
  ProtocolMaxon drive;
  drive.displayedModeAccepted = false;

  const auto result = drive.referenceCurrentPositionAsZero();

  ASSERT_EQ(result.reference, Maxon::PersistentZeroResult::Reference::Unchanged);
  ASSERT_EQ(drive.stagedModes.size(), 2u);
  EXPECT_EQ(drive.stagedModes[0], ModeOfOperationEnum::HomingMode);
  EXPECT_EQ(drive.stagedModes[1], ModeOfOperationEnum::CyclicJVPTMode);
  EXPECT_DOUBLE_EQ(drive.stagedCommands[1].getTargetJointPosition(), 0.42);
  EXPECT_DOUBLE_EQ(drive.stagedCommands[1].getTargetJointVelocity(), 0.0);
  EXPECT_DOUBLE_EQ(drive.stagedCommands[1].getTargetJointTorque(), 0.0);
  EXPECT_TRUE(drive.homingStartEdges.empty());
}

TEST(PersistentZeroProtocol, StaleAttainedBitRestoresSafeDisabledTarget) {
  ProtocolMaxon drive;
  drive.setStatusword(kSwitchOnDisabled | kHomingAttained | kPositionReferenced);

  const auto result = drive.referenceCurrentPositionAsZero();

  ASSERT_EQ(result.reference, Maxon::PersistentZeroResult::Reference::Unchanged);
  ASSERT_EQ(drive.stagedModes.size(), 2u);
  EXPECT_EQ(drive.stagedModes[0], ModeOfOperationEnum::HomingMode);
  EXPECT_EQ(drive.stagedModes[1], ModeOfOperationEnum::CyclicJVPTMode);
  EXPECT_EQ(drive.homingStartEdges, std::vector<bool>({false}));
}

TEST(PersistentZeroProtocol, CancellationAfterPreparationDoesNotTriggerHoming) {
  ProtocolMaxon drive;
  unsigned cancellationChecks = 0;

  const auto result = drive.referenceCurrentPositionAsZero(
      [&cancellationChecks] { return ++cancellationChecks >= 6; });

  ASSERT_EQ(result.reference, Maxon::PersistentZeroResult::Reference::Unchanged);
  EXPECT_EQ(drive.methodWrites, std::vector<int8_t>({37}));
  EXPECT_EQ(drive.homePositionWrites, std::vector<int32_t>({0}));
  EXPECT_EQ(drive.stagedModes,
            (std::vector<ModeOfOperationEnum>{ModeOfOperationEnum::HomingMode,
                                              ModeOfOperationEnum::CyclicJVPTMode}));
  EXPECT_EQ(drive.homingStartEdges, std::vector<bool>({false}));
}

TEST(PersistentZeroProtocol, RestoresBaselineBeforeOneSaveAttempt) {
  ProtocolMaxon drive;

  const auto result = drive.persistReferencedZero();

  EXPECT_EQ(result.persistence,
            Maxon::PersistentZeroResult::Persistence::Persisted);
  EXPECT_EQ(drive.gainWrites,
            (std::vector<std::pair<uint8_t, uint32_t>>{{0x01, 120}, {0x03, 34}}));
  EXPECT_EQ(drive.storeCalls, 1u);
}

TEST(PersistentZeroProtocol, LostSaveAcknowledgementIsUnknownAndNotRetried) {
  ProtocolMaxon drive;
  drive.storeOk = false;

  const auto result = drive.persistReferencedZero();

  EXPECT_EQ(result.persistence,
            Maxon::PersistentZeroResult::Persistence::Unknown);
  EXPECT_EQ(drive.storeCalls, 1u);
}

TEST(PersistentZeroProtocol, PersistenceCancellationPerformsNoWrites) {
  ProtocolMaxon drive;

  const auto result = drive.persistReferencedZero([] { return true; });

  EXPECT_EQ(result.persistence,
            Maxon::PersistentZeroResult::Persistence::NotAttempted);
  EXPECT_TRUE(drive.gainWrites.empty());
  EXPECT_EQ(drive.storeCalls, 0u);
}

TEST(PersistentZeroProtocol, CancellationAfterPBaselineDoesNotWriteDOrStore) {
  ProtocolMaxon drive;
  unsigned cancellationChecks = 0;

  const auto result = drive.persistReferencedZero(
      [&cancellationChecks] { return ++cancellationChecks >= 3; });

  EXPECT_EQ(result.persistence,
            Maxon::PersistentZeroResult::Persistence::NotAttempted);
  EXPECT_EQ(drive.gainWrites,
            (std::vector<std::pair<uint8_t, uint32_t>>{{0x01, 120}}));
  EXPECT_EQ(drive.storeCalls, 0u);
}

TEST(PersistentZeroProtocol, CancellationAfterBaselineDoesNotStore) {
  ProtocolMaxon drive;
  unsigned cancellationChecks = 0;

  const auto result = drive.persistReferencedZero(
      [&cancellationChecks] { return ++cancellationChecks >= 4; });

  EXPECT_EQ(result.persistence,
            Maxon::PersistentZeroResult::Persistence::NotAttempted);
  EXPECT_EQ(drive.gainWrites,
            (std::vector<std::pair<uint8_t, uint32_t>>{{0x01, 120}, {0x03, 34}}));
  EXPECT_EQ(drive.storeCalls, 0u);
}

}  // namespace
}  // namespace maxon
