#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "maxon_epos_ethercat_sdk/Maxon.hpp"

namespace maxon {
namespace {

constexpr uint16_t kSwitchOnDisabled = 1u << 6;
constexpr uint16_t kOperationEnabled = 0x0027;
constexpr uint16_t kQuickStopActive = 0x0007;
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

  bool serialReadOk{true};
  bool displayedModeAccepted{true};
  // False: 0x6061 keeps showing HomingMode once homing ran (the JVPT restore fails).
  bool acceptModesAfterHoming{true};
  bool completeOnStart{true};
  // Statusword state bits the drive reports once Method 37 completes; the
  // default keeps whatever state the test armed the drive in.
  std::optional<uint16_t> stateAfterCompletion;
  bool homeReferenceReadOk{true};
  int32_t homeReference{-1628};
  bool freshCompletion{true};
  bool includeReferencedBit{true};
  bool includeHomingError{false};
  bool actualPositionReadOk{true};
  int32_t actualPosition{0};
  // 0x6064 in the completion cycle relative to the requested Home Position.
  int32_t pdoOffsetAfterHoming{0};
  // 0x30B0 as read back after completion; default: what was written.
  std::optional<int32_t> homePositionAfterHoming;
  bool homePositionReadOk{true};
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
    return serialReadOk;
  }

  bool persistentZeroVerifyMethod(int8_t method) override {
    methodWrites.push_back(method);
    return true;
  }

  bool persistentZeroVerifyHomePosition(int32_t position) override {
    homePositionWrites.push_back(position);
    return true;
  }

  bool persistentZeroReadHomePosition(int32_t& position) override {
    position = homePositionAfterHoming ? *homePositionAfterHoming
        : homePositionWrites.empty() ? 0 : homePositionWrites.back();
    return homePositionReadOk;
  }

  bool persistentZeroReadDisplayedMode(int8_t& mode) override {
    // 0x6061 follows the last staged mode; a refusing drive stays in JVPT, and
    // one that will not leave homing keeps showing HomingMode after the edge.
    if (!acceptModesAfterHoming && !homingStartEdges.empty()) {
      mode = static_cast<int8_t>(ModeOfOperationEnum::HomingMode);
      return true;
    }
    mode = static_cast<int8_t>(displayedModeAccepted && !stagedModes.empty()
        ? stagedModes.back()
        : ModeOfOperationEnum::CyclicJVPTMode);
    return true;
  }

  bool persistentZeroReadHomeReference(int32_t& value) override {
    value = homeReference;
    return homeReferenceReadOk;
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
    if (!homePositionWrites.empty()) {
      reading_.setActualJointPositionRAW(homePositionWrites.back() + pdoOffsetAfterHoming);
    }
    const uint16_t state = stateAfterCompletion ? *stateAfterCompletion
        : static_cast<uint16_t>(reading_.getRawStatusword() & 0x006F);
    uint16_t status = state | kHomingAttained;
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

TEST(PersistentZeroProtocol, ReferencesAnEnabledDriveAndRestoresJvptAtTheReference) {
  ProtocolMaxon drive;
  drive.setStatusword(kOperationEnabled);
  drive.actualPosition = -125;

  const auto result = drive.referenceCurrentPositionAs(-M_PI / 4.0);

  EXPECT_EQ(result.reference, Maxon::PersistentZeroResult::Reference::Applied);
  EXPECT_EQ(result.persistence, Maxon::PersistentZeroResult::Persistence::NotAttempted);
  ASSERT_TRUE(result.homeReference.has_value());
  EXPECT_EQ(*result.homeReference, -1628);
  EXPECT_EQ(drive.homePositionWrites, std::vector<int32_t>({-125}));
  ASSERT_EQ(drive.stagedModes,
            (std::vector<ModeOfOperationEnum>{ModeOfOperationEnum::HomingMode,
                                              ModeOfOperationEnum::CyclicJVPTMode}));
  // The handover target is the reference itself, not a pre-shift sample.
  EXPECT_DOUBLE_EQ(drive.stagedCommands[1].getTargetJointPosition(), -M_PI / 4.0);
  EXPECT_DOUBLE_EQ(drive.stagedCommands[1].getTargetJointVelocity(), 0.0);
  EXPECT_DOUBLE_EQ(drive.stagedCommands[1].getTargetJointTorque(), 0.0);
  EXPECT_EQ(drive.homingStartEdges, std::vector<bool>({false, true, false}));
  EXPECT_EQ(drive.storeCalls, 0u);
}

TEST(PersistentZeroProtocol, EnabledDriveThatDropsOutDuringMethod37IsUnknown) {
  ProtocolMaxon drive;
  drive.setStatusword(kOperationEnabled);
  drive.stateAfterCompletion = kSwitchOnDisabled;

  const auto result = drive.referenceCurrentPositionAsZero();

  EXPECT_EQ(result.reference, Maxon::PersistentZeroResult::Reference::Unknown);
  EXPECT_NE(result.detail.find("left OperationEnabled"), std::string::npos);
  EXPECT_EQ(drive.actualPositionReads, 0u);
  EXPECT_EQ(drive.homingStartEdges.back(), false);
}

TEST(PersistentZeroProtocol, DisabledDriveThatEnablesDuringMethod37IsUnknown) {
  ProtocolMaxon drive;
  drive.stateAfterCompletion = kOperationEnabled;

  const auto result = drive.referenceCurrentPositionAsZero();

  EXPECT_EQ(result.reference, Maxon::PersistentZeroResult::Reference::Unknown);
  EXPECT_NE(result.detail.find("left SwitchOnDisabled"), std::string::npos);
}

TEST(PersistentZeroProtocol, UnreadableSerialIsUnchangedWithoutWrites) {
  // The identity read precedes every mutation: its failure leaves the drive
  // exactly as found, and must not be reported as an unknown reference.
  ProtocolMaxon maxon;
  maxon.serialReadOk = false;
  const auto result = maxon.referenceCurrentPositionAsZero();
  EXPECT_EQ(result.reference, Maxon::PersistentZeroResult::Reference::Unchanged);
  EXPECT_EQ(result.persistence, Maxon::PersistentZeroResult::Persistence::NotAttempted);
  EXPECT_NE(result.detail.find("0x1018:04"), std::string::npos);
  EXPECT_TRUE(maxon.methodWrites.empty());
  EXPECT_TRUE(maxon.homePositionWrites.empty());
  EXPECT_TRUE(maxon.stagedModes.empty());
  EXPECT_TRUE(maxon.homingStartEdges.empty());
  const auto persisted = maxon.persistReferencedZero();
  EXPECT_EQ(persisted.persistence, Maxon::PersistentZeroResult::Persistence::NotAttempted);
  EXPECT_TRUE(maxon.gainWrites.empty());
  EXPECT_EQ(maxon.storeCalls, 0u);
}

TEST(PersistentZeroProtocol, OtherDriveStatesAreRefusedWithoutWrites) {
  ProtocolMaxon drive;
  drive.setStatusword(kQuickStopActive);

  const auto result = drive.referenceCurrentPositionAsZero();

  EXPECT_EQ(result.reference, Maxon::PersistentZeroResult::Reference::Unchanged);
  EXPECT_TRUE(drive.methodWrites.empty());
  EXPECT_TRUE(drive.stagedModes.empty());
}

TEST(PersistentZeroProtocol, MissingHomeReferenceReadbackKeepsTheAppliedResult) {
  ProtocolMaxon drive;
  drive.homeReferenceReadOk = false;

  const auto result = drive.referenceCurrentPositionAsZero();

  EXPECT_EQ(result.reference, Maxon::PersistentZeroResult::Reference::Applied);
  EXPECT_FALSE(result.homeReference.has_value());
}

TEST(PersistentZeroProtocol, RejectsCompletionWithoutANewerPdoSample) {
  ProtocolMaxon drive;
  drive.freshCompletion = false;

  const auto result = drive.referenceCurrentPositionAsZero();

  EXPECT_EQ(result.reference, Maxon::PersistentZeroResult::Reference::Unknown);
  EXPECT_EQ(drive.actualPositionReads, 0u);
  EXPECT_EQ(drive.homingStartEdges.back(), false);
}

TEST(PersistentZeroProtocol, ReferencesAtAnExplicitSignedPosition) {
  ProtocolMaxon drive;
  drive.actualPosition = -125;
  const auto result = drive.referenceCurrentPositionAs(-M_PI / 4.0);
  EXPECT_EQ(result.reference, Maxon::PersistentZeroResult::Reference::Applied);
  EXPECT_EQ(drive.homePositionWrites, std::vector<int32_t>({-125}));
  EXPECT_EQ(drive.actualPositionReads, 1u);
  for (const auto word : drive.controlwords) EXPECT_EQ(word & (1u << 3), 0u);
}

TEST(PersistentZeroProtocol, ExplicitReferenceIsProvenByTheRegisterNotTheReadback) {
  // 0x30B0 holds the requested 250 counts and the edge was fresh: the frame is
  // set. The 40 counts seen at completion are motion after the latch.
  ProtocolMaxon drive;
  drive.pdoOffsetAfterHoming = 40;
  const auto result = drive.referenceCurrentPositionAs(M_PI / 2.0);
  EXPECT_EQ(drive.homePositionWrites, std::vector<int32_t>({250}));
  EXPECT_EQ(result.reference, Maxon::PersistentZeroResult::Reference::Applied);
  EXPECT_EQ(result.motionAfterLatchCounts, 40);
  EXPECT_EQ(drive.storeCalls, 0u);
}

TEST(PersistentZeroProtocol, InvalidReferencePerformsNoMutation) {
  for (double position : {std::numeric_limits<double>::quiet_NaN(),
                          std::numeric_limits<double>::infinity(), 1e20}) {
    ProtocolMaxon drive;
    const auto result = drive.referenceCurrentPositionAs(position);
    EXPECT_EQ(result.reference, Maxon::PersistentZeroResult::Reference::Unchanged);
    EXPECT_TRUE(drive.methodWrites.empty());
    EXPECT_TRUE(drive.homePositionWrites.empty());
    EXPECT_TRUE(drive.homingStartEdges.empty());
  }
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

TEST(PersistentZeroProtocol, MotionAfterTheLatchIsReportedNotJudged) {
  // Method 37 latches the frame at the homing instant; whatever 0x6064 shows
  // in the completion cycle or at the readback is motion since (a loaded joint
  // handed to the homing-mode position loop is pulled toward its demand). The
  // reference is applied and both deltas are reported in counts.
  for (const int32_t moved : {-30, -2, -1, 1, 2, 29}) {
    ProtocolMaxon drive;
    drive.pdoOffsetAfterHoming = moved;
    drive.actualPosition = 10 * moved;  // the SDO readback comes later
    const auto result = drive.referenceCurrentPositionAsZero();
    EXPECT_EQ(result.reference, Maxon::PersistentZeroResult::Reference::Applied) << moved;
    EXPECT_EQ(result.motionAfterLatchCounts, moved) << moved;
    EXPECT_EQ(result.motionAtReadbackCounts, 10 * moved) << moved;
    EXPECT_TRUE(result.frameSet);
  }
  ProtocolMaxon still;
  const auto quiet = still.referenceCurrentPositionAsZero();
  EXPECT_EQ(quiet.motionAfterLatchCounts, 0);
  EXPECT_EQ(quiet.motionAtReadbackCounts, 0);
}

TEST(PersistentZeroProtocol, FrameSetIsReportedIndependentlyOfTheReferenceVerdict) {
  // After the frame is proven, the only Unknown verdicts left are the mode
  // restore and the drive-state check on an enabled drive: frameSet stays true.
  ProtocolMaxon restoreFails;
  restoreFails.setStatusword(kOperationEnabled);
  restoreFails.acceptModesAfterHoming = false;
  const auto noJvpt = restoreFails.referenceCurrentPositionAsZero();
  EXPECT_EQ(noJvpt.reference, Maxon::PersistentZeroResult::Reference::Unknown);
  EXPECT_NE(noJvpt.detail.find("CyclicJVPTMode"), std::string::npos) << noJvpt.detail;
  EXPECT_TRUE(noJvpt.frameSet);
  ProtocolMaxon fine;
  EXPECT_TRUE(fine.referenceCurrentPositionAsZero().frameSet);
  ProtocolMaxon serialLess;
  serialLess.serialReadOk = false;
  EXPECT_FALSE(serialLess.referenceCurrentPositionAsZero().frameSet);
  ProtocolMaxon register7;
  register7.homePositionAfterHoming = 7;
  EXPECT_FALSE(register7.referenceCurrentPositionAsZero().frameSet);
  ProtocolMaxon erroring;
  erroring.includeHomingError = true;
  EXPECT_FALSE(erroring.referenceCurrentPositionAsZero().frameSet);
  ProtocolMaxon modeless;
  modeless.setStatusword(kOperationEnabled);
  modeless.displayedModeAccepted = false;
  const auto noHoming = modeless.referenceCurrentPositionAsZero();
  EXPECT_EQ(noHoming.reference, Maxon::PersistentZeroResult::Reference::Unchanged);
  EXPECT_FALSE(noHoming.frameSet);
}

TEST(PersistentZeroProtocol, HomePositionRegisterMustStillHoldTheRequestedValue) {
  ProtocolMaxon changed;
  changed.homePositionAfterHoming = 7;
  const auto result = changed.referenceCurrentPositionAs(-M_PI / 4.0);
  EXPECT_EQ(result.reference, Maxon::PersistentZeroResult::Reference::Unknown);
  EXPECT_NE(result.detail.find("0x30B0) reads 7 against requested -125"), std::string::npos) << result.detail;
  ProtocolMaxon unreadable;
  unreadable.homePositionReadOk = false;
  EXPECT_EQ(unreadable.referenceCurrentPositionAsZero().reference, Maxon::PersistentZeroResult::Reference::Unknown);
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
