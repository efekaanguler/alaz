// Copyright 2026 Alaz Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include "my_vehicle_interface/safety_policy.hpp"

namespace my_vehicle_interface {

TEST(SafetyPolicy, EmergencyAlwaysRequiresStop) {
  const auto decision = evaluateSafetyStop(true, true, true, 0.0, 0.2);
  EXPECT_TRUE(decision.stop_required);
  EXPECT_EQ(decision.reason, SafetyStopReason::kEmergency);
}

TEST(SafetyPolicy, MissingCommandRequiresStop) {
  const auto decision = evaluateSafetyStop(false, true, false, 0.0, 0.2);
  EXPECT_TRUE(decision.stop_required);
  EXPECT_EQ(decision.reason, SafetyStopReason::kMissingCommand);
}

TEST(SafetyPolicy, StaleCommandRequiresStop) {
  const auto decision = evaluateSafetyStop(false, true, true, 0.21, 0.2);
  EXPECT_TRUE(decision.stop_required);
  EXPECT_EQ(decision.reason, SafetyStopReason::kCommandTimeout);
}

TEST(SafetyPolicy, FreshCommandClearsStop) {
  const auto decision = evaluateSafetyStop(false, true, true, 0.1, 0.2);
  EXPECT_FALSE(decision.stop_required);
  EXPECT_EQ(decision.reason, SafetyStopReason::kNone);
}

TEST(SafetyPolicy, TimeoutBoundaryIsStillFresh) {
  const auto decision = evaluateSafetyStop(false, true, true, 0.2, 0.2);
  EXPECT_FALSE(decision.stop_required);
}

TEST(SafetyPolicy, MissingMissionHeartbeatRequiresStop) {
  const auto decision = evaluateSafetyStop(false, false, true, 0.0, 0.2);
  EXPECT_TRUE(decision.stop_required);
  EXPECT_EQ(decision.reason, SafetyStopReason::kEmergencyStateTimeout);
}

}  // namespace my_vehicle_interface
