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

#ifndef MY_VEHICLE_INTERFACE__SAFETY_POLICY_HPP_
#define MY_VEHICLE_INTERFACE__SAFETY_POLICY_HPP_

#include <cstdint>

namespace my_vehicle_interface {

enum class SafetyStopReason : uint8_t {
  kNone = 0,
  kEmergency = 1,
  kEmergencyStateTimeout = 2,
  kMissingCommand = 3,
  kCommandTimeout = 4,
};

struct SafetyDecision {
  bool stop_required;
  SafetyStopReason reason;
};

inline SafetyDecision evaluateSafetyStop(bool emergency_stop_active,
                                         bool emergency_state_fresh,
                                         bool has_control_command,
                                         double command_age_sec,
                                         double command_timeout_sec) {
  if (!emergency_state_fresh) {
    return {true, SafetyStopReason::kEmergencyStateTimeout};
  }
  if (emergency_stop_active) {
    return {true, SafetyStopReason::kEmergency};
  }
  if (!has_control_command) {
    return {true, SafetyStopReason::kMissingCommand};
  }
  if (command_age_sec > command_timeout_sec) {
    return {true, SafetyStopReason::kCommandTimeout};
  }
  return {false, SafetyStopReason::kNone};
}

inline const char *safetyStopReasonName(SafetyStopReason reason) {
  switch (reason) {
  case SafetyStopReason::kEmergency:
    return "emergency";
  case SafetyStopReason::kEmergencyStateTimeout:
    return "mission_emergency_state_timeout";
  case SafetyStopReason::kMissingCommand:
    return "missing_control_command";
  case SafetyStopReason::kCommandTimeout:
    return "control_command_timeout";
  case SafetyStopReason::kNone:
  default:
    return "none";
  }
}

}  // namespace my_vehicle_interface

#endif  // MY_VEHICLE_INTERFACE__SAFETY_POLICY_HPP_
