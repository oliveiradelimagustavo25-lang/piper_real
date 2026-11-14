#pragma once

#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <my_robot_interface/msg/failure.hpp>

namespace my_robot_interface::failure
{
using Failure = my_robot_interface::msg::Failure;
enum class Decision {Continue, Wait, SkipTarget, Recover, EndGoal, EndBatch};

// For APIs that already throw: preserve the native failure without parsing what().
class Error : public std::runtime_error
{
public:
  explicit Error(Failure value)
  : std::runtime_error(value.detail), failure(std::move(value)) {}
  Failure failure;
};

inline Failure makeFailure(uint16_t code, uint8_t origin, const std::string & detail)
{
  Failure value;
  value.code = code;
  value.origin = origin;
  value.detail = detail;
  return value;
}

inline bool targetFailure(const Failure & f)
{
  return f.target_specific &&
         (f.code == Failure::TARGET_UNAVAILABLE || f.code == Failure::INPUT_INVALID ||
         f.code == Failure::PLAN_FAILED || f.code == Failure::OBSERVATION_STALE);
}

inline Decision decideStep(
  const Failure & f, std::chrono::milliseconds remaining_wait = std::chrono::milliseconds{0})
{
  if (f.code == Failure::NONE) {return Decision::Continue;}
  if (f.code == Failure::RECOVERY_FAILED) {return Decision::EndGoal;}
  // Invalid perception records do not require knowledge of the arm's state.
  if (f.origin == Failure::PERCEPTION && f.code == Failure::INPUT_INVALID) {
    return Decision::Continue;
  }
  if ((f.command_state != Failure::NOT_SENT && f.command_state != Failure::TERMINAL) ||
    f.payload_state != Failure::CLEAR || f.scene_state != Failure::CONSISTENT)
  {
    return Decision::Recover;
  }
  switch (f.code) {
    case Failure::OBSERVATION_STALE:
    case Failure::TF_UNAVAILABLE:
    case Failure::DEPENDENCY_UNAVAILABLE:
      return remaining_wait.count() > 0 ? Decision::Wait : Decision::EndGoal;
    case Failure::EXECUTION_RESULT_UNKNOWN:
    case Failure::FEEDBACK_STALE:
    case Failure::SCENE_INCONSISTENT:
    case Failure::RECOVERY_REQUIRED:
      return Decision::Recover;
    default:
      return Decision::EndGoal;
  }
}

inline Decision decideBatch(
  const Failure & f, std::chrono::milliseconds remaining_wait = std::chrono::milliseconds{0})
{
  const auto decision = decideStep(f, remaining_wait);
  if (decision != Decision::EndGoal) {return decision;}
  return targetFailure(f) ? Decision::SkipTarget : Decision::EndBatch;
}
}  // namespace my_robot_interface::failure
