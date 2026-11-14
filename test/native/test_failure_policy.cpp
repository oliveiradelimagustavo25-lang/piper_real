#include <iostream>
#include <stdexcept>
#include <my_robot_interface/failure_policy.hpp>

using namespace my_robot_interface::failure;

void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}

int main()
{
  Failure f;
  require(f.code == Failure::UNCLASSIFIED, "Default failure must be unclassified");
  require(decideBatch(f) != Decision::SkipTarget, "Missing facts must not permit skipping");
  f.code = Failure::INPUT_INVALID;
  f.origin = Failure::PERCEPTION;
  require(decideStep(f) == Decision::Continue, "Independent perception must not require arm feedback");
  f.origin = Failure::EXECUTOR;
  f.target_specific = true;
  f.command_state = Failure::NOT_SENT;
  f.payload_state = Failure::CLEAR;
  f.scene_state = Failure::CONSISTENT;
  require(decideBatch(f) == Decision::SkipTarget, "Isolated invalid target should not end batch");
  f.code = Failure::PLAN_FAILED;
  f.command_state = Failure::TERMINAL;
  require(decideBatch(f) == Decision::SkipTarget, "Past completed motion must not block skipping");
  for (const auto command : {Failure::UNKNOWN, Failure::IN_FLIGHT, uint8_t(99)}) {
    f.command_state = command;
    require(decideBatch(f) == Decision::Recover, "Unresolved command must not be retried or skipped");
  }
  f.command_state = Failure::NOT_SENT;
  for (const auto payload : {Failure::UNKNOWN, Failure::MAY_HOLD}) {
    f.payload_state = payload;
    require(decideBatch(f) == Decision::Recover, "Unresolved payload must not be skipped");
  }
  f.payload_state = Failure::CLEAR;
  f.scene_state = Failure::UNCERTAIN;
  require(decideBatch(f) == Decision::Recover, "Released payload does not resolve scene uncertainty");
  f.scene_state = Failure::CONSISTENT;
  f.target_specific = false;
  f.code = Failure::DEPENDENCY_UNAVAILABLE;
  const auto budget = std::chrono::seconds(1);
  require(decideStep(f, budget) == Decision::Wait, "Transient dependency should allow bounded waiting");
  require(decideBatch(f) == Decision::EndBatch, "Global dependency must not become target skip");
  f.code = Failure::PLAN_FAILED;
  require(decideStep(f, budget) == Decision::EndGoal, "Time left must not automatically replay motion");
  for (const auto code : {Failure::EXECUTION_RESULT_UNKNOWN, Failure::FEEDBACK_STALE}) {
    f.code = code;
    require(decideStep(f, budget) == Decision::Recover, "Stale or unknown execution needs reconciliation");
  }
  f.code = 60000;
  require(decideBatch(f) == Decision::EndBatch, "Unknown code must not become success");
  f.code = Failure::RECOVERY_FAILED;
  require(decideBatch(f) == Decision::EndBatch, "Recovery must not recursively retry");
  std::cout << "Failure policy checks passed\n";
}
