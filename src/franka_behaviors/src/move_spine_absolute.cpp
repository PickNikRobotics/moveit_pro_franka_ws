// Copyright 2026 PickNik Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the PickNik Inc. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <franka_behaviors/move_spine_absolute.hpp>

#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/impl/action_client_behavior_base_impl.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

#include <string>

namespace
{
constexpr auto kPortActionName = "action_name";
constexpr auto kPortPosition = "position";
constexpr auto kPortVelocity = "velocity";
constexpr auto kPortAcceleration = "acceleration";
constexpr auto kPortDeceleration = "deceleration";
constexpr auto kPortTimeout = "timeout";

constexpr auto kDescriptionMoveSpineAbsolute = R"(
  <p>Moves the real Franka spine to an absolute height through franka_spine_server's <code>move_absolute</code>
  action, with the given velocity, acceleration and deceleration. The arms do not move.</p>
  <p>Switch the spine on first with <code>SwitchOnSpine</code>. The spine rejects a height outside its travel, and the
  Behavior fails with the spine's own error text.</p>
)";

// Explains a failed result with the spine's error text and how the motion stopped.
std::string resultDetail(const franka_behaviors::MoveAbsolute::Result& result)
{
  std::string detail;
  if (!result.error.empty())
  {
    detail += ": " + result.error;
  }
  if (!result.stop_by.empty())
  {
    detail += " (stopped by " + result.stop_by + ")";
  }
  return detail.empty() ? "." : detail;
}
}  // namespace

namespace franka_behaviors
{
MoveSpineAbsolute::MoveSpineAbsolute(const std::string& name, const BT::NodeConfiguration& config,
                                     const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::ActionClientBehaviorBase<MoveAbsolute>(name, config, shared_resources)
{
}

BT::PortsList MoveSpineAbsolute::providedPorts()
{
  return {
    BT::InputPort<std::string>(kPortActionName, "/franka_spine_node/move_absolute",
                               "The franka_spine_server move_absolute action."),
    BT::InputPort<double>(kPortPosition, "Target spine height (meters)."),
    BT::InputPort<double>(kPortVelocity, 0.05, "Motion velocity (meters per second), greater than zero."),
    BT::InputPort<double>(kPortAcceleration, 0.1, "Motion acceleration (meters per second squared), greater than zero."),
    BT::InputPort<double>(kPortDeceleration, 0.1, "Motion deceleration (meters per second squared), greater than zero."),
    BT::InputPort<double>(kPortTimeout, 60.0, "Seconds to wait for the move to finish before the Behavior fails."),
  };
}

BT::KeyValueVector MoveSpineAbsolute::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Motion - Execute" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionMoveSpineAbsolute } };
}

tl::expected<std::string, std::string> MoveSpineAbsolute::getActionName()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(getInput<std::string>(kPortActionName));
  if (!ports.has_value())
  {
    return tl::make_unexpected("Failed to get required value from input data port: " + ports.error());
  }
  const auto& [action_name] = ports.value();
  if (action_name.empty())
  {
    return tl::make_unexpected(std::string("The '") + kPortActionName + "' port must not be empty.");
  }
  return action_name;
}

tl::expected<std::chrono::duration<double>, std::string> MoveSpineAbsolute::getResultTimeout()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(getInput<double>(kPortTimeout));
  if (!ports.has_value())
  {
    return tl::make_unexpected("Failed to get required value from input data port: " + ports.error());
  }
  const auto& [timeout] = ports.value();
  return std::chrono::duration<double>{ timeout };
}

tl::expected<MoveAbsolute::Goal, std::string> MoveSpineAbsolute::createGoal()
{
  const auto ports =
      moveit_pro::behaviors::getRequiredInputs(getInput<double>(kPortPosition), getInput<double>(kPortVelocity),
                                               getInput<double>(kPortAcceleration),
                                               getInput<double>(kPortDeceleration));
  if (!ports.has_value())
  {
    return tl::make_unexpected("Failed to get required values from input data ports: " + ports.error());
  }
  const auto& [position, velocity, acceleration, deceleration] = ports.value();
  // The action definition requires these three to be greater than zero.
  for (const auto& [port, value] : { std::pair{ kPortVelocity, velocity }, std::pair{ kPortAcceleration, acceleration },
                                     std::pair{ kPortDeceleration, deceleration } })
  {
    if (!(value > 0.0))
    {
      return tl::make_unexpected(std::string("The '") + port + "' port must be greater than zero.");
    }
  }

  MoveAbsolute::Goal goal;
  goal.position = position;
  goal.velocity = velocity;
  goal.acceleration = acceleration;
  goal.deceleration = deceleration;
  return goal;
}

tl::expected<bool, std::string> MoveSpineAbsolute::processResult(const std::shared_ptr<MoveAbsolute::Result> result)
{
  if (result && !result->success)
  {
    return tl::make_unexpected("The spine did not reach the target height" + resultDetail(*result));
  }
  return true;
}

std::string MoveSpineAbsolute::getAbortedMessage(const std::shared_ptr<const MoveAbsolute::Result> result) const
{
  return "The spine move failed" + (result ? resultDetail(*result) : std::string("."));
}
}  // namespace franka_behaviors

template class moveit_pro::behaviors::ActionClientBehaviorBase<franka_behaviors::MoveAbsolute>;
