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

#include <franka_behaviors/error_recovery_action.hpp>

#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/impl/action_client_behavior_base_impl.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

#include <string>

namespace
{
constexpr auto kPortActionName = "action_name";
constexpr auto kPortTimeout = "timeout";

constexpr auto kDescriptionErrorRecovery = R"(
  <p>Clears a reflex or error on a Franka arm, for example after a collision or a released user stop. It calls the
  franka_ros2 <code>error_recovery</code> action, which runs the libfranka automatic error recovery.</p>
  <p>It does not restore control: the fault also stops the arm's ros2_control hardware component and its
  controllers. Follow it with <code>SetHardwareComponentState</code> to <code>active</code> and
  <code>SwitchController</code>, as the "Recover Arm" Objective does.</p>
)";
}  // namespace

namespace franka_behaviors
{
ErrorRecoveryAction::ErrorRecoveryAction(const std::string& name, const BT::NodeConfiguration& config,
                                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::ActionClientBehaviorBase<ErrorRecovery>(name, config, shared_resources)
{
}

BT::PortsList ErrorRecoveryAction::providedPorts()
{
  return {
    BT::InputPort<std::string>(kPortActionName, "/action_server/error_recovery",
                               "The franka_ros2 error_recovery action. An arm with a prefix has it in a namespace."),
    BT::InputPort<double>(kPortTimeout, 10.0, "Seconds to wait for the recovery result before the Behavior fails."),
  };
}

BT::KeyValueVector ErrorRecoveryAction::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Franka" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionErrorRecovery } };
}

tl::expected<std::string, std::string> ErrorRecoveryAction::getActionName()
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

tl::expected<std::chrono::duration<double>, std::string> ErrorRecoveryAction::getResultTimeout()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(getInput<double>(kPortTimeout));
  if (!ports.has_value())
  {
    return tl::make_unexpected("Failed to get required value from input data port: " + ports.error());
  }
  const auto& [timeout] = ports.value();
  return std::chrono::duration<double>{ timeout };
}

tl::expected<ErrorRecovery::Goal, std::string> ErrorRecoveryAction::createGoal()
{
  return ErrorRecovery::Goal{};
}
}  // namespace franka_behaviors

template class moveit_pro::behaviors::ActionClientBehaviorBase<franka_behaviors::ErrorRecovery>;
