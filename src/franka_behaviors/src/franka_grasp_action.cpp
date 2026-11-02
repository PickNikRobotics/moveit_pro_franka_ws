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

#include <franka_behaviors/franka_grasp_action.hpp>

#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/impl/action_client_behavior_base_impl.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

#include <string>

namespace
{
constexpr auto kPortActionName = "action_name";
constexpr auto kPortWidth = "width";
constexpr auto kPortSpeed = "speed";
constexpr auto kPortForce = "force";
constexpr auto kPortInnerEpsilon = "inner_epsilon";
constexpr auto kPortOuterEpsilon = "outer_epsilon";
constexpr auto kPortTimeout = "timeout";

constexpr auto kDescriptionFrankaGraspAction = R"(
  <p>Grasps an object with the Franka Hand through the franka_gripper <code>grasp</code> action. The fingers close
  at the given speed and hold the object with the given force.</p>
  <p>The grasp succeeds when the final distance between the fingers is within the epsilon range around the target
  width. Otherwise the Behavior fails, for example when no object is between the fingers.</p>
)";
}  // namespace

namespace franka_behaviors
{
FrankaGraspAction::FrankaGraspAction(const std::string& name, const BT::NodeConfiguration& config,
                                     const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::ActionClientBehaviorBase<Grasp>(name, config, shared_resources)
{
}

BT::PortsList FrankaGraspAction::providedPorts()
{
  return {
    BT::InputPort<std::string>(kPortActionName, "/franka_gripper/grasp", "The franka_gripper grasp action."),
    BT::InputPort<double>(kPortWidth, "Target distance between the fingers (meters)."),
    BT::InputPort<double>(kPortSpeed, 0.1, "Closing speed of the fingers (meters per second)."),
    BT::InputPort<double>(kPortForce, "Grasp force (newtons)."),
    BT::InputPort<double>(kPortInnerEpsilon, 0.005, "How much smaller than the width the grasp can be (meters)."),
    BT::InputPort<double>(kPortOuterEpsilon, 0.005, "How much larger than the width the grasp can be (meters)."),
    BT::InputPort<double>(kPortTimeout, 10.0, "Seconds to wait for the grasp result before the Behavior fails."),
  };
}

BT::KeyValueVector FrankaGraspAction::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Grasping" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionFrankaGraspAction } };
}

tl::expected<std::string, std::string> FrankaGraspAction::getActionName()
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

tl::expected<std::chrono::duration<double>, std::string> FrankaGraspAction::getResultTimeout()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(getInput<double>(kPortTimeout));
  if (!ports.has_value())
  {
    return tl::make_unexpected("Failed to get required value from input data port: " + ports.error());
  }
  const auto& [timeout] = ports.value();
  return std::chrono::duration<double>{ timeout };
}

tl::expected<Grasp::Goal, std::string> FrankaGraspAction::createGoal()
{
  const auto ports =
      moveit_pro::behaviors::getRequiredInputs(getInput<double>(kPortWidth), getInput<double>(kPortSpeed),
                                               getInput<double>(kPortForce), getInput<double>(kPortInnerEpsilon),
                                               getInput<double>(kPortOuterEpsilon));
  if (!ports.has_value())
  {
    return tl::make_unexpected("Failed to get required values from input data ports: " + ports.error());
  }
  const auto& [width, speed, force, inner_epsilon, outer_epsilon] = ports.value();

  Grasp::Goal goal;
  goal.width = width;
  goal.speed = speed;
  goal.force = force;
  goal.epsilon.inner = inner_epsilon;
  goal.epsilon.outer = outer_epsilon;
  return goal;
}

std::string FrankaGraspAction::getAbortedMessage(const std::shared_ptr<const Grasp::Result> result) const
{
  const std::string detail = (result && !result->error.empty()) ? ": " + result->error : ".";
  return "The Franka Hand grasp failed" + detail;
}
}  // namespace franka_behaviors

template class moveit_pro::behaviors::ActionClientBehaviorBase<franka_behaviors::Grasp>;
