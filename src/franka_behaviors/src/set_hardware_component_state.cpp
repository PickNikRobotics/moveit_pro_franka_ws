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

#include <franka_behaviors/set_hardware_component_state.hpp>

#include <lifecycle_msgs/msg/state.hpp>
#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/impl/service_client_behavior_base_impl.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

#include <algorithm>
#include <cctype>
#include <optional>
#include <string>

namespace
{
constexpr auto kPortServiceName = "service_name";
constexpr auto kPortHardwareName = "hardware_name";
constexpr auto kPortTargetState = "target_state";
constexpr auto kPortTimeout = "timeout";

constexpr auto kDescriptionSetHardwareComponentState = R"(
  <p>Moves a ros2_control hardware component to a lifecycle state (active, inactive, unconfigured or finalized)
  through the controller manager's <code>set_hardware_component_state</code> service.</p>
  <p>Use it to bring a hardware component back after a driver error: the error takes the component out of the active
  state, and <code>SwitchController</code> alone cannot restore it.</p>
  <p>A request for the state the component is already in reports failure. Wrap such a step in
  <code>ForceSuccess</code>.</p>
)";

// The controller manager matches on the id; the label makes the request readable in logs.
std::optional<lifecycle_msgs::msg::State> parseTargetState(std::string name)
{
  std::transform(name.begin(), name.end(), name.begin(),
                 [](unsigned char character) { return static_cast<char>(std::tolower(character)); });
  lifecycle_msgs::msg::State state;
  if (name == "active")
  {
    state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  }
  else if (name == "inactive")
  {
    state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
  }
  else if (name == "unconfigured")
  {
    state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;
  }
  else if (name == "finalized")
  {
    state.id = lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED;
  }
  else
  {
    return std::nullopt;
  }
  state.label = name;
  return state;
}
}  // namespace

namespace franka_behaviors
{
SetHardwareComponentState::SetHardwareComponentState(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::ServiceClientBehaviorBase<SetHardwareComponentStateSrv>(name, config, shared_resources)
{
}

BT::PortsList SetHardwareComponentState::providedPorts()
{
  return {
    BT::InputPort<std::string>(kPortServiceName, "/controller_manager/set_hardware_component_state",
                               "The controller manager's set_hardware_component_state service."),
    BT::InputPort<std::string>(kPortHardwareName, "",
                               "The hardware component: the name attribute of its <ros2_control> tag in the URDF."),
    BT::InputPort<std::string>(kPortTargetState, "active",
                               "The target lifecycle state: active, inactive, unconfigured or finalized."),
    BT::InputPort<double>(kPortTimeout, 10.0, "Seconds to wait for the service response before the Behavior fails."),
  };
}

BT::KeyValueVector SetHardwareComponentState::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Motion - Controls" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionSetHardwareComponentState } };
}

tl::expected<std::string, std::string> SetHardwareComponentState::getServiceName()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(getInput<std::string>(kPortServiceName));
  if (!ports.has_value())
  {
    return tl::make_unexpected("Failed to get required value from input data port: " + ports.error());
  }
  const auto& [service_name] = ports.value();
  if (service_name.empty())
  {
    return tl::make_unexpected(std::string("The '") + kPortServiceName + "' port must not be empty.");
  }
  return service_name;
}

tl::expected<SetHardwareComponentStateSrv::Request, std::string> SetHardwareComponentState::createRequest()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(getInput<std::string>(kPortHardwareName),
                                                              getInput<std::string>(kPortTargetState));
  if (!ports.has_value())
  {
    return tl::make_unexpected("Failed to get required values from input data ports: " + ports.error());
  }
  const auto& [hardware_name, target_state] = ports.value();
  if (hardware_name.empty())
  {
    return tl::make_unexpected(std::string("The '") + kPortHardwareName + "' port must name a hardware component.");
  }
  const auto state = parseTargetState(target_state);
  if (!state.has_value())
  {
    return tl::make_unexpected(std::string("Invalid '") + kPortTargetState + "' value '" + target_state +
                               "'. Use active, inactive, unconfigured or finalized.");
  }

  SetHardwareComponentStateSrv::Request request;
  request.name = hardware_name;
  request.target_state = state.value();
  return request;
}

tl::expected<bool, std::string>
SetHardwareComponentState::processResponse(const SetHardwareComponentStateSrv::Response& response)
{
  if (!response.ok)
  {
    return tl::make_unexpected("The controller manager did not change the hardware component state. The component "
                               "is in the '" +
                               response.state.label + "' state.");
  }
  return true;
}

tl::expected<std::chrono::duration<double>, std::string> SetHardwareComponentState::getResponseTimeout()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(getInput<double>(kPortTimeout));
  if (!ports.has_value())
  {
    return tl::make_unexpected("Failed to get required value from input data port: " + ports.error());
  }
  const auto& [timeout] = ports.value();
  return std::chrono::duration<double>{ timeout };
}
}  // namespace franka_behaviors

template class moveit_pro::behaviors::ServiceClientBehaviorBase<franka_behaviors::SetHardwareComponentStateSrv>;
