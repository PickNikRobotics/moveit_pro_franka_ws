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

#include <franka_behaviors/spine_state_service.hpp>

#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/impl/service_client_behavior_base_impl.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

#include <string>
#include <utility>

namespace
{
constexpr auto kPortServiceName = "service_name";
constexpr auto kPortTimeout = "timeout";

constexpr auto kDescriptionSwitchOnSpine = R"(
  <p>Switches the real Franka spine on through franka_spine_server's <code>switch_on</code> service, so it accepts
  <code>MoveSpineAbsolute</code> goals.</p>
  <p>Fails when the spine refuses, for example in a fault state; clear the fault with <code>ResetSpineFault</code>
  first.</p>
)";

constexpr auto kDescriptionResetSpineFault = R"(
  <p>Clears a fault on the real Franka spine through franka_spine_server's <code>fault_reset</code> service. The spine
  is then switched off; switch it on with <code>SwitchOnSpine</code>.</p>
  <p>Fails when the spine cannot clear the fault.</p>
)";
}  // namespace

namespace franka_behaviors
{
template <typename ServiceT>
SpineStateService<ServiceT>::SpineStateService(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources, std::string command)
  : moveit_pro::behaviors::ServiceClientBehaviorBase<ServiceT>(name, config, shared_resources)
  , command_(std::move(command))
{
}

template <typename ServiceT>
BT::PortsList SpineStateService<ServiceT>::servicePorts(const std::string& default_service_name)
{
  return {
    BT::InputPort<std::string>(kPortServiceName, default_service_name, "The franka_spine_server service."),
    BT::InputPort<double>(kPortTimeout, 10.0, "Seconds to wait for the service response before the Behavior fails."),
  };
}

template <typename ServiceT>
tl::expected<std::string, std::string> SpineStateService<ServiceT>::getServiceName()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(this->template getInput<std::string>(kPortServiceName));
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

template <typename ServiceT>
tl::expected<typename ServiceT::Request, std::string> SpineStateService<ServiceT>::createRequest()
{
  return typename ServiceT::Request{};
}

template <typename ServiceT>
tl::expected<bool, std::string> SpineStateService<ServiceT>::processResponse(const typename ServiceT::Response& response)
{
  if (!response.success)
  {
    std::string detail = " (spine state '" + response.state + "')";
    if (!response.message.empty())
    {
      detail += ": " + response.message;
    }
    return tl::make_unexpected("Failed to " + command_ + detail);
  }
  return true;
}

template <typename ServiceT>
tl::expected<std::chrono::duration<double>, std::string> SpineStateService<ServiceT>::getResponseTimeout()
{
  const auto ports = moveit_pro::behaviors::getRequiredInputs(this->template getInput<double>(kPortTimeout));
  if (!ports.has_value())
  {
    return tl::make_unexpected("Failed to get required value from input data port: " + ports.error());
  }
  const auto& [timeout] = ports.value();
  return std::chrono::duration<double>{ timeout };
}

template class SpineStateService<SwitchOnSrv>;
template class SpineStateService<FaultResetSrv>;

SwitchOnSpine::SwitchOnSpine(const std::string& name, const BT::NodeConfiguration& config,
                             const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : SpineStateService<SwitchOnSrv>(name, config, shared_resources, "switch the spine on")
{
}

BT::PortsList SwitchOnSpine::providedPorts()
{
  return servicePorts("/franka_spine_node/switch_on");
}

BT::KeyValueVector SwitchOnSpine::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Motion - Controls" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionSwitchOnSpine } };
}

ResetSpineFault::ResetSpineFault(const std::string& name, const BT::NodeConfiguration& config,
                                 const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : SpineStateService<FaultResetSrv>(name, config, shared_resources, "clear the spine fault")
{
}

BT::PortsList ResetSpineFault::providedPorts()
{
  return servicePorts("/franka_spine_node/fault_reset");
}

BT::KeyValueVector ResetSpineFault::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Motion - Controls" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionResetSpineFault } };
}

}  // namespace franka_behaviors

template class moveit_pro::behaviors::ServiceClientBehaviorBase<franka_behaviors::SwitchOnSrv>;
template class moveit_pro::behaviors::ServiceClientBehaviorBase<franka_behaviors::FaultResetSrv>;
