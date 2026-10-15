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

#pragma once

#include <controller_manager_msgs/srv/set_hardware_component_state.hpp>
#include <moveit_pro_behavior_interface/service_client_behavior_base.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace franka_behaviors
{
using SetHardwareComponentStateSrv = controller_manager_msgs::srv::SetHardwareComponentState;

/**
 * @brief Moves a ros2_control hardware component to a lifecycle state through the controller manager's
 * `set_hardware_component_state` service.
 *
 * @details
 * A driver error takes the component out of the active state, and SwitchController cannot bring it back.
 * Nothing here is Franka-specific. Details: docs/AGENT_COMMENTS.md.
 *
 * | Data Port Name | Port Type | Object Type |
 * | -------------- | --------- | ----------- |
 * | service_name   | Input     | std::string |
 * | hardware_name  | Input     | std::string |
 * | target_state   | Input     | std::string |
 * | timeout        | Input     | double      |
 */
class SetHardwareComponentState final
  : public moveit_pro::behaviors::ServiceClientBehaviorBase<SetHardwareComponentStateSrv>
{
public:
  SetHardwareComponentState(const std::string& name, const BT::NodeConfiguration& config,
                            const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

private:
  tl::expected<std::string, std::string> getServiceName() override;

  tl::expected<SetHardwareComponentStateSrv::Request, std::string> createRequest() override;

  /** @brief Fails when the controller manager reports that the transition did not happen. */
  tl::expected<bool, std::string> processResponse(const SetHardwareComponentStateSrv::Response& response) override;

  tl::expected<std::chrono::duration<double>, std::string> getResponseTimeout() override;

  std::shared_future<tl::expected<bool, std::string>>& getFuture() override
  {
    return future_;
  }

  std::shared_future<tl::expected<bool, std::string>> future_;
};
}  // namespace franka_behaviors
