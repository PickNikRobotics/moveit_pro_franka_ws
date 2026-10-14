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

#include <franka_spine_msgs/srv/fault_reset.hpp>
#include <franka_spine_msgs/srv/switch_on.hpp>
#include <moveit_pro_behavior_interface/service_client_behavior_base.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

#include <string>

namespace franka_behaviors
{
using SwitchOnSrv = franka_spine_msgs::srv::SwitchOn;
using FaultResetSrv = franka_spine_msgs::srv::FaultReset;

/**
 * @brief Calls one of franka_spine_server's state services, which take an empty request and answer with success,
 * the spine state and a message.
 *
 * @details
 * Fails when the spine refuses the command, with the spine's state and message.
 *
 * | Data Port Name | Port Type | Object Type |
 * | -------------- | --------- | ----------- |
 * | service_name   | Input     | std::string |
 * | timeout        | Input     | double      |
 */
template <typename ServiceT>
class SpineStateService : public moveit_pro::behaviors::ServiceClientBehaviorBase<ServiceT>
{
public:
  /** @param command What the service does, for messages (for example "switch the spine on"). */
  SpineStateService(const std::string& name, const BT::NodeConfiguration& config,
                    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources,
                    std::string command);

protected:
  /** @brief The ports of every spine state service, with this service's default name. */
  static BT::PortsList servicePorts(const std::string& default_service_name);

private:
  tl::expected<std::string, std::string> getServiceName() override;

  tl::expected<typename ServiceT::Request, std::string> createRequest() override;

  tl::expected<bool, std::string> processResponse(const typename ServiceT::Response& response) override;

  tl::expected<std::chrono::duration<double>, std::string> getResponseTimeout() override;

  std::shared_future<tl::expected<bool, std::string>>& getFuture() override
  {
    return future_;
  }

  std::string command_;
  std::shared_future<tl::expected<bool, std::string>> future_;
};

/**
 * @brief Switches the real Franka spine on, so it accepts MoveSpineAbsolute goals.
 *
 * @details
 * A spine in a fault state refuses; clear the fault with ResetSpineFault first.
 *
 * | Data Port Name | Port Type | Object Type |
 * | -------------- | --------- | ----------- |
 * | service_name   | Input     | std::string |
 * | timeout        | Input     | double      |
 */
class SwitchOnSpine final : public SpineStateService<SwitchOnSrv>
{
public:
  SwitchOnSpine(const std::string& name, const BT::NodeConfiguration& config,
                const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();
};

/**
 * @brief Clears a fault on the real Franka spine. The spine is then switched off; switch it on with SwitchOnSpine.
 *
 * | Data Port Name | Port Type | Object Type |
 * | -------------- | --------- | ----------- |
 * | service_name   | Input     | std::string |
 * | timeout        | Input     | double      |
 */
class ResetSpineFault final : public SpineStateService<FaultResetSrv>
{
public:
  ResetSpineFault(const std::string& name, const BT::NodeConfiguration& config,
                  const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();
};
}  // namespace franka_behaviors
