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

#include <franka_spine_msgs/action/move_absolute.hpp>
#include <moveit_pro_behavior_interface/action_client_behavior_base.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace franka_behaviors
{
using MoveAbsolute = franka_spine_msgs::action::MoveAbsolute;

/**
 * @brief Moves the real Franka spine to an absolute height through franka_spine_server's `move_absolute` action.
 *
 * @details
 * The real spine has no ros2_control interface, so no trajectory controller can move it. The spine must be switched
 * on first (SwitchOnSpine). The Behavior does not check the height against the spine travel; the spine rejects a
 * height outside it.
 *
 * | Data Port Name | Port Type | Object Type |
 * | -------------- | --------- | ----------- |
 * | action_name    | Input     | std::string |
 * | position       | Input     | double      |
 * | velocity       | Input     | double      |
 * | acceleration   | Input     | double      |
 * | deceleration   | Input     | double      |
 * | timeout        | Input     | double      |
 */
class MoveSpineAbsolute final : public moveit_pro::behaviors::ActionClientBehaviorBase<MoveAbsolute>
{
public:
  MoveSpineAbsolute(const std::string& name, const BT::NodeConfiguration& config,
                    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

private:
  tl::expected<std::string, std::string> getActionName() override;

  tl::expected<std::chrono::duration<double>, std::string> getResultTimeout() override;

  tl::expected<MoveAbsolute::Goal, std::string> createGoal() override;

  /** @brief Fails when the spine reports that it did not reach the target. */
  tl::expected<bool, std::string> processResult(const std::shared_ptr<MoveAbsolute::Result> result) override;

  /** @brief Adds the spine's own error text to the failure message. */
  std::string getAbortedMessage(const std::shared_ptr<const MoveAbsolute::Result> result) const override;

  std::shared_future<tl::expected<bool, std::string>>& getFuture() override
  {
    return future_;
  }

  std::shared_future<tl::expected<bool, std::string>> future_;
};
}  // namespace franka_behaviors
