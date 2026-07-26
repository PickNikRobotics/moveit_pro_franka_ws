// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <franka_msgs/srv/set_force_torque_collision_behavior.hpp>
#include <moveit_pro_behavior_interface/service_client_behavior_base.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace franka_behaviors
{
using SetForceTorqueCollisionBehaviorSrv = franka_msgs::srv::SetForceTorqueCollisionBehavior;

/**
 * @brief Calls the franka_ros2 `set_force_torque_collision_behavior` service to set the contact/collision reflex
 * thresholds used during nominal (non-acceleration) motion.
 *
 * @details Each `joint1`..`joint7` port supplies the torque threshold for one joint and sets both the lower and upper
 * nominal torque thresholds for that joint. The `translational_forces` port sets the lower and upper nominal force
 * thresholds for Fx, Fy, and Fz at the end-effector; the `rotational_forces` port sets them for the torques about x,
 * y, and z. Exceeding a threshold triggers a Franka reflex; recover with the franka error-recovery action.
 *
 * | Data Port Name        | Port Type | Object Type |
 * | --------------------- | --------- | ----------- |
 * | service_name          | Input     | std::string |
 * | joint1 .. joint7      | Input     | double      |
 * | translational_forces  | Input     | double      |
 * | rotational_forces     | Input     | double      |
 */
class SetForceTorqueCollisionBehavior final
  : public moveit_pro::behaviors::ServiceClientBehaviorBase<SetForceTorqueCollisionBehaviorSrv>
{
public:
  SetForceTorqueCollisionBehavior(const std::string& name, const BT::NodeConfiguration& config,
                                  const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

private:
  tl::expected<std::string, std::string> getServiceName() override;

  tl::expected<SetForceTorqueCollisionBehaviorSrv::Request, std::string> createRequest() override;

  tl::expected<bool, std::string> processResponse(const SetForceTorqueCollisionBehaviorSrv::Response& response) override;

  /** @brief Classes derived from AsyncBehaviorBase must implement getFuture() so that it returns a shared_future class
   * member. */
  std::shared_future<tl::expected<bool, std::string>>& getFuture() override
  {
    return future_;
  }

  /** @brief Classes derived from AsyncBehaviorBase must have this shared_future as a class member. */
  std::shared_future<tl::expected<bool, std::string>> future_;
};
}  // namespace franka_behaviors
