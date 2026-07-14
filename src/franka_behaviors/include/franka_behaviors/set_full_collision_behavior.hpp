// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <franka_msgs/srv/set_full_collision_behavior.hpp>
#include <moveit_pro_behavior_interface/service_client_behavior_base.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node.hpp>

namespace franka_behaviors
{
using SetFullCollisionBehaviorSrv = franka_msgs::srv::SetFullCollisionBehavior;

/**
 * @brief Calls the franka_ros2 `set_full_collision_behavior` service to set the contact/collision reflex thresholds
 * for both the acceleration and nominal motion phases.
 *
 * @details This is the superset of SetForceTorqueCollisionBehavior: the franka_msgs service exposes separate
 * acceleration-phase and nominal-phase threshold arrays. Each port here drives both phases — i.e. a single `joint{n}`
 * value sets the lower and upper, acceleration and nominal torque thresholds for that joint, so each joint maps to
 * exactly one port (matching the lower==upper requirement, extended to acceleration==nominal). Split the ports if the
 * two phases ever need distinct limits.
 *
 * The `joint1`..`joint7` ports carry per-joint torque thresholds. These are distinct from the end-effector torques:
 * the `rotational_forces` port carries the Cartesian torque thresholds about x, y, and z at the end-effector, while
 * the `translational_forces` port carries the Cartesian forces along x, y, and z.
 *
 * | Data Port Name        | Port Type | Object Type |
 * | --------------------- | --------- | ----------- |
 * | service_name          | Input     | std::string |
 * | joint1 .. joint7      | Input     | double      |
 * | translational_forces  | Input     | double      |
 * | rotational_forces     | Input     | double      |
 */
class SetFullCollisionBehavior final
  : public moveit_pro::behaviors::ServiceClientBehaviorBase<SetFullCollisionBehaviorSrv>
{
public:
  SetFullCollisionBehavior(const std::string& name, const BT::NodeConfiguration& config,
                           const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

  static BT::KeyValueVector metadata();

private:
  tl::expected<std::string, std::string> getServiceName() override;

  tl::expected<SetFullCollisionBehaviorSrv::Request, std::string> createRequest() override;

  tl::expected<bool, std::string> processResponse(const SetFullCollisionBehaviorSrv::Response& response) override;

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
