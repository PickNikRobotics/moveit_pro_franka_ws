// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include "franka_behaviors/set_full_collision_behavior.hpp"

#include <fmt/format.h>

#include <moveit_pro_behavior_interface/get_required_ports.hpp>
#include <moveit_pro_behavior_interface/impl/service_client_behavior_base_impl.hpp>
#include <moveit_pro_behavior_interface/metadata_fields.hpp>

#include "franka_behaviors/collision_thresholds.hpp"

namespace
{
inline constexpr auto kDescriptionSetFullCollisionBehavior = R"(
  <p>Sets the Franka FR3 contact/collision reflex thresholds for both the acceleration and nominal motion phases by
  calling the franka_ros2 <code>set_full_collision_behavior</code> service.</p>
  <p>Each <code>joint1</code>..<code>joint7</code> port sets the lower and upper, acceleration and nominal torque
  thresholds for that joint. <code>translational_forces</code> sets the corresponding force thresholds for Fx, Fy, and
  Fz at the end-effector; <code>rotational_forces</code> sets them for the torques about x, y, and z. These
  end-effector torques are distinct from the per-joint torques set by the joint ports.</p>
)";

// Default single-arm service name exposed by the franka_ros2 service server. Override the service_name port for
// dual-arm setups (e.g. /left/service_server/set_full_collision_behavior).
constexpr auto kDefaultServiceName = "/service_server/set_full_collision_behavior";
}  // namespace

namespace franka_behaviors
{
SetFullCollisionBehavior::SetFullCollisionBehavior(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources)
  : moveit_pro::behaviors::ServiceClientBehaviorBase<SetFullCollisionBehaviorSrv>(name, config, shared_resources)
{
}

BT::PortsList SetFullCollisionBehavior::providedPorts()
{
  BT::PortsList ports = { BT::InputPort<std::string>(
      kPortServiceName, kDefaultServiceName, "Name of the franka_ros2 set_full_collision_behavior service.") };
  addThresholdPorts(ports);
  return ports;
}

BT::KeyValueVector SetFullCollisionBehavior::metadata()
{
  return { { moveit_pro::behaviors::kSubcategoryMetadataKey, "Franka" },
           { moveit_pro::behaviors::kDescriptionMetadataKey, kDescriptionSetFullCollisionBehavior } };
}

tl::expected<std::string, std::string> SetFullCollisionBehavior::getServiceName()
{
  const auto service_name = getInput<std::string>(kPortServiceName);
  if (!service_name.has_value())
  {
    return tl::make_unexpected(
        fmt::format("Failed to get required value from input port '{}': {}", kPortServiceName, service_name.error()));
  }
  if (service_name.value().empty())
  {
    return tl::make_unexpected(fmt::format("The '{}' port must contain a valid service name.", kPortServiceName));
  }
  return service_name.value();
}

tl::expected<SetFullCollisionBehaviorSrv::Request, std::string> SetFullCollisionBehavior::createRequest()
{
  const auto thresholds = readThresholds([this](const std::string& port_id) -> tl::expected<double, std::string> {
    const auto value = getInput<double>(port_id);
    if (!value)
    {
      return tl::make_unexpected(value.error());
    }
    return value.value();
  });
  if (!thresholds.has_value())
  {
    return tl::make_unexpected(thresholds.error());
  }

  // Each port drives both the lower/upper and acceleration/nominal arrays, so a joint or force maps to one port.
  SetFullCollisionBehaviorSrv::Request request;
  request.lower_torque_thresholds_acceleration = thresholds->torque;
  request.upper_torque_thresholds_acceleration = thresholds->torque;
  request.lower_torque_thresholds_nominal = thresholds->torque;
  request.upper_torque_thresholds_nominal = thresholds->torque;
  request.lower_force_thresholds_acceleration = thresholds->force;
  request.upper_force_thresholds_acceleration = thresholds->force;
  request.lower_force_thresholds_nominal = thresholds->force;
  request.upper_force_thresholds_nominal = thresholds->force;
  return request;
}

tl::expected<bool, std::string>
SetFullCollisionBehavior::processResponse(const SetFullCollisionBehaviorSrv::Response& response)
{
  if (!response.success)
  {
    return tl::make_unexpected(fmt::format("set_full_collision_behavior service reported failure: {}", response.error));
  }
  return true;
}
}  // namespace franka_behaviors

// This specializes ServiceClientBehaviorBase for the SetFullCollisionBehavior service.
template class moveit_pro::behaviors::ServiceClientBehaviorBase<franka_behaviors::SetFullCollisionBehaviorSrv>;
