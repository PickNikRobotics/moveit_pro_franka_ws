// Copyright 2026 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <array>
#include <functional>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <fmt/format.h>
#include <tl_expected/expected.hpp>

namespace franka_behaviors
{
// Number of joints on the Franka FR3 arm. Sizes the torque-threshold arrays in the franka_msgs services.
inline constexpr std::size_t kNumJoints = 7;
// Number of Cartesian force/torque components at the end-effector: [Fx, Fy, Fz, Tx, Ty, Tz].
inline constexpr std::size_t kNumCartesianComponents = 6;

// Port IDs shared by both collision-behavior Behaviors.
inline constexpr auto kPortServiceName = "service_name";
inline constexpr auto kPortTranslationalForces = "translational_forces";
inline constexpr auto kPortRotationalForces = "rotational_forces";

// Builds the joint-torque port ID for a 1-based joint index (joint1..joint7).
inline std::string jointPortId(std::size_t one_based_joint_index)
{
  return fmt::format("joint{}", one_based_joint_index);
}

/**
 * @brief Collision-threshold values resolved from the Behavior's input ports.
 * @details The same values populate the lower and upper threshold arrays of the franka_msgs services, per the
 * requirement that lower and upper thresholds are equal and driven by a single port each.
 */
struct CollisionThresholds
{
  // Per-joint torque thresholds [Nm], indexed joint1..joint7.
  std::array<double, kNumJoints> torque;
  // Cartesian force/torque thresholds at the end-effector: [Fx, Fy, Fz, Tx, Ty, Tz].
  // The translational_forces port fills Fx/Fy/Fz; the rotational_forces port fills Tx/Ty/Tz.
  std::array<double, kNumCartesianComponents> force;
};

/**
 * @brief Assemble the torque and Cartesian force/torque threshold arrays from per-port values.
 * @param joint_torques The seven per-joint torque thresholds (joint1..joint7).
 * @param translational_force The Cartesian translational force threshold applied to Fx, Fy, and Fz.
 * @param rotational_force The Cartesian rotational force (torque) threshold applied to Tx, Ty, and Tz.
 */
inline CollisionThresholds makeThresholds(const std::array<double, kNumJoints>& joint_torques,
                                          double translational_force, double rotational_force)
{
  return CollisionThresholds{ joint_torques,
                              { translational_force, translational_force, translational_force, rotational_force,
                                rotational_force, rotational_force } };
}

// Reads a required double input port; named to surface which port failed.
using DoublePortReader = std::function<tl::expected<double, std::string>(const std::string&)>;

/**
 * @brief Read all collision-threshold input ports through the provided reader.
 * @details Factored out so both Behaviors share identical port-reading and validation logic. The reader wraps the
 * Behavior's protected getInput<double>(), which a free function cannot call directly.
 * @return The resolved thresholds, or an error naming the first port that could not be read.
 */
inline tl::expected<CollisionThresholds, std::string> readThresholds(const DoublePortReader& read_port)
{
  std::array<double, kNumJoints> joint_torques{};
  for (std::size_t i = 0; i < kNumJoints; ++i)
  {
    const std::string port_id = jointPortId(i + 1);
    const auto value = read_port(port_id);
    if (!value.has_value())
    {
      return tl::make_unexpected(
          fmt::format("Failed to get required value from input port '{}': {}", port_id, value.error()));
    }
    joint_torques[i] = value.value();
  }

  const auto translational_force = read_port(kPortTranslationalForces);
  if (!translational_force.has_value())
  {
    return tl::make_unexpected(fmt::format("Failed to get required value from input port '{}': {}",
                                           kPortTranslationalForces, translational_force.error()));
  }

  const auto rotational_force = read_port(kPortRotationalForces);
  if (!rotational_force.has_value())
  {
    return tl::make_unexpected(fmt::format("Failed to get required value from input port '{}': {}",
                                           kPortRotationalForces, rotational_force.error()));
  }

  return makeThresholds(joint_torques, translational_force.value(), rotational_force.value());
}

/**
 * @brief Append the collision-threshold input ports (joint1..joint7, translational_forces, rotational_forces) shared
 * by both collision-behavior Behaviors.
 * @details All threshold ports are required (no default) so the operator must consciously set every limit before the
 * Behavior reconfigures the robot's reflexes.
 */
inline void addThresholdPorts(BT::PortsList& ports)
{
  for (std::size_t i = 1; i <= kNumJoints; ++i)
  {
    ports.insert(BT::InputPort<double>(
        jointPortId(i),
        fmt::format("Collision torque threshold [Nm] for joint{} (sets both the lower and upper thresholds).", i)));
  }
  ports.insert(BT::InputPort<double>(
      kPortTranslationalForces,
      "Cartesian translational force threshold [N] applied to Fx, Fy, and Fz at the end-effector (sets both the lower "
      "and upper thresholds)."));
  ports.insert(BT::InputPort<double>(
      kPortRotationalForces,
      "Cartesian rotational force (torque) threshold [Nm] applied to the torques about x, y, and z at the "
      "end-effector (sets both the lower and upper thresholds)."));
}
}  // namespace franka_behaviors
