#include <behaviortree_cpp/bt_factory.h>
#include <moveit_pro_behavior_interface/behavior_context.hpp>
#include <moveit_pro_behavior_interface/shared_resources_node_loader.hpp>

#include <franka_behaviors/franka_grasp_action.hpp>
#include <franka_behaviors/set_force_torque_collision_behavior.hpp>
#include <franka_behaviors/set_full_collision_behavior.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace franka_behaviors
{
class FrankaBehaviorsLoader : public moveit_pro::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
                         const std::shared_ptr<moveit_pro::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_pro::behaviors::registerBehavior<GraspAction>(factory, "FrankaGraspAction", shared_resources);
    moveit_pro::behaviors::registerBehavior<SetForceTorqueCollisionBehavior>(
        factory, "FrankaSetForceTorqueCollisionBehavior", shared_resources);
    moveit_pro::behaviors::registerBehavior<SetFullCollisionBehavior>(factory, "FrankaSetFullCollisionBehavior",
                                                                         shared_resources);
  }
};
}  // namespace franka_behaviors

PLUGINLIB_EXPORT_CLASS(franka_behaviors::FrankaBehaviorsLoader, moveit_pro::behaviors::SharedResourcesNodeLoaderBase);
