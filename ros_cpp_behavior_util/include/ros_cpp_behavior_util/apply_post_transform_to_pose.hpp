////////////////////////////////////////////////////////////////////////////////////////////
//      Author  : Crasun Jans
////////////////////////////////////////////////////////////////////////////////////////////
#ifndef APPLY_POST_TRANSFORM_TO_POSE_HPP_
#define APPLY_POST_TRANSFORM_TO_POSE_HPP_

#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "ros_cpp_behavior_util/ros_node_base.hpp"

namespace ros_cpp_behavior_util
{

/**
 * @brief BehaviorTree sync action node that applies a fixed spatial offset
 *        (post-transform) to an input pose and outputs the resulting pose.
 *
 * Blackboard keys
 * ---------------
 *   "node"  –  rclcpp::Node::SharedPtr   (read, mandatory)
 *
 * Input ports
 * -----------
 *   goal_pose_type  – Which offset to apply: "wbc_approach_start" or "arm_approach_start"
 *   input_pose      – std::shared_ptr<geometry_msgs::msg::PoseStamped> to transform
 *
 * Output ports
 * ------------
 *   output_pose  – std::shared_ptr<geometry_msgs::msg::PoseStamped> with the offset applied
 */
class ApplyPostTransformToPose : public RosSyncActionBase
{
public:
    using PoseStamped = geometry_msgs::msg::PoseStamped;

    ApplyPostTransformToPose(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    // ── Fixed spatial offsets from the tag frame ──────────────────────────────
    static const Eigen::Matrix4d T_tag_to_arm_approach_start_;
    static const Eigen::Matrix4d T_tag_to_wbc_approach_start_;

    static constexpr char ref_frame_[] = "odom";
};

}  // namespace ros_cpp_behavior_util

#endif  // APPLY_POST_TRANSFORM_TO_POSE_HPP_
