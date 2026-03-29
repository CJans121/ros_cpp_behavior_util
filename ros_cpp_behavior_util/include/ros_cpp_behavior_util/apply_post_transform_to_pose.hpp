#ifndef APPLY_POST_TRANSFORM_TO_POSE_HPP_
#define APPLY_POST_TRANSFORM_TO_POSE_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

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
class ApplyPostTransformToPose : public BT::SyncActionNode
{
public:
    using PoseStamped = geometry_msgs::msg::PoseStamped;

    ApplyPostTransformToPose(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

private:
    /// Retrieve the shared node from the blackboard; throws BT::RuntimeError if missing.
    rclcpp::Node::SharedPtr getNode() const;

    // ── Fixed spatial offsets from the tag frame ──────────────────────────────
    const Eigen::Matrix4d T_tag_to_arm_approach_start_ =
        (Eigen::Matrix4d() <<
             0.0, -1.0, 0.0, 0.0,
             0.0,  0.0, 1.0, 0.0,
            -1.0,  0.0, 0.0, 0.8,
             0.0,  0.0, 0.0, 1.0).finished();

    const Eigen::Matrix4d T_tag_to_wbc_approach_start_ =
        (Eigen::Matrix4d() <<
             0.0, -1.0, 0.0, 0.0,
             0.0,  0.0, 1.0, 0.0,
            -1.0,  0.0, 0.0, 1.5,
             0.0,  0.0, 0.0, 1.0).finished();

    static constexpr char ref_frame_[] = "odom";
};

}  // namespace ros_cpp_behavior_util

#endif  // APPLY_POST_TRANSFORM_TO_POSE_HPP_
