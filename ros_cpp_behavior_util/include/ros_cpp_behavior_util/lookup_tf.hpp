#ifndef LOOKUP_TF_HPP_
#define LOOKUP_TF_HPP_

#include <behaviortree_cpp/action_node.h>
#include <chrono>
#include <deque>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace ros_cpp_behavior_util
{

/**
 * @brief BehaviorTree action node that looks up a TF transform and outputs a
 *        PoseStamped once the transform has converged to a steady value.
 *
 * Blackboard keys
 * ---------------
 *   "node"  –  rclcpp::Node::SharedPtr   (read, mandatory)
 *
 * Input ports
 * -----------
 *   ref_frame      – Parent / reference frame name
 *   target_frame   – Child / target frame name
 *   timeout_secs   – Maximum wall-clock seconds to wait before failing
 *
 * Output ports
 * ------------
 *   pose – std::shared_ptr<geometry_msgs::msg::PoseStamped>
 */
class LookupTF : public BT::StatefulActionNode
{
public:
    LookupTF(const std::string& name, const BT::NodeConfig& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void           onHalted() override;

private:
    /// Retrieve the shared node from the blackboard; throws on failure.
    rclcpp::Node::SharedPtr getNode() const;


    // ROS node (retrieved once in onStart)
    rclcpp::Node::SharedPtr node_;

    // ── TF infrastructure (created once in onStart) ──────────────────────────
    std::unique_ptr<tf2_ros::Buffer>            tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ── Port values ──────────────────────────────────────────────────────────
    std::string ref_frame_;
    std::string target_frame_;
    double      timeout_secs_{0.0};

    // ── Timing ───────────────────────────────────────────────────────────────
    std::chrono::steady_clock::time_point start_time_;

    // ── Steady-read bookkeeping ───────────────────────────────────────────────
    std::deque<geometry_msgs::msg::TransformStamped> tf_queue_;

    /// How close consecutive samples must be (metres) to count as "steady".
    static constexpr double steady_position_threshold_{0.02};

    /// Number of consecutive samples required before declaring a steady read.
    static constexpr std::size_t steady_required_samples_{10};

    /// Short per-call timeout forwarded to tf2::Buffer::lookupTransform.
    static constexpr double tf_lookup_timeout_secs_{0.05};
};

}  // namespace ros_cpp_behavior_util

#endif  // LOOKUP_TF_HPP_
