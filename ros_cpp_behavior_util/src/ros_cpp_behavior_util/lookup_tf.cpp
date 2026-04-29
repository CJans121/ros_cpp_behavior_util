#include "ros_cpp_behavior_util/lookup_tf.hpp"

#include <cmath>
#include <tf2/exceptions.h>
#include <tf2/time.h>

namespace ros_cpp_behavior_util
{

// ── Construction ─────────────────────────────────────────────────────────────

LookupTF::LookupTF(const std::string& name, const BT::NodeConfig& config)
: RosStatefulActionBase(name, config)
{
}

// ── Port declaration ─────────────────────────────────────────────────────────

BT::PortsList LookupTF::providedPorts()
{
    return {
        BT::InputPort<std::string>("ref_frame",    "Parent / reference frame"),
        BT::InputPort<std::string>("target_frame", "Child / target frame to locate"),
        BT::InputPort<double>     ("timeout_secs", "Wall-clock timeout in seconds"),
        BT::OutputPort<std::shared_ptr<geometry_msgs::msg::PoseStamped>>("pose",
            "Resolved pose of target_frame expressed in ref_frame"),
    };
}

// ── onStart ───────────────────────────────────────────────────────────────────

BT::NodeStatus LookupTF::onStart()
{
    if (!getInput<std::string>("ref_frame", ref_frame_))
        throw BT::RuntimeError("[LookupTF] missing required input [ref_frame]");

    if (!getInput<std::string>("target_frame", target_frame_))
        throw BT::RuntimeError("[LookupTF] missing required input [target_frame]");

    if (!getInput<double>("timeout_secs", timeout_secs_))
        throw BT::RuntimeError("[LookupTF] missing required input [timeout_secs]");

    node_ = getNode();

    // Always recreate so a halted-then-restarted node gets a fresh buffer.
    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_,
                                                                /*spin_thread=*/false);
    tf_buffer_->setUsingDedicatedThread(true); // tell buffer spinner exists, caller is responsible for spinning
    tf_queue_.clear();
    start_time_ = std::chrono::steady_clock::now();

    RCLCPP_DEBUG(node_->get_logger(),
                 "[LookupTF] starting lookup '%s' → '%s' (timeout %.2f s)",
                 ref_frame_.c_str(), target_frame_.c_str(), timeout_secs_);

    return BT::NodeStatus::RUNNING;
}

// ── onRunning ─────────────────────────────────────────────────────────────────

BT::NodeStatus LookupTF::onRunning()
{
    const auto   now     = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - start_time_).count();

    if (elapsed > timeout_secs_)
    {
        RCLCPP_WARN(node_->get_logger(),
                    "[LookupTF] timed out after %.2f s waiting for '%s' → '%s'",
                    elapsed, ref_frame_.c_str(), target_frame_.c_str());
        return BT::NodeStatus::FAILURE;
    }

    geometry_msgs::msg::TransformStamped tf_stamped;
    try
    {
        tf_stamped = tf_buffer_->lookupTransform(
            ref_frame_,
            target_frame_,
            tf2::TimePointZero,
            tf2::durationFromSec(tf_lookup_timeout_secs_));
    }
    catch (const tf2::TransformException& ex)
    {
        RCLCPP_DEBUG(node_->get_logger(),
                     "[LookupTF] transform not yet available: %s", ex.what());
        return BT::NodeStatus::RUNNING;
    }

    // ── Accumulate samples for steady-read check ─────────────────────────────
    tf_queue_.push_back(tf_stamped);
    if (tf_queue_.size() > steady_required_samples_)
        tf_queue_.pop_front();

    if (tf_queue_.size() < steady_required_samples_)
        return BT::NodeStatus::RUNNING;

    // ── Verify all samples are within the position threshold ─────────────────
    const auto& ref = tf_queue_.back().transform.translation;
    for (const auto& sample : tf_queue_)
    {
        const auto& t    = sample.transform.translation;
        const double dist = std::sqrt(
            std::pow(t.x - ref.x, 2) +
            std::pow(t.y - ref.y, 2) +
            std::pow(t.z - ref.z, 2));

        if (dist > steady_position_threshold_)
            return BT::NodeStatus::RUNNING;
    }

    // ── Steady read achieved – convert and output ─────────────────────────────
    geometry_msgs::msg::PoseStamped pose_out;
    pose_out.header           = tf_stamped.header;
    pose_out.pose.position.x  = ref.x;
    pose_out.pose.position.y  = ref.y;
    pose_out.pose.position.z  = ref.z;
    pose_out.pose.orientation = tf_stamped.transform.rotation;

    setOutput("pose", std::make_shared<geometry_msgs::msg::PoseStamped>(pose_out));

    RCLCPP_DEBUG(node_->get_logger(),
                 "[LookupTF] steady transform found: (%.3f, %.3f, %.3f)",
                 ref.x, ref.y, ref.z);

    return BT::NodeStatus::SUCCESS;
}

// ── onHalted ──────────────────────────────────────────────────────────────────

void LookupTF::onHalted()
{
    // Release TF resources so no callbacks fire on a stale buffer while idle.
    tf_listener_.reset();
    tf_buffer_.reset();
    tf_queue_.clear();

    RCLCPP_DEBUG(node_->get_logger(), "[LookupTF] halted — TF resources released");
}

}  // namespace ros_cpp_behavior_util
