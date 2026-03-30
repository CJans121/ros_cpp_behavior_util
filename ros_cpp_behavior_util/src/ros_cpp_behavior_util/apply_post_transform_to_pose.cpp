#include "ros_cpp_behavior_util/apply_post_transform_to_pose.hpp"

namespace ros_cpp_behavior_util
{

// ── Static member definitions ─────────────────────────────────────────────────

const Eigen::Matrix4d ApplyPostTransformToPose::T_tag_to_arm_approach_start_ =
    (Eigen::Matrix4d() <<
         0.0, -1.0, 0.0, 0.0,
         0.0,  0.0, 1.0, 0.0,
        -1.0,  0.0, 0.0, 0.8,
         0.0,  0.0, 0.0, 1.0).finished();

const Eigen::Matrix4d ApplyPostTransformToPose::T_tag_to_wbc_approach_start_ =
    (Eigen::Matrix4d() <<
         0.0, -1.0, 0.0, 0.0,
         0.0,  0.0, 1.0, 0.0,
        -1.0,  0.0, 0.0, 1.5,
         0.0,  0.0, 0.0, 1.0).finished();

// ── Construction ──────────────────────────────────────────────────────────────

ApplyPostTransformToPose::ApplyPostTransformToPose(
    const std::string& name, const BT::NodeConfig& config)
: RosSyncActionBase(name, config)
{
}

// ── Port declaration ──────────────────────────────────────────────────────────

BT::PortsList ApplyPostTransformToPose::providedPorts()
{
    return {
        BT::InputPort<std::string>("goal_pose_type",
            "Which offset to apply: \"wbc_approach_start\" or \"arm_approach_start\""),
        BT::InputPort<std::shared_ptr<PoseStamped>>("input_pose",
            "Pose to which the post-transform will be applied"),
        BT::OutputPort<std::shared_ptr<PoseStamped>>("output_pose",
            "Resulting pose after the offset is applied"),
    };
}

// ── tick ──────────────────────────────────────────────────────────────────────

BT::NodeStatus ApplyPostTransformToPose::tick()
{
    node_ = getNode();

    // ── Read and validate ports ───────────────────────────────────────────────
    const auto input_pose_port = getInput<std::shared_ptr<PoseStamped>>("input_pose");
    if (!input_pose_port)
        throw BT::RuntimeError(
            "[ApplyPostTransformToPose] missing required input [input_pose]");

    const auto goal_pose_type_port = getInput<std::string>("goal_pose_type");
    if (!goal_pose_type_port)
        throw BT::RuntimeError(
            "[ApplyPostTransformToPose] missing required input [goal_pose_type]");

    const PoseStamped& input_msg = *input_pose_port.value();
    const std::string& goal_type = goal_pose_type_port.value();

    if (goal_type != "wbc_approach_start" && goal_type != "arm_approach_start")
        throw BT::RuntimeError(
            "[ApplyPostTransformToPose] invalid [goal_pose_type] \"" + goal_type +
            "\", expected \"wbc_approach_start\" or \"arm_approach_start\"");

    // ── Build input isometry ──────────────────────────────────────────────────
    Eigen::Isometry3d input_pose = Eigen::Isometry3d::Identity();
    input_pose.translation() << input_msg.pose.position.x,
                                input_msg.pose.position.y,
                                input_msg.pose.position.z;
    input_pose.linear() = Eigen::Quaterniond(
        input_msg.pose.orientation.w,
        input_msg.pose.orientation.x,
        input_msg.pose.orientation.y,
        input_msg.pose.orientation.z).toRotationMatrix();

    // ── Apply offset ──────────────────────────────────────────────────────────
    const Eigen::Matrix4d& T_offset = (goal_type == "wbc_approach_start")
        ? T_tag_to_wbc_approach_start_
        : T_tag_to_arm_approach_start_;

    RCLCPP_INFO(node_->get_logger(),
                "[ApplyPostTransformToPose] applying offset for goal type \"%s\"",
                goal_type.c_str());

    const Eigen::Matrix4d output_mat = input_pose.matrix() * T_offset;

    // ── Build output message ──────────────────────────────────────────────────
    auto output_pose = std::make_shared<PoseStamped>();
    output_pose->header.frame_id = ref_frame_;
    output_pose->header.stamp    = node_->now();

    output_pose->pose.position.x = output_mat(0, 3);
    output_pose->pose.position.y = output_mat(1, 3);
    output_pose->pose.position.z = output_mat(2, 3);

    const Eigen::Quaterniond quat(Eigen::Matrix3d(output_mat.block<3, 3>(0, 0)));
    output_pose->pose.orientation.x = quat.x();
    output_pose->pose.orientation.y = quat.y();
    output_pose->pose.orientation.z = quat.z();
    output_pose->pose.orientation.w = quat.w();

    setOutput("output_pose", output_pose);

    return BT::NodeStatus::SUCCESS;
}

}  // namespace ros_cpp_behavior_util
