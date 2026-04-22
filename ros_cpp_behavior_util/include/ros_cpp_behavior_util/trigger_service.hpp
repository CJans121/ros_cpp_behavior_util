////////////////////////////////////////////////////////////////////////////////////////////
//      Author  : Crasun Jans
////////////////////////////////////////////////////////////////////////////////////////////
#ifndef TRIGGER_SERVICE_HPP_
#define TRIGGER_SERVICE_HPP_

#include <chrono>
#include <optional>
#include <string>
#include <variant>
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "ros_cpp_behavior_util/ros_node_base.hpp"

namespace ros_cpp_behavior_util
{

/**
 * @brief BehaviorTree action node that calls a ROS 2 service of type
 *        std_srvs/Trigger or std_srvs/Empty and returns SUCCESS once the
 *        server responds (and, for Trigger, reports success).
 *
 * Blackboard keys
 * ---------------
 *   "node"  –  rclcpp::Node::SharedPtr   (read, mandatory)
 *
 * Input ports
 * -----------
 *   name       – Name of the service to call
 *   timeout    – Seconds to wait for a response before failing (default: 2.0)
 *   srv_type   – "Trigger" (default) or "Empty"
 */
class TriggerService : public RosStatefulActionBase
{
public:
    TriggerService(const std::string& name, const BT::NodeConfig& config)
        : RosStatefulActionBase(name, config)
    {}

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart()   override;
    BT::NodeStatus onRunning() override;
    void           onHalted()  override;

private:
    // ── type aliases ──────────────────────────────────────────────────────────
    template<typename SrvT>
    struct FuturePair
    {
        std::shared_ptr<rclcpp::Client<SrvT>>                client;
        std::optional<typename rclcpp::Client<SrvT>::FutureAndRequestId> future;
    };
    using TriggerClientPtr  = rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr;
    using EmptyClientPtr    = rclcpp::Client<std_srvs::srv::Empty>::SharedPtr;
    using TriggerFuturePair = FuturePair<std_srvs::srv::Trigger>;
    using EmptyFuturePair   = FuturePair<std_srvs::srv::Empty>;
    using ClientVariant     = std::variant<std::monostate, TriggerClientPtr, EmptyClientPtr>;
    using FutureVariant     = std::variant<std::monostate, TriggerFuturePair, EmptyFuturePair>;

    // ── state ─────────────────────────────────────────────────────────────────
    std::string               service_name_;
    ClientVariant             client_;
    FutureVariant             future_;
    rclcpp::Time              service_call_time_;
    std::chrono::milliseconds timeout_{2000};

    static constexpr std::chrono::seconds      wait_for_service_timeout_{2};
    static constexpr std::chrono::milliseconds spin_future_duration_{5};

    // ── helpers ───────────────────────────────────────────────────────────────
    void                     reset();
    void                     cleanup();
    bool                     dispatchRequest();
    rclcpp::FutureReturnCode spinFuture();
    BT::NodeStatus           handleSuccess();
};

}  // namespace ros_cpp_behavior_util

#endif  // TRIGGER_SERVICE_HPP_
