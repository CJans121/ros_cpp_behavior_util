////////////////////////////////////////////////////////////////////////////////////////////
//      Author  : Crasun Jans
////////////////////////////////////////////////////////////////////////////////////////////
#ifndef TRIGGER_SERVICE_HPP
#define TRIGGER_SERVICE_HPP

#include <chrono>
#include <optional>
#include <string>
#include <variant>

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace ros_cpp_behavior_util {

class TriggerService : public BT::StatefulActionNode {
public:
    TriggerService(const std::string& name, const BT::NodeConfiguration& config)
        : BT::StatefulActionNode(name, config)
    {}

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart()   override;
    BT::NodeStatus onRunning() override;
    void           onHalted()  override;

private:
    // ── type aliases ─────────────────────────────────────────────────────────
    using TriggerClientPtr = rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr;
    using EmptyClientPtr   = rclcpp::Client<std_srvs::srv::Empty>::SharedPtr;

    template<typename ClientT, typename SrvT>
    struct FuturePair {
        std::shared_ptr<rclcpp::Client<SrvT>> client;
        std::optional<typename rclcpp::Client<SrvT>::SharedFuture> future;
    };
    using TriggerFuturePair = FuturePair<TriggerClientPtr, std_srvs::srv::Trigger>;
    using EmptyFuturePair   = FuturePair<EmptyClientPtr,   std_srvs::srv::Empty>;

    using ClientVariant = std::variant<std::monostate, TriggerClientPtr, EmptyClientPtr>;
    using FutureVariant = std::variant<std::monostate, TriggerFuturePair, EmptyFuturePair>;

    // ── state ─────────────────────────────────────────────────────────────────
    ClientVariant              client_;
    FutureVariant              future_;
    rclcpp::Time               service_call_time_;
    std::chrono::milliseconds  timeout_{2000}; // Service response timeout
    std::chrono::seconds  wait_for_service_timeout_{2000}; // Timeout for waiting for service to appear
    std::chrono::milliseconds spin_future_duration_{5}; // How long to spin the node when awaiting service response

    // ── helpers ───────────────────────────────────────────────────────────────
    [[nodiscard]] rclcpp::Node::SharedPtr  getNode() const;
    [[nodiscard]] rclcpp::Logger           getLogger() const;
    [[nodiscard]] std::string              serviceName() const;
    void                                   reset();
    bool                                   dispatchRequest();
    rclcpp::FutureReturnCode               spinFuture(const rclcpp::Node::SharedPtr& node);
    BT::NodeStatus                         handleSuccess();
};

} // namespace ros_cpp_behavior_util

#endif // TRIGGER_SERVICE_HPP
