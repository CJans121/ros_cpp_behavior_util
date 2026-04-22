#include "ros_cpp_behavior_util/trigger_service.hpp"

namespace ros_cpp_behavior_util
{

BT::PortsList TriggerService::providedPorts()
{
    return {
        BT::InputPort<std::string>("service_name",     "Name of the service to call"),
        BT::InputPort<double>     ("timeout_secs",  2.0, "Timeout in seconds to await response"),
        BT::InputPort<std::string>("srv_type", "Service type: \"Trigger\" (default) or \"Empty\""),
    };
}

// ── private helpers ───────────────────────────────────────────────────────────

void TriggerService::reset()
{
    client_ = std::monostate{};
    future_ = std::monostate{};
}

void TriggerService::cleanup()
{
    std::visit([](auto& pair) {
        using T = std::decay_t<decltype(pair)>;
        if constexpr (!std::is_same_v<T, std::monostate>)
        {
            if (pair.future.has_value())
            {
                pair.client->remove_pending_request(pair.future.value());
                pair.future = std::nullopt;
            }
        }
    }, future_);
    reset();
}

bool TriggerService::dispatchRequest()
{
    if (std::holds_alternative<TriggerClientPtr>(client_))
    {
        auto& c = std::get<TriggerClientPtr>(client_);
        future_ = TriggerFuturePair{c, c->async_send_request(
            std::make_shared<std_srvs::srv::Trigger::Request>())};
    }
    else if (std::holds_alternative<EmptyClientPtr>(client_))
    {
        auto& c = std::get<EmptyClientPtr>(client_);
        future_ = EmptyFuturePair{c, c->async_send_request(
            std::make_shared<std_srvs::srv::Empty::Request>())};
    }
    else
    {
        RCLCPP_ERROR(node_->get_logger(), "dispatchRequest called with no active client");
        return false;
    }
    return true;
}

rclcpp::FutureReturnCode TriggerService::spinFuture()
{
    constexpr auto spin_duration = spin_future_duration_;
    return std::visit([&](auto& pair) -> rclcpp::FutureReturnCode {
        using T = std::decay_t<decltype(pair)>;
        if constexpr (std::is_same_v<T, std::monostate>)
        {
            RCLCPP_ERROR(node_->get_logger(), "Invalid state: no active future");
            return rclcpp::FutureReturnCode::INTERRUPTED;
        }
        else
        {
            if (!pair.future.has_value())
            {
                RCLCPP_ERROR(node_->get_logger(), "Invalid state: empty future");
                return rclcpp::FutureReturnCode::INTERRUPTED;
            }
            return rclcpp::spin_until_future_complete(
                node_, pair.future->future, spin_duration);
        }
    }, future_);
}

BT::NodeStatus TriggerService::handleSuccess()
{
    // Extract the Trigger response (if applicable) before resetting state.
    bool        trigger_failed = false;
    std::string trigger_message;
    if (std::holds_alternative<TriggerFuturePair>(future_))
    {
        const auto resp =
            std::get<TriggerFuturePair>(future_).future->future.get();
        if (!resp->success)
        {
            trigger_failed  = true;
            trigger_message = resp->message;
        }
    }
    reset();

    if (trigger_failed)
    {
        RCLCPP_ERROR(node_->get_logger(),
            "Service \"%s\" returned failure: %s",
            service_name_.c_str(), trigger_message.c_str());
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(node_->get_logger(),
        "Service \"%s\" completed successfully", service_name_.c_str());
    return BT::NodeStatus::SUCCESS;
}

// ── lifecycle ─────────────────────────────────────────────────────────────────

BT::NodeStatus TriggerService::onStart()
{
    reset();

    const auto service_name = getInput<std::string>("service_name");
    if (!service_name)
    {
        throw BT::RuntimeError(
            "[TriggerService] missing required input [name]: " + service_name.error());
    }
    service_name_ = *service_name;

    if (const auto t = getInput<double>("timeout_secs"))
    {
        timeout_ = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(*t));
    }

    node_ = getNode();

    const auto srv_type = getInput<std::string>("srv_type");
    if (!srv_type || srv_type.value() == "Trigger")
    {
        client_ = node_->create_client<std_srvs::srv::Trigger>(service_name_);
    }
    else if (srv_type.value() == "Empty")
    {
        client_ = node_->create_client<std_srvs::srv::Empty>(service_name_);
    }
    else
    {
        throw BT::RuntimeError(
            "[TriggerService] invalid [srv_type] \"" + srv_type.value() +
            "\", expected \"Trigger\" or \"Empty\"");
    }

    rclcpp::spin_some(node_);
    const bool server_found = std::visit([](const auto& c) -> bool {
        if constexpr (std::is_same_v<std::decay_t<decltype(c)>, std::monostate>)
            return false;
        else
            return c->wait_for_service(wait_for_service_timeout_);
    }, client_);

    if (!server_found)
    {
        RCLCPP_ERROR(node_->get_logger(),
            "Service server \"%s\" not found, aborting", service_name_.c_str());
        return BT::NodeStatus::FAILURE;
    }

    service_call_time_ = node_->now();
    if (!dispatchRequest())
    {
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(node_->get_logger(), "Request sent to \"%s\"", service_name_.c_str());
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TriggerService::onRunning()
{
    const rclcpp::Duration elapsed = node_->now() - service_call_time_;
    if (elapsed > rclcpp::Duration(timeout_))
    {
        RCLCPP_ERROR(node_->get_logger(),
            "Timed out waiting for service \"%s\"", service_name_.c_str());
        cleanup();
        return BT::NodeStatus::FAILURE;
    }

    switch (spinFuture())
    {
        case rclcpp::FutureReturnCode::TIMEOUT:
            return BT::NodeStatus::RUNNING;

        case rclcpp::FutureReturnCode::INTERRUPTED:
            RCLCPP_ERROR(node_->get_logger(),
                "Service \"%s\" interrupted", service_name_.c_str());
            cleanup();
            return BT::NodeStatus::FAILURE;

        case rclcpp::FutureReturnCode::SUCCESS:
            return handleSuccess();
    }

    return BT::NodeStatus::FAILURE;  // unreachable; satisfies -Wreturn-type
}

void TriggerService::onHalted()
{
    cleanup();
}

}  // namespace ros_cpp_behavior_util
