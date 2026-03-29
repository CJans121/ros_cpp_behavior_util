#include "ros_cpp_behavior_util/trigger_service.hpp"

namespace ros_cpp_behavior_util {

BT::PortsList TriggerService::providedPorts() {
    return {
        BT::InputPort<std::string>("name", "Name of the service to call"),
        BT::InputPort<float>("timeout", 2.0f, "Timeout in seconds to await response"),
        BT::InputPort<std::string>("srv_type", "Whether to use an Empty or Trigger type srv. Default is Trigger")
    };
}

// ── private helpers ───────────────────────────────────────────────────────────

rclcpp::Node::SharedPtr TriggerService::getNode() const {
    rclcpp::Node::SharedPtr node;
    if (!config().blackboard->get("node", node) || !node) {
        RCLCPP_ERROR(getLogger(), "\"node\" not found on blackboard");
        return nullptr;
    }
    return node;
}

rclcpp::Logger TriggerService::getLogger() const {
    rclcpp::Node::SharedPtr node;
    if (config().blackboard->get("node", node) && node) {
        return node->get_logger();
    }
    return rclcpp::get_logger("TriggerService");
}

std::string TriggerService::serviceName() const {
    return std::visit([](const auto& c) -> std::string {
        if constexpr (std::is_same_v<std::decay_t<decltype(c)>, std::monostate>) return "<none>";
        else return c->get_service_name();
    }, client_);
}

void TriggerService::reset() {
    client_ = std::monostate{};
    future_ = std::monostate{};
}

bool TriggerService::dispatchRequest() {
    if (std::holds_alternative<TriggerClientPtr>(client_)) {
        auto& c = std::get<TriggerClientPtr>(client_);
        future_ = TriggerFuturePair{c, c->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>())};
    } else if (std::holds_alternative<EmptyClientPtr>(client_)) {
        auto& c = std::get<EmptyClientPtr>(client_);
        future_ = EmptyFuturePair{c, c->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>())};
    } else {
        RCLCPP_ERROR(getLogger(), "dispatchRequest called with no active client");
        return false;
    }
    return true;
}

rclcpp::FutureReturnCode TriggerService::spinFuture(const rclcpp::Node::SharedPtr& node) {
    constexpr auto SPIN_DURATION = spin_future_duration_;
    return std::visit([&](auto& pair) -> rclcpp::FutureReturnCode {
        using T = std::decay_t<decltype(pair)>;
        if constexpr (std::is_same_v<T, std::monostate>) {
            RCLCPP_ERROR(getLogger(), "Invalid state: no active future");
            return rclcpp::FutureReturnCode::INTERRUPTED;
        } else {
            if (!pair.future.has_value()) {
                RCLCPP_ERROR(getLogger(), "Invalid state: empty future");
                return rclcpp::FutureReturnCode::INTERRUPTED;
            }
            return rclcpp::spin_until_future_complete(node, *pair.future, SPIN_DURATION);
        }
    }, future_);
}

BT::NodeStatus TriggerService::handleSuccess() {
    if (std::holds_alternative<TriggerFuturePair>(future_)) {
        auto& pair = std::get<TriggerFuturePair>(future_);
        const auto resp = pair.future->get();
        pair.future = std::nullopt;
        if (!resp->success) {
            RCLCPP_ERROR(getLogger(), "Service \"%s\" returned failure: %s",
                serviceName().c_str(), resp->message.c_str());
            return BT::NodeStatus::FAILURE;
        }
    } else {
        std::get<EmptyFuturePair>(future_).future = std::nullopt;
    }

    RCLCPP_INFO(getLogger(), "Service \"%s\" completed successfully", serviceName().c_str());
    reset();
    return BT::NodeStatus::SUCCESS;
}

// ── lifecycle ─────────────────────────────────────────────────────────────────

BT::NodeStatus TriggerService::onStart() {
    reset();

    // Get required port values
    const auto service_name = getInput<std::string>("name");
    if (!service_name) {
        RCLCPP_ERROR(getLogger(), "Could not read \"name\" port: %s", service_name.error().c_str());
        return BT::NodeStatus::FAILURE;
    }

    // Get optional port values
    const auto timeout_input = getInput<float>("timeout");
    if (timeout_input) {
        timeout_ = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<float>(timeout_input.value()));
    }

    const rclcpp::Node::SharedPtr node = getNode();
    if (!node) { return BT::NodeStatus::FAILURE; }

    const auto srv_type = getInput<std::string>("srv_type");
    if (!srv_type || srv_type.value() == "Trigger") {
        client_ = node->create_client<std_srvs::srv::Trigger>(service_name.value());
    }
    else if (srv_type.value() == "Empty") {
        client_ = node->create_client<std_srvs::srv::Empty>(service_name.value());
	}
    else {
	RCLCPP_ERROR(getLogger(), "Invalid \"srv_type\" port value \"%s\", expected \"Trigger\" or \"Empty\"",
	    srv_type.value().c_str());
	return BT::NodeStatus::FAILURE;
    }
    // EoF optional port value handling
    
    rclcpp::spin_some(node);
    const bool server_found = std::visit([](const auto& c) -> bool {
        if constexpr (std::is_same_v<std::decay_t<decltype(c)>, std::monostate>) return false;
        else return c->wait_for_service(wait_for_service_timeout_);
    }, client_);

    if (!server_found) {
        RCLCPP_ERROR(getLogger(), "Service server \"%s\" not found, aborting", service_name.value().c_str());
        return BT::NodeStatus::FAILURE;
    }

    service_call_time_ = node->now();
    if (!dispatchRequest()) {
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_INFO(getLogger(), "Request sent to \"%s\"", serviceName().c_str());
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TriggerService::onRunning() {
    const rclcpp::Node::SharedPtr node = getNode();
    if (!node) { return BT::NodeStatus::FAILURE; }

    const rclcpp::Duration elapsed = node->now() - service_call_time_;
    if (elapsed > rclcpp::Duration(timeout_)) {
        RCLCPP_ERROR(getLogger(), "Timed out waiting for service \"%s\"", serviceName().c_str());
        onHalted();
        return BT::NodeStatus::FAILURE;
    }

    switch (spinFuture(node)) {
        case rclcpp::FutureReturnCode::TIMEOUT:
            return BT::NodeStatus::RUNNING;

        case rclcpp::FutureReturnCode::INTERRUPTED:
            RCLCPP_ERROR(getLogger(), "Service \"%s\" interrupted", serviceName().c_str());
            onHalted();
            return BT::NodeStatus::FAILURE;

        case rclcpp::FutureReturnCode::SUCCESS:
            return handleSuccess();
    }

    return BT::NodeStatus::FAILURE;
}

void TriggerService::onHalted() {
    std::visit([](auto& pair) {
        using T = std::decay_t<decltype(pair)>;
        if constexpr (!std::is_same_v<T, std::monostate>) {
            if (pair.future.has_value()) {
                pair.client->remove_pending_request(*pair.future);
                pair.future = std::nullopt;
            }
        }
    }, future_);
    reset();
}

} // namespace ros_cpp_behavior_util
