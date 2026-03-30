////////////////////////////////////////////////////////////////////////////////////////////
//      Author  : Crasun Jans
////////////////////////////////////////////////////////////////////////////////////////////
#ifndef ROS_NODE_BASE_HPP_
#define ROS_NODE_BASE_HPP_

#include <string>
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>

namespace ros_cpp_behavior_util
{

/**
 * @brief Template mixin base for BehaviorTree.CPP action nodes that require an
 *        rclcpp::Node shared pointer on the blackboard under the key "node".
 *
 * Provides:
 *   - A protected @c node_ member to be populated by derived classes via @c getNode().
 *   - A @c getNode() helper that reads from the blackboard and throws
 *     BT::RuntimeError if the key is absent or null.
 *
 * Usage:
 * @code
 *   class MyStateful : public RosStatefulActionBase { ... };
 *   class MySync     : public RosSyncActionBase     { ... };
 * @endcode
 */
template<typename BtBase>
class RosNodeBase : public BtBase
{
public:
    RosNodeBase(const std::string& name, const BT::NodeConfig& config)
        : BtBase(name, config)
    {}

protected:
    rclcpp::Node::SharedPtr node_;

    /// Reads the shared node from the blackboard.
    /// Throws BT::RuntimeError if the "node" key is missing or null.
    [[nodiscard]] rclcpp::Node::SharedPtr getNode() const
    {
        rclcpp::Node::SharedPtr node;
        if (!this->config().blackboard->get("node", node) || !node)
        {
            throw BT::RuntimeError(
                "[" + this->name() + "] \"node\" not found on blackboard");
        }
        return node;
    }
};

using RosStatefulActionBase = RosNodeBase<BT::StatefulActionNode>;
using RosSyncActionBase     = RosNodeBase<BT::SyncActionNode>;

}  // namespace ros_cpp_behavior_util

#endif  // ROS_NODE_BASE_HPP_
