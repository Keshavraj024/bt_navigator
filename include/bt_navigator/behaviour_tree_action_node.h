/**
 * @file behaviour_tree_action_node.h
 * @brief Defines the GoToGoal class which implements a Behavior Tree (BT) action node
 *        for navigating to the specified goal poses using ROS 2 Navigation stack.
 */

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/behavior_tree.h"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "yaml-cpp/yaml.h"
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <string>

/**
 * @class GoToGoal
 * @brief Implements a Behavior Tree (BT) action node for navigating to a specified goal pose.
 *
 * The GoToGoal class is a stateful action node that interfaces with the ROS 2 Navigation stack.
 * It uses an action client to submit a NavigateToPose goal and tracks its progress and result.
 * This class is intended to be used as a part of a Behavior Tree to enable navigation behavior.
 */
class GoToGoal : public BT::StatefulActionNode
{
public:
    /**
     * @brief Constructor for the GoToGoal class.
     * @param name The name of the behavior node.
     * @param config The configuration for the behavior node.
     * @param node_ptr A shared pointer to the ROS 2 node.
     */
    GoToGoal(const std::string &name, const BT::NodeConfiguration &config,
             const rclcpp::Node::SharedPtr &node_ptr);

    /**
     * @brief Returns the list of provided ports for this action node.
     * @return The list of provided ports.
     */
    static BT::PortsList providedPorts();

    /**
     * @brief Callback function executed when the behavior node starts.
     * @return The status of the behavior node.
     */
    BT::NodeStatus onStart() override;

    /**
     * @brief Callback function executed when the behavior node is running.
     * @return The status of the behavior node.
     */
    BT::NodeStatus onRunning() override;

    /**
     * @brief Callback function executed when the behavior node is halted.
     */
    void onHalted() override{};

    /**
     * @brief Callback function for handling the result of the NavigateToPose action.
     * @param result The wrapped result of the NavigateToPose action.
     */
    void navToPoseCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result);

private:
    rclcpp::Node::SharedPtr m_node_ptr;                                                      /**< Shared pointer to the ROS 2 node. */
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr m_action_client_ptr; /**< Action client for NavigateToPose. */
    bool m_done_flag;                                                                        /**< Flag indicating whether the navigation is complete. */
    YAML::Node m_target_poses{};                                                             /**<store target poses in a YAML format*/
};
