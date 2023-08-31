/**
 * @file bt_wrapper.h
 * @brief Defines the BtWrapper class that manages the Behavior Tree (BT) execution.
 */

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "bt_navigator/behaviour_tree_action_node.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

/**
 * @class BtWrapper
 * @brief Manages the execution of a Behavior Tree (BT) for navigation.
 *
 * The BtWrapper class provides functionality to set up, create, and update a Behavior Tree
 * for navigation tasks.
 */
class BtWrapper : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for the BtWrapper class.
     */
    explicit BtWrapper();

    /**
     * @brief Set up the node and Behavior Tree execution.
     */
    void setup();

    /**
     * @brief Create the Behavior Tree.
     */
    void createBehaviourTree();

    /**
     * @brief Update the Behavior Tree execution on certain interval.
     */
    void updateBehaviourTree();

private:
    rclcpp::TimerBase::SharedPtr m_timer; /**< Timer for periodic execution. */
    BT::Tree m_tree;                      /**< The Behavior Tree instance. */
};
