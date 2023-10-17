#include "bt_navigator/behaviour_tree_action_node.h"

GoToGoal::GoToGoal(const std::string &name, const BT::NodeConfiguration &config,
                   const rclcpp::Node::SharedPtr &node_ptr) : BT::StatefulActionNode(name, config), m_node_ptr(node_ptr)
{
    m_action_client_ptr = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(m_node_ptr, "/navigate_to_pose");
    const std::string file_location = m_node_ptr->get_parameter("file_location").as_string();
    RCLCPP_INFO(m_node_ptr->get_logger(), file_location.c_str());
    m_target_poses = YAML::LoadFile(file_location);
}

BT::PortsList GoToGoal::providedPorts()
{
    return {BT::InputPort<std::string>("pose")};
}

BT::NodeStatus GoToGoal::onStart()
{
    const auto poseKey = getInput<std::string>("pose");

    const std::vector<float> targetpose = m_target_poses[poseKey.value()].as<std::vector<float>>();

    auto sendGoalOptions = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    sendGoalOptions.result_callback = std::bind(&GoToGoal::navToPoseCallback, this, std::placeholders::_1);

    auto goalPose = nav2_msgs::action::NavigateToPose::Goal();
    goalPose.pose.header.frame_id = "map";
    goalPose.pose.pose.position.x = targetpose[0];
    goalPose.pose.pose.position.y = targetpose[1];

    tf2::Quaternion q;
    q.setRPY(0, 0, targetpose[2]);
    q.normalize();

    goalPose.pose.pose.orientation.x = q.getX();
    goalPose.pose.pose.orientation.y = q.getY();
    goalPose.pose.pose.orientation.z = q.getZ();
    goalPose.pose.pose.orientation.w = q.getW();

    m_goalFeedback = GoalFeedback::RUNNING;
    m_action_client_ptr->async_send_goal(goalPose, sendGoalOptions);
    RCLCPP_INFO(m_node_ptr->get_logger(), "Sent Goal to Nav2\n");
    return BT::NodeStatus::RUNNING;
}
BT::NodeStatus GoToGoal::onRunning()
{
    switch (m_goalFeedback)
    {
    case GoalFeedback::SUCCEEDED:
        return BT::NodeStatus::SUCCESS;
        break;
    case GoalFeedback::ABORTED:
        return BT::NodeStatus::FAILURE;
        break;
    case GoalFeedback::CANCELED:
        return BT::NodeStatus::FAILURE;
        break;
    case GoalFeedback::RUNNING:
        return BT::NodeStatus::RUNNING;
        break;
    default:
        return BT::NodeStatus::FAILURE;
        break;
    }
}

void GoToGoal::navToPoseCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result)
{
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(m_node_ptr->get_logger(), "Goal reached\n");
        m_goalFeedback = GoalFeedback::SUCCEEDED;
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(m_node_ptr->get_logger(), "Goal was aborted");
        m_goalFeedback = GoalFeedback::ABORTED;
        break;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(m_node_ptr->get_logger(), "Goal was canceled");
        m_goalFeedback = GoalFeedback::CANCELED;
        break;
    default:
        RCLCPP_ERROR(m_node_ptr->get_logger(), "Unknown result code");
        m_goalFeedback = GoalFeedback::UNKNOWN;
        break;
    }
}
