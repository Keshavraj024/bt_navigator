#include "bt_navigator/bt_wrapper.h"

using namespace std::chrono_literals;

BtWrapper::BtWrapper() : Node("bt_wrapper")
{
    this->declare_parameter("file_location", "none");
}

void BtWrapper::setup()
{
    BtWrapper::createBehaviourTree();
    const auto timeInterval = 500ms;
    m_timer = this->create_wall_timer(timeInterval, std::bind(&BtWrapper::updateBehaviourTree, this));
}
void BtWrapper::createBehaviourTree()
{
    BT::BehaviorTreeFactory factory;

    const std::string bt_xml_dir =
        ament_index_cpp::get_package_share_directory("bt_navigator") + "/bt_xml";

    BT::NodeBuilder builder =
        [this](const std::string &name, const BT::NodeConfiguration &config)
    {
        return std::make_unique<GoToGoal>(name, config, shared_from_this());
    };

    factory.registerBuilder<GoToGoal>("GoToGoal", builder);
    m_tree = factory.createTreeFromFile(bt_xml_dir + "/bt_tree.xml");
    RCLCPP_INFO(this->get_logger(), "Tree is created ");
}
void BtWrapper::updateBehaviourTree()
{
    BT::NodeStatus treeStatus = m_tree.tickExactlyOnce();

    if (treeStatus == BT::NodeStatus::SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(), "Navigation completed Successfully");
    }
    else if (treeStatus == BT::NodeStatus::RUNNING)
    {
        return;
    }
    else if (treeStatus == BT::NodeStatus::FAILURE)
    {
        RCLCPP_INFO(this->get_logger(), "Navigation Failed");
        m_timer->cancel();
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BtWrapper>();
    node->setup();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}