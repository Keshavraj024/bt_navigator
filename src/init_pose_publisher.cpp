#include "bt_navigator/init_pose_publisher.h"
#include <vector>
#include <thread>

using namespace std::chrono_literals;

InitPosePublisher::InitPosePublisher() : Node("init_pose_publisher")
{

    this->declare_parameter("intial_pose_x", -2.0);
    this->declare_parameter("intial_pose_y", -0.5);
    this->declare_parameter("intial_pose_theta", 0.01);
    this->declare_parameter("intial_covariance", 0.5 * 0.5);

    m_x = this->get_parameter("intial_pose_x").as_double();
    m_y = this->get_parameter("intial_pose_y").as_double();
    m_theta = this->get_parameter("intial_pose_theta").as_double();
    m_covariance = this->get_parameter("intial_covariance").as_double();

    m_init_pose_publisher = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

    waitForSubscribers();
}

void InitPosePublisher::waitForSubscribers()
{
    std::chrono::seconds elapsedTime{};

    while (rclcpp::ok() && m_init_pose_publisher->get_subscription_count() == 0)
    {
        auto currentTime = std::chrono::steady_clock::now();
        RCLCPP_INFO(this->get_logger(), "No subscribers Found");
        elapsedTime = std::chrono::duration_cast<std::chrono::seconds>(currentTime - m_startTime);
        RCLCPP_INFO(this->get_logger(), "Elapsed Time: %ld seconds", elapsedTime.count());

        if (elapsedTime.count() > maxWaitingTime)
        {
            RCLCPP_INFO(this->get_logger(), "Exiting due to maximum waiting time exceeded.");
            rclcpp::shutdown();
            break;
        }

        std::this_thread::sleep_for(1s);
    }
}

void InitPosePublisher::setInitPose()
{
    geometry_msgs::msg::PoseWithCovarianceStamped msg{};
    msg.header.frame_id = "map";
    msg.pose.pose.position.x = m_x;
    msg.pose.pose.position.y = m_y;

    tf2::Quaternion q;
    q.setRPY(0, 0, m_theta);

    msg.pose.pose.orientation.x = q.getX();
    msg.pose.pose.orientation.y = q.getY();
    msg.pose.pose.orientation.z = q.getZ();
    msg.pose.pose.orientation.w = q.getW();

    msg.pose.covariance = {
        m_covariance, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, m_covariance, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, m_covariance};

    m_init_pose_publisher->publish(msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InitPosePublisher>();
    node->setInitPose();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}