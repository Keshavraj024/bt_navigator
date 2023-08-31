/**
 * @file init_pose_publisher.h
 * @brief This file defines the InitPosePublisher class, which is responsible for publishing initial pose to navigation stack.
 */

#ifndef __INIT_POSE_PUBLISHER_H__
#define __INIT_POSE_PUBLISHER_H__

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <chrono>

/**
 * @class InitPosePublisher
 * @brief A class for publishing initial pose information.
 */
class InitPosePublisher : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for the InitPosePublisher class.
     */
    explicit InitPosePublisher();

    /**
     * @brief Wait for subscribers to the initial pose publisher.
     *
     * This method waits until subscribers are connected to the initial pose publisher or a maximum waiting time is exceeded.
     * If no subscribers are found within the specified time, the node will shut down.
     *
     * @note This method is blocking and should be called during the initialization of the node.
     */
    void waitForSubscribers();
    
    /**
     * @brief Set the initial pose parameters.
     */
    void setInitPose();

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_init_pose_publisher; /**< Publisher for initial pose information. */
    double m_x;                                                                                        /**< Initial x-coordinate. */
    double m_y;                                                                                        /**< Initial y-coordinate. */
    double m_theta;                                                                                    /**< Initial orientation angle. */
    double m_covariance;                                                                               /**< Covariance value for pose uncertainty. */
    std::chrono::steady_clock::time_point m_startTime = std::chrono::steady_clock::now();              /**< Time at which the node was started. */
    int64_t maxWaitingTime{10};                                                                        /**< Maximum waiting time in seconds. */
};

#endif