/*********************************************************************
 * Software License Agreement (CC BY-NC-SA 4.0 License)
 *
 *  Copyright (c) 2014, PAL Robotics, S.L.
 *  All rights reserved.
 *
 *  This work is licensed under the Creative Commons
 *  Attribution-NonCommercial-ShareAlike 4.0 International License.
 *
 *  To view a copy of this license, visit
 *  http://creativecommons.org/licenses/by-nc-sa/4.0/
 *  or send a letter to
 *  Creative Commons, 444 Castro Street, Suite 900,
 *  Mountain View, California, 94041, USA.
 *********************************************************************/

/*
 * @author Enrique Fernandez
 * @author Siegfried Gevatter
 * @author Jeremie Deray
 */

#include <twist_mux/twist_mux.hpp>
#include <twist_mux/topic_handle.hpp>
#include <twist_mux/twist_mux_diagnostics.hpp>
#include <twist_mux/twist_mux_diagnostics_status.hpp>
#include <twist_mux/utils.hpp>
#include <twist_mux/params_helpers.hpp>

/**
 * @brief hasIncreasedAbsVelocity Check if the absolute velocity has increased
 * in any of the components: linear (abs(x)) or angular (abs(yaw))
 * @param old_twist Old velocity
 * @param new_twist New velocity
 * @return true is any of the absolute velocity components has increased
 */
bool hasIncreasedAbsVelocity(const geometry_msgs::msg::Twist& old_twist, const geometry_msgs::msg::Twist& new_twist)
{
  const auto old_linear_x = std::abs(old_twist.linear.x);
  const auto new_linear_x = std::abs(new_twist.linear.x);

  const auto old_angular_z = std::abs(old_twist.angular.z);
  const auto new_angular_z = std::abs(new_twist.angular.z);

  return (old_linear_x < new_linear_x) or (old_angular_z < new_angular_z);
}

namespace twist_mux
{
// see e.g. https://stackoverflow.com/a/40691657
constexpr std::chrono::duration<long int> TwistMux::DIAGNOSTICS_PERIOD;

TwistMux::TwistMux(int window_size)
  : Node("twist_mux", "",
         rclcpp::NodeOptions().allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true))
{
}

void TwistMux::init()
{
  /// Get topics and locks:
  velocity_hs_ = std::make_shared<velocity_topic_container>();
  lock_hs_ = std::make_shared<lock_topic_container>();
  getTopicHandles("topics", *velocity_hs_);
  getTopicHandles("locks", *lock_hs_);

  /// Publisher for output topic:
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel_out", rclcpp::QoS(rclcpp::KeepLast(1)));

  /// Diagnostics:
  diagnostics_ = std::make_shared<diagnostics_type>(this);
  status_ = std::make_shared<status_type>();
  status_->velocity_hs = velocity_hs_;
  status_->lock_hs = lock_hs_;

  diagnostics_timer_ = this->create_wall_timer(DIAGNOSTICS_PERIOD, [this]() -> void { updateDiagnostics(); });
}

void TwistMux::updateDiagnostics()
{
  status_->priority = getLockPriority();
  diagnostics_->updateStatus(status_);
}

void TwistMux::publishTwist(const geometry_msgs::msg::Twist::ConstSharedPtr& msg)
{
  cmd_pub_->publish(*msg);
}

template <typename T>
void TwistMux::getTopicHandles(const std::string& param_name, std::list<T>& topic_hs)
{
  RCLCPP_DEBUG(get_logger(), "getTopicHandles: %s", param_name.c_str());

  rcl_interfaces::msg::ListParametersResult list = list_parameters({ param_name }, 10);

  try
  {
    for (auto prefix : list.prefixes)
    {
      RCLCPP_DEBUG(get_logger(), "Prefix: %s", prefix.c_str());

      std::string topic;
      double timeout = 0;
      int priority = 0;

      auto nh = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*) {});

      fetch_param(nh, prefix + ".topic", topic);
      fetch_param(nh, prefix + ".timeout", timeout);
      fetch_param(nh, prefix + ".priority", priority);

      RCLCPP_DEBUG(get_logger(), "Retrieved topic: %s", topic.c_str());
      RCLCPP_DEBUG(get_logger(), "Listed prefix: %.2f", timeout);
      RCLCPP_DEBUG(get_logger(), "Listed prefix: %d", priority);

      topic_hs.emplace_back(prefix, topic, std::chrono::duration<double>(timeout), priority, this);
    }
  }
  catch (const ParamsHelperException& e)
  {
    RCLCPP_FATAL(get_logger(), "Error parsing params '%s':\n\t%s", param_name.c_str(), e.what());
    throw e;
  }
}

int TwistMux::getLockPriority()
{
  LockTopicHandle::priority_type priority = 0;

  /// max_element on the priority of lock topic handles satisfying
  /// that is locked:
  for (const auto& lock_h : *lock_hs_)
  {
    if (lock_h.isLocked())
    {
      auto tmp = lock_h.getPriority();
      if (priority < tmp)
      {
        priority = tmp;
      }
    }
  }

  RCLCPP_DEBUG(get_logger(), "Priority = %d.", static_cast<int>(priority));

  return priority;
}

bool TwistMux::hasPriority(const VelocityTopicHandle& twist)
{
  const auto lock_priority = getLockPriority();

  LockTopicHandle::priority_type priority = 0;
  std::string velocity_name = "NULL";

  /// max_element on the priority of velocity topic handles satisfying
  /// that is NOT masked by the lock priority:
  for (const auto& velocity_h : *velocity_hs_)
  {
    if (not velocity_h.isMasked(lock_priority))
    {
      const auto velocity_priority = velocity_h.getPriority();
      if (priority < velocity_priority)
      {
        priority = velocity_priority;
        velocity_name = velocity_h.getName();
      }
    }
  }

  return twist.getName() == velocity_name;
}

}  // namespace twist_mux