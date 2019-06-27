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

#ifndef TWIST_MUX__TOPIC_HANDLE_H_
#define TWIST_MUX__TOPIC_HANDLE_H_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <twist_mux/utils.h>
#include <twist_mux/twist_mux.h>

#include <string>
#include <vector>

namespace twist_mux
{

template<typename T>
class TopicHandle_
{
public:

  // Not copy constructible
  TopicHandle_(TopicHandle_&)                  = delete;
  TopicHandle_(const TopicHandle_&)            = delete;

  // Not copy assignable
  TopicHandle_& operator=(TopicHandle_&)       = delete;
  TopicHandle_& operator=(const TopicHandle_&) = delete;

  typedef int priority_type;

  /**
   * @brief TopicHandle_
   * @param nh Node handle
   * @param name Name identifier
   * @param topic Topic name
   * @param timeout Timeout to consider that the messages are old; note
   * that initially the message stamp is set to 0.0, so the message has
   * expired
   * @param priority Priority of the topic
   */
  TopicHandle_(const std::string& name, const std::string& topic,
               const rclcpp::Duration& timeout,
               priority_type priority, TwistMux* mux)
    : name_(name)
    , topic_(topic)
    , timeout_(timeout)
    , priority_(clamp(priority, priority_type(0), priority_type(255)))
    , mux_(mux)
    , stamp_(0)
  {
    RCLCPP_INFO(mux_->get_logger(),
                "Topic handler '%s' subscribed to topic '%s' "
                ": timeout = %s , priority = %d.",
                name_.c_str(), topic_.c_str(),
                ((timeout_.seconds()>0) ? std::to_string(timeout_.seconds()) + "s" : "None").c_str(),
                static_cast<int>(priority_));
  }

  virtual ~TopicHandle_() = default;

  /**
   * @brief hasExpired
   * @return true if the message has expired; false otherwise.
   *         If the timeout is set to 0.0, this function always returns
   *         false
   */
  bool hasExpired() const
  {
    return (timeout_.nanoseconds() > 0.0) /*and
           ((mux_->now() - stamp_) > timeout_)*/;
  }

  const std::string& getName() const
  {
    return name_;
  }

  const std::string& getTopic() const
  {
    return topic_;
  }

  const rclcpp::Duration& getTimeout() const
  {
    return timeout_;
  }

  /**
   * @brief getPriority Priority getter
   * @return Priority
   */
  const priority_type& getPriority() const
  {
    return priority_;
  }

  const T& getStamp() const
  {
    return stamp_;
  }

  const T& getMessage() const
  {
    return msg_;
  }

protected:

  std::string name_;
  std::string topic_;
  typename rclcpp::Subscription<T>::SharedPtr subscriber_;
  rclcpp::Duration timeout_;
  priority_type priority_;

protected:

  /// @todo replace by weak_ptr?
  TwistMux* mux_;

  rclcpp::Time stamp_;
  T msg_;
};

class VelocityTopicHandle : public TopicHandle_<geometry_msgs::msg::Twist>
{
private:
  typedef TopicHandle_<geometry_msgs::msg::Twist> base_type;

  // https://index.ros.org/doc/ros2/About-Quality-of-Service-Settings
  rmw_qos_profile_t twist_qos_profile = rmw_qos_profile_sensor_data;

public:
  typedef typename base_type::priority_type priority_type;

  VelocityTopicHandle(const std::string& name, const std::string& topic,
                      const rclcpp::Duration& timeout,
                      priority_type priority, TwistMux* mux)
    : base_type(name, topic, timeout, priority, mux)
  {
    /// @todo Deprecated
    subscriber_ = mux_->create_subscription<geometry_msgs::msg::Twist>
                         (topic_,
                          std::bind(&VelocityTopicHandle::callback, this, std::placeholders::_1),
                          twist_qos_profile);

//    subscriber_ = nh_.create_subscription<geometry_msgs::msg::Twist>
//                         (topic_, twist_qos_profile,
//                          std::bind(&VelocityTopicHandle::callback, this, std::placeholders::_1));
  }

  bool isMasked(priority_type lock_priority) const
  {
    return hasExpired() or (getPriority() < lock_priority);
  }

  void callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg)
  {
    stamp_ = mux_->now();
    msg_   = *msg;

    // Check if this twist has priority.
    // Note that we have to check all the locks because they might time out
    // and since we have several topics we must look for the highest one in
    // all the topic list; so far there's no O(1) solution.
    if (mux_->hasPriority(*this))
    {
      mux_->publishTwist(msg);
    }
  }
};

class LockTopicHandle : public TopicHandle_<std_msgs::msg::Bool>
{
private:
  typedef TopicHandle_<std_msgs::msg::Bool> base_type;

  // https://index.ros.org/doc/ros2/About-Quality-of-Service-Settings
  rmw_qos_profile_t lock_qos_profile = rmw_qos_profile_sensor_data;

public:
  typedef typename base_type::priority_type priority_type;

  LockTopicHandle(const std::string& name, const std::string& topic,
                  const rclcpp::Duration& timeout,
                  priority_type priority, TwistMux* mux)
    : base_type(name, topic, timeout, priority, mux)
  {
    /// @todo Deprecated
    subscriber_ = mux_->create_subscription<std_msgs::msg::Bool>
                         (topic_,
                          std::bind(&LockTopicHandle::callback, this, std::placeholders::_1),
                          lock_qos_profile);
  }

  /**
   * @brief isLocked
   * @return true if has expired or locked (i.e. bool message data is true)
   */
  bool isLocked() const
  {
    return hasExpired() or getMessage().data;
  }

  void callback(const std_msgs::msg::Bool::ConstSharedPtr msg)
  {
    stamp_ = mux_->now();
    msg_   = *msg;
  }
};

} // namespace twist_mux

#endif // TWIST_MUX__TOPIC_HANDLE_H_
