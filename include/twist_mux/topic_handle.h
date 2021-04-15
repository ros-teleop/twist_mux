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
 *
 * Modified by Joao Aguizo, 15th April 2021
*/

#ifndef TOPIC_HANDLE_H
#define TOPIC_HANDLE_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Twist.h>

#include <twist_mux/utils.h>
#include <twist_mux/twist_mux.h>

#include <boost/utility.hpp>
#include <boost/scoped_ptr.hpp>

#include <string>
#include <vector>

namespace twist_mux
{

template<typename T>
class TopicHandle_ : boost::noncopyable
{
public:

  typedef int priority_type;
  typedef int flag_type;

  /**
   * @brief TopicHandle_
   * @param nh Node handle
   * @param name Name identifier
   * @param topic Topic name
   * @param timeout Timeout to consider that the messages are old; note
   * that initially the message stamp is set to 0.0, so the message has
   * expired
   * @param priority Priority of the topic
   * @param flag Flags the type of the source
   */
  TopicHandle_(ros::NodeHandle& nh, const std::string& name, const std::string& topic, double timeout, priority_type priority, flag_type flag, TwistMux* mux)
    : nh_(nh)
    , name_(name)
    , topic_(topic)
    , timeout_(timeout)
    , priority_(clamp(priority, priority_type(0), priority_type(255)))
    , flag_(clamp(flag, flag_type(0), flag_type(255)))
    , mux_(mux)
    , stamp_(0.0)
  {
    ROS_INFO_STREAM
    (
      "Topic handler '" << name_ << "' subscribed to topic '" << topic_ <<
      "': timeout = " << ((timeout_) ? std::to_string(timeout_) + "s" : "None") <<
      ", priority = " << static_cast<int>(priority_) <<
      ", flag = " << static_cast<int>(flag_)
    );
    publisher_ = nh.advertise<std_msgs::UInt8>("flag", 1); // make topic name configurable?
  }

  virtual ~TopicHandle_()
  {
    subscriber_.shutdown();
    publisher_.shutdown();
  }

  /**
   * @brief hasExpired
   * @return true if the message has expired; false otherwise.
   *         If the timeout is set to 0.0, this function always returns
   *         false
   */
  bool hasExpired() const
  {
    return (timeout_ > 0.0) &&
           ((ros::Time::now() - stamp_).toSec() > timeout_);
  }

  const std::string& getName() const
  {
    return name_;
  }

  const std::string& getTopic() const
  {
    return topic_;
  }

  const double& getTimeout() const
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
  ros::NodeHandle nh_;

  std::string name_;
  std::string topic_;
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;
  double timeout_;
  priority_type priority_;
  flag_type flag_;

protected:
  TwistMux* mux_;

  ros::Time stamp_;
  T msg_;
  std_msgs::UInt8 flag_msg_;
};

class VelocityTopicHandle : public TopicHandle_<geometry_msgs::Twist>
{
private:
  typedef TopicHandle_<geometry_msgs::Twist> base_type;

public:
  typedef typename base_type::priority_type priority_type;
  typedef typename base_type::flag_type flag_type;

  VelocityTopicHandle(ros::NodeHandle& nh, const std::string& name, const std::string& topic, double timeout, priority_type priority, flag_type flag, TwistMux* mux)
    : base_type(nh, name, topic, timeout, priority, clamp(flag, flag_type(0), flag_type(127)), mux)
  {
    subscriber_ = nh_.subscribe(topic_, 1, &VelocityTopicHandle::callback, this);
  }

  bool isMasked(priority_type lock_priority) const
  {
    return hasExpired() || (getPriority() < lock_priority);
  }

  void callback(const geometry_msgs::TwistConstPtr& msg)
  {
    stamp_ = ros::Time::now();
    msg_   = *msg;

    // Check if this twist has priority.
    // Note that we have to check all the locks because they might time out
    // and since we have several topics we must look for the highest one in
    // all the topic list; so far there's no O(1) solution.
    if (mux_->hasPriority(*this))
    {
      mux_->publishTwist(msg);
      
      // Also publishes the topic flag
      flag_msg_.data = flag_;
      publisher_.publish(flag_msg_);
    }
  }
};

class LockTopicHandle : public TopicHandle_<std_msgs::Bool>
{
private:
  typedef TopicHandle_<std_msgs::Bool> base_type;

public:
  typedef typename base_type::priority_type priority_type;
  typedef typename base_type::flag_type flag_type;

  LockTopicHandle(ros::NodeHandle& nh, const std::string& name, const std::string& topic, double timeout, priority_type priority, flag_type flag, TwistMux* mux)
    : base_type(nh, name, topic, timeout, priority, clamp(flag, flag_type(128), flag_type(255)), mux)
  {
    subscriber_ = nh_.subscribe(topic_, 1, &LockTopicHandle::callback, this);
  }

  /**
   * @brief isLocked
   * @return true if has expired or locked (i.e. bool message data is true)
   */
  bool isLocked() const
  {
    return hasExpired() || getMessage().data;
  }

  void callback(const std_msgs::BoolConstPtr& msg)
  {
    stamp_         = ros::Time::now();
    msg_           = *msg;
    
    // Also publishes the topic flag
    flag_msg_.data = flag_;
    publisher_.publish(flag_msg_);
  }
};

} // namespace twist_mux

#endif // TOPIC_HANDLE_H
