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

#ifndef TWIST_MUX__TWIST_MUX_H_
#define TWIST_MUX__TWIST_MUX_H_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <list>
#include <memory>
#include <string>

using std::chrono_literals::operator""s;

namespace twist_mux
{
// Forwarding declarations:
class TwistMuxDiagnostics;
struct TwistMuxDiagnosticsStatus;
class VelocityTopicHandle;
class LockTopicHandle;

/**
 * @brief The TwistMux class implements a top-level twist multiplexer module
 * that priorize different velocity command topic inputs according to locks.
 */
class TwistMux : public rclcpp::Node
{
public:
  template<typename T>
  using handle_container = std::list<T>;

  using velocity_topic_container = handle_container<VelocityTopicHandle>;
  using lock_topic_container = handle_container<LockTopicHandle>;

  explicit TwistMux(int window_size = 10);
  ~TwistMux() = default;

  void init();

  bool hasPriority(const VelocityTopicHandle & twist);

  void publishTwist(const geometry_msgs::msg::Twist::ConstSharedPtr & msg);

  void updateDiagnostics();

protected:
  typedef TwistMuxDiagnostics diagnostics_type;
  typedef TwistMuxDiagnosticsStatus status_type;

  rclcpp::TimerBase::SharedPtr diagnostics_timer_;

  static constexpr std::chrono::duration<int64_t> DIAGNOSTICS_PERIOD = 1s;

  /**
   * @brief velocity_hs_ Velocity topics' handles.
   * Note that if we use a vector, as a consequence of the re-allocation and
   * the fact that we have a subscriber inside with a pointer to 'this', we
   * must reserve the number of handles initially.
   */
  std::shared_ptr<velocity_topic_container> velocity_hs_;
  std::shared_ptr<lock_topic_container> lock_hs_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

  geometry_msgs::msg::Twist last_cmd_;

  template<typename T>
  void getTopicHandles(const std::string & param_name, handle_container<T> & topic_hs);

  int getLockPriority();

  std::shared_ptr<diagnostics_type> diagnostics_;
  std::shared_ptr<status_type> status_;
};

}  // namespace twist_mux

#endif  // TWIST_MUX__TWIST_MUX_H_
