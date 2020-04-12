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
 * @author Jeremie Deray
 * @author Brighten Lee
 */

#ifndef TWIST_MUX__TWIST_MUX_DIAGNOSTICS_STATUS_H_
#define TWIST_MUX__TWIST_MUX_DIAGNOSTICS_STATUS_H_

#include <twist_mux/twist_mux.h>
#include <twist_mux/topic_handle.h>

#include <rclcpp/rclcpp.hpp>

namespace twist_mux
{
struct TwistMuxDiagnosticsStatus
{
  typedef std::shared_ptr<TwistMuxDiagnosticsStatus> Ptr;
  typedef std::shared_ptr<const TwistMuxDiagnosticsStatus> ConstPtr;

  double reading_age;
  rclcpp::Time last_loop_update;
  double main_loop_time;

  LockTopicHandle::priority_type priority;

  std::shared_ptr<TwistMux::velocity_topic_container> velocity_hs;
  std::shared_ptr<TwistMux::lock_topic_container> lock_hs;

  TwistMuxDiagnosticsStatus()
    : reading_age(0)
    , last_loop_update(rclcpp::Clock().now())
    , main_loop_time(0)
    , priority(0)
  {
    velocity_hs = std::make_shared<TwistMux::velocity_topic_container>();
    lock_hs = std::make_shared<TwistMux::lock_topic_container>();
  }
};

typedef TwistMuxDiagnosticsStatus::Ptr TwistMuxDiagnosticsStatusPtr;
typedef TwistMuxDiagnosticsStatus::ConstPtr TwistMuxDiagnosticsStatusConstPtr;

}  // namespace twist_mux

#endif  // TWIST_MUX__TWIST_MUX_DIAGNOSTICS_STATUS_H_