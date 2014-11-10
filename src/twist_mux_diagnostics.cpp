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
 */

#include <twist_mux/twist_mux_diagnostics.h>
#include <twist_mux/twist_mux_diagnostics_status.h>

#include <diagnostic_updater/diagnostic_updater.h>

namespace twist_mux
{

TwistMuxDiagnostics::TwistMuxDiagnostics()
{
  diagnostic_.add("Twist mux status", this, &TwistMuxDiagnostics::diagnostics);
  diagnostic_.setHardwareID("none");
}

TwistMuxDiagnostics::~TwistMuxDiagnostics()
{}

void TwistMuxDiagnostics::update()
{
  diagnostic_.update();
}

void TwistMuxDiagnostics::updateStatus(const status_type::ConstPtr& status)
{
  ROS_DEBUG_THROTTLE(1.0, "Updating status.");

  status_.velocity_hs = status->velocity_hs;
  status_.lock_hs     = status->lock_hs;
  status_.priority    = status->priority;

  status_.main_loop_time = status->main_loop_time;
  status_.reading_age    = status->reading_age;

  update();
}

void TwistMuxDiagnostics::diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  /// Check if the loop period is quick enough
  if (status_.main_loop_time > MAIN_LOOP_TIME_MIN)
    stat.summary(ERROR, "loop time too long");
  else if (status_.reading_age > READING_AGE_MIN)
    stat.summary(ERROR, "data received is too old");
  else
    stat.summary(OK, "ok");

  for (const auto& velocity_h : *status_.velocity_hs)
  {
    stat.addf("velocity " + velocity_h.getName(),
              " %s (listening to %s @ %fs with priority #%d)",
              (velocity_h.isMasked(status_.priority) ? "masked" : "unmasked"),
              velocity_h.getTopic().c_str(),
              velocity_h.getTimeout(),
              static_cast<int>(velocity_h.getPriority()));
  }

  for (const auto& lock_h : *status_.lock_hs)
  {
    stat.addf("lock " + lock_h.getName(),
              " %s (listening to %s @ %fs with priority #%d)",
              (lock_h.isLocked() ? "locked" : "free"),
              lock_h.getTopic().c_str(),
              lock_h.getTimeout(),
              static_cast<int>(lock_h.getPriority()));
  }

  stat.add("current priority", static_cast<int>(status_.priority));

  stat.add("loop time in [sec]", status_.main_loop_time);
  stat.add("data age in [sec]", status_.reading_age);

  ROS_DEBUG_THROTTLE(1.0, "Publishing diagnostics.");
}

} // namespace twist_mux
