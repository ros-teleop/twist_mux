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

#ifndef TWIST_MUX__TWIST_MUX_DIAGNOSTICS_H_
#define TWIST_MUX__TWIST_MUX_DIAGNOSTICS_H_

#include <twist_mux/twist_mux_diagnostics_status.h>

#include <diagnostic_updater/diagnostic_updater.hpp>

#include <memory>

namespace twist_mux
{
class TwistMuxDiagnostics
{
public:
  typedef TwistMuxDiagnosticsStatus status_type;

  static constexpr double MAIN_LOOP_TIME_MIN = 0.2;   // [s]
  static constexpr double READING_AGE_MIN = 3.0;     // [s]

  explicit TwistMuxDiagnostics(TwistMux * mux);
  virtual ~TwistMuxDiagnostics() = default;

  void diagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);

  void update();

  void updateStatus(const status_type::ConstPtr & status);

private:
  /**
   * @brief Levels
   */
  enum
  {
    OK = diagnostic_msgs::msg::DiagnosticStatus::OK,
    WARN = diagnostic_msgs::msg::DiagnosticStatus::WARN,
    ERROR = diagnostic_msgs::msg::DiagnosticStatus::ERROR
  };

  std::shared_ptr<diagnostic_updater::Updater> diagnostic_;
  std::shared_ptr<status_type> status_;
};
}  // namespace twist_mux

#endif  // TWIST_MUX__TWIST_MUX_DIAGNOSTICS_H_
