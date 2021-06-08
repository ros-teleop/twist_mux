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

#include <twist_mux/twist_mux.h>

#include <memory>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto twist_mux_node = std::make_shared<twist_mux::TwistMux>();

  twist_mux_node->init();

  rclcpp::spin(twist_mux_node);

  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
