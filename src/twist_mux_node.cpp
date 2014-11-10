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
 */

#include <ros/ros.h>
#include <twist_mux/twist_mux.h>

int
main(int argc, char *argv[])
{
  ros::init(argc, argv, "twist_mux");

  twist_mux::TwistMux mux;

  while (ros::ok())
  {
    ros::spin();
  }

  return EXIT_SUCCESS;
}

