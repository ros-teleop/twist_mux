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

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>

class TwistMarker
{
public:

  TwistMarker(double scale = 1.0, double z = 0.0, const std::string& frame_id = "base_footprint")
    : frame_id_(frame_id)
    , scale_(scale)
    , z_(z)
  {
    // ID and type:
    marker_.id = 0;
    marker_.type = visualization_msgs::Marker::ARROW;

    // Frame ID:
    marker_.header.frame_id = frame_id_;

    // Pre-allocate points for setting the arrow with the twist:
    marker_.points.resize(2);

    // Vertical position:
    marker_.pose.position.z = z_;

    // Scale:
    marker_.scale.x = 0.05 * scale_;
    marker_.scale.y = 2 * marker_.scale.x;

    // Color:
    marker_.color.a = 1.0;
    marker_.color.r = 0.0;
    marker_.color.g = 1.0;
    marker_.color.b = 0.0;
  }

  void update(const geometry_msgs::Twist& twist)
  {
    marker_.points[1].x = twist.linear.x;

    if (fabs(twist.linear.y) > fabs(twist.angular.z))
    {
      marker_.points[1].y = twist.linear.y;
    }
    else
    {
      marker_.points[1].y = twist.angular.z;
    }
  }

  const visualization_msgs::Marker& getMarker()
  {
    return marker_;
  }

private:
  visualization_msgs::Marker marker_;

  std::string frame_id_;
  double scale_;
  double z_;
};

class TwistMarkerPublisher
{
public:

  TwistMarkerPublisher(double scale = 1.0, double z = 0.0)
    : marker_(scale, z)
  {
    ros::NodeHandle nh;

    pub_ = nh.advertise<visualization_msgs::Marker>("marker", 1, true);
    sub_ = nh.subscribe("twist", 1, &TwistMarkerPublisher::callback, this);
  }

  void callback(const geometry_msgs::TwistConstPtr& twist)
  {
    marker_.update(*twist);

    pub_.publish(marker_.getMarker());
  }

private:
  ros::Subscriber sub_;
  ros::Publisher  pub_;

  TwistMarker marker_;
};

int
main(int argc, char *argv[])
{
  ros::init(argc, argv, "twist_marker");

  TwistMarkerPublisher t(1.0, 2.0);

  while (ros::ok())
  {
    ros::spin();
  }

  return EXIT_SUCCESS;
}

