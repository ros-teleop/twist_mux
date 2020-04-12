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

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <string>

class TwistMarker
{
public:
  TwistMarker(std::string& frame_id, double scale, double z) : frame_id_(frame_id), scale_(scale), z_(z)
  {
    // ID and type:
    marker_.id = 0;
    marker_.type = visualization_msgs::msg::Marker::ARROW;

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

    // Error when all points are zero:
    marker_.points[1].z = 0.01;
  }

  void update(const geometry_msgs::msg::Twist& twist)
  {
    using std::abs;

    marker_.points[1].x = twist.linear.x;

    if (abs(twist.linear.y) > abs(twist.angular.z))
      marker_.points[1].y = twist.linear.y;
    else
      marker_.points[1].y = twist.angular.z;
  }

  const visualization_msgs::msg::Marker& getMarker()
  {
    return marker_;
  }

private:
  visualization_msgs::msg::Marker marker_;

  std::string frame_id_;
  double scale_;
  double z_;
};

class TwistMarkerPublisher : public rclcpp::Node
{
public:
  TwistMarkerPublisher() : Node("twist_marker")
  {
    std::string frame_id;
    double scale;
    double z;

    this->declare_parameter("frame_id");
    this->declare_parameter("scale");
    this->declare_parameter("vertical_position");

    this->get_parameter_or<std::string>("frame_id", frame_id, "base_footprint");
    this->get_parameter_or<double>("scale", scale, 1.0);
    this->get_parameter_or<double>("vertical_position", z, 2.0);

    marker_ = std::make_shared<TwistMarker>(frame_id, scale, z);

    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "twist", rclcpp::SystemDefaultsQoS(), std::bind(&TwistMarkerPublisher::callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<visualization_msgs::msg::Marker>("marker", rclcpp::QoS(rclcpp::KeepLast(1)));
  }

  void callback(const geometry_msgs::msg::Twist::ConstSharedPtr twist)
  {
    marker_->update(*twist);

    pub_->publish(marker_->getMarker());
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_;

  std::shared_ptr<TwistMarker> marker_ = nullptr;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto twist_mux_node = std::make_shared<TwistMarkerPublisher>();

  rclcpp::spin(twist_mux_node);

  rclcpp::shutdown();

  return EXIT_SUCCESS;
}