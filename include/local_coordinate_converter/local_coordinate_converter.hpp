// Copyright (c) 2023 Leo Drive Teknoloji A.Ş.

#ifndef LOCAL_COORDINATE_CONVERTER__LOCAL_COORDINATE_CONVERTER
#define LOCAL_COORDINATE_CONVERTER__LOCAL_COORDINATE_CONVERTER


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <memory>

class LocalCoordinateConverter : public rclcpp::Node {
public:
    LocalCoordinateConverter();


    void NavSatFix2PoseWithCovarianceStamped(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg);

    std::vector<rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr> sub_vec;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr> pub_vec;

};

#endif  // LOCAL_COORDINATE_CONVERTER__LOCAL_COORDINATE_CONVERTER