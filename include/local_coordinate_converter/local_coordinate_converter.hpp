// Copyright (c) 2023 Leo Drive Teknoloji A.Ş.

#ifndef LOCAL_COORDINATE_CONVERTER__LOCAL_COORDINATE_CONVERTER
#define LOCAL_COORDINATE_CONVERTER__LOCAL_COORDINATE_CONVERTER


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <autoware_sensing_msgs/msg/gnss_ins_orientation_stamped.hpp>
#include <memory>

class LocalCoordinateConverter : public rclcpp::Node {
public:
    LocalCoordinateConverter();


    void NavSatFix2PoseWithCovarianceStamped(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg, int index);
    void ReadAutowareOrientation(const autoware_sensing_msgs::msg::GnssInsOrientationStamped::ConstSharedPtr msg, int index);

    std::vector<rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr> sub_navsatfix_vec;
    std::vector<rclcpp::Subscription<autoware_sensing_msgs::msg::GnssInsOrientationStamped>::SharedPtr> sub_autoware_orientation_vec;
    rclcpp::Subscription<autoware_sensing_msgs::msg::GnssInsOrientationStamped>::SharedPtr sub_autoware_orientation;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr> pub_vec;

    struct LocalOrigin {
        double latitude = 0.0;
        double longitude = 0.0;
        double altitude = 0.0;
        double x = 0.0;
        double y = 0.0;
    };
    LocalOrigin local_origin;
    void LatLon2UTM(double lat, double lon, double &x, double &y);

    std::vector<autoware_sensing_msgs::msg::GnssInsOrientationStamped> orientation_vec = std::vector<autoware_sensing_msgs::msg::GnssInsOrientationStamped>(10);

};

#endif  // LOCAL_COORDINATE_CONVERTER__LOCAL_COORDINATE_CONVERTER