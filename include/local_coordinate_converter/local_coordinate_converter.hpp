// Copyright (c) 2023 Leo Drive Teknoloji A.Ş.

#ifndef LOCAL_COORDINATE_CONVERTER__LOCAL_COORDINATE_CONVERTER
#define LOCAL_COORDINATE_CONVERTER__LOCAL_COORDINATE_CONVERTER


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/MGRS.hpp>
#include <std_msgs/msg/float64.hpp>
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
    std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr> pub_pose_vec;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_pos_difference;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_pos_difference_x;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_pos_difference_y;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_pos_difference_z;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_ekf_posewithcovariance;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_ekf_pose;

    void ReadEKF(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
    enum class MGRSPrecision {
        _1_METER = 5,
        _100MICRO_METER = 9,
    };

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

    double last_pose_x_0 , last_pose_x_1, last_pose_y_0, last_pose_y_1, last_pose_z_0, last_pose_z_1, positionDifference = 0 ,positionDifference_x = 0, positionDifference_y = 0, positionDifference_z = 0;
};

#endif  // LOCAL_COORDINATE_CONVERTER__LOCAL_COORDINATE_CONVERTER