// Copyright (c) 2023 Leo Drive Teknoloji A.Ş.
#include "local_coordinate_converter/local_coordinate_converter.hpp"

#include <rclcpp/rclcpp.hpp>
#include <autoware_sensing_msgs/msg/gnss_ins_orientation_stamped.hpp>
#include <memory>

LocalCoordinateConverter::LocalCoordinateConverter()
        : Node("LocalCoordinateConverter") {

    local_origin.latitude = declare_parameter("origin_latitude", 0.0);
    local_origin.longitude = declare_parameter("origin_longitude", 0.0);
    local_origin.altitude = declare_parameter("origin_altitude", 0.0);

    LatLon2UTM(local_origin.latitude, local_origin.longitude, local_origin.x, local_origin.y);

    const auto number_of_input_topics = declare_parameter("number_of_input_topics", 3);

    sub_navsatfix_vec.reserve(number_of_input_topics);
    sub_autoware_orientation_vec.reserve(number_of_input_topics);
    orientation_vec.reserve(number_of_input_topics);
    pub_vec.reserve(number_of_input_topics);

    std::vector<std::string> autoware_orientation_topic_names_{};
    autoware_orientation_topic_names_ = (declare_parameter<std::vector<std::string>>("autoware_orientation_topic_names"));

    std::vector<std::string> navsatfix_topic_names_{};
    navsatfix_topic_names_ = (declare_parameter<std::vector<std::string>>("navsatfix_topic_names"));

    for (int index = 0; index < navsatfix_topic_names_.size(); ++index) {
        const auto topic_name = navsatfix_topic_names_[index];

        std::function<void(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)>  fnavsatfix
                = std::bind(&LocalCoordinateConverter::NavSatFix2PoseWithCovarianceStamped,this, std::placeholders::_1, index);

        sub_navsatfix_vec.push_back(this->create_subscription<sensor_msgs::msg::NavSatFix>(topic_name, rclcpp::SensorDataQoS(), fnavsatfix));


        std::function<void(const autoware_sensing_msgs::msg::GnssInsOrientationStamped::ConstSharedPtr msg)>  fautowareorientation
                = std::bind(&LocalCoordinateConverter::ReadAutowareOrientation,this, std::placeholders::_1, index);

        sub_autoware_orientation_vec.push_back(this->create_subscription<autoware_sensing_msgs::msg::GnssInsOrientationStamped>(autoware_orientation_topic_names_[index], rclcpp::SensorDataQoS(),fautowareorientation));

        pub_vec.push_back(this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(topic_name + "_local_pose", 10));

    }

}
void LocalCoordinateConverter::LatLon2UTM(double lat, double lon, double &x, double &y) {
    int zone;
    bool northp;
    GeographicLib::UTMUPS::Forward(lat, lon, zone, northp, x, y);
}
void LocalCoordinateConverter::NavSatFix2PoseWithCovarianceStamped(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg, int index) {
    geometry_msgs::msg::PoseWithCovarianceStamped pose_with_cov_msg;
    pose_with_cov_msg.header = msg->header;
    double x, y;
    LatLon2UTM(msg->latitude, msg->longitude, x, y);
    pose_with_cov_msg.pose.pose.position.x = x - local_origin.x;
    pose_with_cov_msg.pose.pose.position.y = y - local_origin.y;
    pose_with_cov_msg.pose.pose.position.z = msg->altitude - local_origin.altitude;

    pose_with_cov_msg.pose.covariance[0] = msg->position_covariance[0];
    pose_with_cov_msg.pose.covariance[7] = msg->position_covariance[4];
    pose_with_cov_msg.pose.covariance[14] = msg->position_covariance[8];


    pose_with_cov_msg.pose.pose.orientation = orientation_vec[index].orientation.orientation;
    pose_with_cov_msg.pose.covariance[21] = std::pow(orientation_vec[index].orientation.rmse_rotation_x,2);
    pose_with_cov_msg.pose.covariance[28] = std::pow(orientation_vec[index].orientation.rmse_rotation_y,2);
    pose_with_cov_msg.pose.covariance[35] = std::pow(orientation_vec[index].orientation.rmse_rotation_z,2);
    
    pub_vec[index]->publish(pose_with_cov_msg);

}

void LocalCoordinateConverter::ReadAutowareOrientation(const autoware_sensing_msgs::msg::GnssInsOrientationStamped::ConstSharedPtr msg, int index) {

    orientation_vec[index].orientation = msg->orientation;

}
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalCoordinateConverter>());
    rclcpp::shutdown();
    return 0;
}