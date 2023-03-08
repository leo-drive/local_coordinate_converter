// Copyright (c) 2023 Leo Drive Teknoloji A.Ş.

#include <memory>
#include <local_coordinate_converter/local_coordinate_converter.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>

LocalCoordinateConverter::LocalCoordinateConverter()
        : Node("LocalCoordinateConverter") {

    local_origin.latitude = declare_parameter("origin_latitude", 0.0);
    local_origin.longitude = declare_parameter("origin_longitude", 0.0);
    local_origin.altitude = declare_parameter("origin_altitude", 0.0);

    LatLon2UTM(local_origin.latitude, local_origin.longitude, local_origin.x, local_origin.y);

    const auto number_of_input_topics = declare_parameter("number_of_input_topics", 3);

    std::vector<std::string> topic_names_{};
    topic_names_ = (declare_parameter<std::vector<std::string>>("topic_names"));
    sub_navsatfix_vec.reserve(number_of_input_topics);

    for (int index = 0; index < topic_names_.size(); ++index) {
        const auto topic_name = topic_names_[index];

        std::function<void(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg)>  fcn
            = std::bind(&LocalCoordinateConverter::NavSatFix2PoseWithCovarianceStamped,this, std::placeholders::_1, index);


        sub_navsatfix_vec.push_back(this->create_subscription<sensor_msgs::msg::NavSatFix>(topic_name, 10, fcn));

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
    pose_with_cov_msg.pose.covariance[4] = msg->position_covariance[4];
    pose_with_cov_msg.pose.covariance[8] = msg->position_covariance[8];
    pub_vec[index]->publish(pose_with_cov_msg);

}
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalCoordinateConverter>());
    rclcpp::shutdown();
    return 0;
}