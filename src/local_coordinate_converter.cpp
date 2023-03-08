// Copyright (c) 2023 Leo Drive Teknoloji A.Ş.

#include <memory>
#include <local_coordinate_converter/local_coordinate_converter.hpp>

#include <rclcpp/rclcpp.hpp>


LocalCoordinateConverter::LocalCoordinateConverter()
        : Node("LocalCoordinateConverter") {

    const auto number_of_input_topics = declare_parameter("number_of_input_topics", 0);

    std::vector<std::string> topic_names_{};
    topic_names_ = (declare_parameter<std::vector<std::string>>("topic_names"));

    sub_vec.reserve(number_of_input_topics);

    for (const auto & topic_name : topic_names_ ) {
        sub_vec.push_back(this->create_subscription<sensor_msgs::msg::NavSatFix>(
                topic_name, 10, std::bind(&LocalCoordinateConverter::NavSatFix2PoseWithCovarianceStamped, this,  std::placeholders::_1)));
        pub_vec.push_back(this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(topic_name + "_local", 10));
    }
}

void LocalCoordinateConverter::NavSatFix2PoseWithCovarianceStamped(const sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "I heard: [%f]", msg->latitude);
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg_local = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
    pub_vec[0]->publish(*msg_local);
}
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalCoordinateConverter>());
    rclcpp::shutdown();
    return 0;
}