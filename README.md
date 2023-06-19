# Decription

This package aims to read data from different sensors and convert sensor_msgs/NavSatFix and autoware_sensing_msgs/GnssInsOrientationStamped messages to geometry_msgs/PoseWithCovarianceStamped and geometry_msgs/PoseStamped messages. In this way, it allows you to easily compare sensor data. 

# Parameters

```number_of_input_topics:``` Specify how many different sensor data you want to convert.</br>
```navsatfix_topic_names:``` Specify your topic names of type sensor_msgs/NavSatFix </br>
```autoware_orientation_topic_names:``` Specify your topic names of type autoware_sensing_msgs/GnssInsOrientationStamped </br>
```origin_latitude:``` origin value latitude for get UTM local coordinates </br>
```origin_longitude:``` origin value latitude for get UTM local coordinates </br>
```origin_altitude:``` origin value latitude for get UTM local coordinates</br>



