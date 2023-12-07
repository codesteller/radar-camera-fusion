#ifndef BEV_TRANSFORM_HPP_
#define BEV_TRANSFORM_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>
#include <opencv2/core.hpp>

#include "bev_transform/four_points.hpp"
#include "aras_msgs/msg/bounding_box.hpp"
#include "aras_msgs/msg/bounding_boxes.hpp"
#include "aras_msgs/msg/object_count.hpp"
#include "aras_msgs/msg/traffic_object.hpp"
#include "aras_msgs/msg/traffic_objects.hpp"


namespace bev_transform {

class BEVTransform : public rclcpp::Node {
public:
    BEVTransform();

private:

    // Declare private member variables for each parameter in the YAML file
    double car_width_;
    double carpet_width_;
    double car_to_carpet_distance_;
    double carpet_length_;
    double tl_x_;
    double tl_y_;
    double tr_x_;
    double tr_y_;
    double br_x_;
    double br_y_;
    double bl_x_;
    double bl_y_;

    static constexpr int kBEVImgWidth = 1000;
    static constexpr int kBEVImgHeight = 10000;
    FourPoints four_points =
        FourPoints(cv::Point2f(250, 8000), cv::Point2f(750, 8000),
                   cv::Point2f(750, 8500), cv::Point2f(250, 8500));
    FourPoints four_image_points;

    double width_pixel_to_meter_ratio_;
    double height_pixel_to_meter_ratio_;
    double car_y_in_pixel_;
    double car_width_in_pixel_;

    cv::Mat bev_transform_matrix_;

    void calibrate();
    void boundingBoxCallback(const aras_msgs::msg::BoundingBoxes::SharedPtr msg);

    rclcpp::Subscription<aras_msgs::msg::BoundingBoxes>::SharedPtr bbox_sub_;
    // rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr distance_pub_;
    rclcpp::Publisher<aras_msgs::msg::TrafficObjects>::SharedPtr tobject_pub_;
    rclcpp::Publisher<aras_msgs::msg::TrafficObjects>::SharedPtr nearest_object_pub_;
};

} // namespace bev_transform

#endif // BEV_TRANSFORM_HPP_
