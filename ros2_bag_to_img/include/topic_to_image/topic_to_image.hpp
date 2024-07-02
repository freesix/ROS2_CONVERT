#ifndef TOPIC_TO_IMAGE_H
#define TOPIC_TO_IMAGE_H

#include <memory>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "boost/lexical_cast.hpp"


class TopicToImage : public rclcpp::Node
{
public:
  explicit TopicToImage(const rclcpp::NodeOptions & options);

private:
  void ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr & image_msg);

  image_transport::Subscriber image_sub_;

  std::string output_path_, file_prefix_, input_topic_;
};

#endif //TOPIC_TO_IMAGE_H
