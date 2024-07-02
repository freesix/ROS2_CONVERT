#ifndef TOPIC_TO_IMAGE_H
#define TOPIC_TO_IMAGE_H

#include <memory>
#include <algorithm>
#include <regex>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "boost/lexical_cast.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rcpputils/filesystem_helper.hpp"
#include "rcutils/time.h"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "sensor_msgs/msg/compressed_image.h"
#include "sensor_msgs/msg/image.hpp"


class BagToImage : public rclcpp::Node
{
public:
  explicit BagToImage(const rclcpp::NodeOptions & options);

private:

  image_transport::Subscriber image_sub_;

  std::string output_path_, input_path_, bag_format_, storage_id_;
  std::vector<std::string> input_topics_;
  void CheckParams();

  void ReadBag();

  void CreateDirectories();

  cv_bridge::CvImagePtr
  MessageToImage(std::shared_ptr<rosbag2_storage::SerializedBagMessage> bag_message, const std::string &topic_type);
};

#endif //TOPIC_TO_IMAGE_H
