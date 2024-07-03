#pragma once 

#include <fstream>
#include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

class ImgToBag : public rclcpp::Node{
public:
    explicit ImgToBag(const rclcpp::NodeOptions &options);

private:    
    sensor_msgs::msg::Image image_msg;

    std::string input_path_, output_bag_path_;
    std::vector<std::string> img_topics_;


    std::shared_ptr<rosbag2_cpp::Writer> writer_ = std::make_shared<rosbag2_cpp::Writer>();
    
    void CheckParams();

    void WriteBag();


    sensor_msgs::msg::Image ImageToBag(std::string &img);
    
    std::vector<std::string> GetImagesPath(std::string &topic);

      void CreateDirectories();

};