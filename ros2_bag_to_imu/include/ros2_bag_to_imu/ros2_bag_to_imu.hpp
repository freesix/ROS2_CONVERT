#pragma once 

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <filesystem>
#include <regex>
#include <boost/lexical_cast.hpp>
#include <fstream>
class BagToImu : public rclcpp::Node
{
public:
    explicit BagToImu(const rclcpp::NodeOptions &options);
    struct Imu_bridge{
        int32_t sec;
        int32_t nanosec;
        double ax;
        double ay;
        double az;
        double gx;
        double gy;
        double gz;
    };


private:

    std::string output_path_, input_path_, bag_format_, storage_id_;
    std::vector<std::string> input_topics_;
    
    void CheckParams();

    void ReadBag();

    void CreateDirectories();

    Imu_bridge MessageToImu(std::shared_ptr<rosbag2_storage::SerializedBagMessage> bag_message,
            const std::string &topic_type);
};