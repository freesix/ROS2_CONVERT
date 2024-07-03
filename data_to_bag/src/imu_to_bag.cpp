#include "imu_to_bag/imu_to_bag.hpp"

ImuToBag::ImuToBag(const rclcpp::NodeOptions &options) : Node("data_to_bag", options){
    input_path_ = this->declare_parameter<std::string>("input/path");
    output_bag_path_ = this->declare_parameter<std::string>("output/bag_path");
    imu_topic_ = this->declare_parameter<std::string>("input/imu_topic");

    CheckParams();
    WriteBag();

    RCLCPP_INFO_STREAM(get_logger(), "Completed");
    rclcpp::shutdown();
}

void ImuToBag::WriteBag(){
    
    writer_->open(output_bag_path_+"out");
    writer_->create_topic({
        imu_topic_,
        "sensor_msgs/msg/Imu",
        rmw_get_serialization_format(),
        ""
    });

    std::ifstream imu_file(input_path_+"/imu.csv");
    if(!imu_file.is_open()){
        RCLCPP_ERROR_STREAM(get_logger(), "Error opening imu file");
        rclcpp::shutdown(); 
    }
    std::string line;
    std::getline(imu_file, line); // skip header
    while(std::getline(imu_file, line)){
        sensor_msgs::msg::Imu imu_msg;
        imu_msg = ImuConver(line);
        RCLCPP_INFO_STREAM(get_logger(), "Writing imu message: " << 
            imu_msg.header.stamp.sec << "." << imu_msg.header.stamp.nanosec);
        writer_->write(imu_msg, imu_topic_, imu_msg.header.stamp);
    }
    imu_file.close();
}

sensor_msgs::msg::Imu ImuToBag::ImuConver(const std::string &line){
    std::istringstream ss(line);
    std::string field;
    std::vector<std::string> fields;
    while(std::getline(ss, field, ' ')){
        fields.push_back(field);
    }

    imu_msg.header.stamp.sec = std::stoll(fields[0].substr(0, fields[0].find('.')));
    imu_msg.header.stamp.nanosec = std::stoll(fields[0].substr(fields[0].find('.')+1));
    imu_msg.linear_acceleration.x = std::stof(fields[1]);
    imu_msg.linear_acceleration.y = std::stof(fields[2]);
    imu_msg.linear_acceleration.z = std::stof(fields[3]);
    imu_msg.angular_velocity.x = std::stof(fields[4]);
    imu_msg.angular_velocity.y = std::stof(fields[5]);
    imu_msg.angular_velocity.z = std::stof(fields[6]); 
    return imu_msg;
}


void ImuToBag::CreateDirectories(){
    rcpputils::fs::path o_dir(output_bag_path_);
    try{
        rcpputils::fs::create_directories(o_dir);
    }
    catch(const std::exception& e){
        RCLCPP_ERROR_STREAM(get_logger(), "Error creating directories: " << e.what());
        rclcpp::shutdown();
    }       
}

void ImuToBag::CheckParams(){
    if(input_path_.empty()){
        RCLCPP_ERROR_STREAM(get_logger(), "No input path provided");
        rclcpp::shutdown();
    }
    if(output_bag_path_.empty()){
        RCLCPP_ERROR_STREAM(get_logger(), "No output bag path provided");
        rclcpp::shutdown();
        // CreateDirectories();
    } 
    if(imu_topic_.empty()){
        RCLCPP_INFO_STREAM(get_logger(), "No imu topic provided");
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ImuToBag)
