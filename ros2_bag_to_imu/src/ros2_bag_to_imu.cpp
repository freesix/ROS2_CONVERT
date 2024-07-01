#include "ros2_bag_to_imu/ros2_bag_to_imu.hpp"


BagToImu::BagToImu(const rclcpp::NodeOptions &options) : Node("bag_to_imu", options){
    input_path_ = this->declare_parameter<std::string>("input/path");
    bag_format_ = this->declare_parameter<std::string>("input/bag_format", "cdr");
    storage_id_ = this->declare_parameter<std::string>("input/bag_storage_id", "sqlite3");
    RCLCPP_INFO_STREAM(get_logger(), "Using bag format: " << bag_format_ << ".");
    RCLCPP_INFO_STREAM(get_logger(), "Using bag storage_id: " << storage_id_ << ".");

    input_topics_ = this->declare_parameter<std::vector<std::string>>("input/topics");
    output_path_ = this->declare_parameter<std::string>("output/path", "~/tmp");

    CheckParams();
    ReadBag();

    RCLCPP_INFO_STREAM(get_logger(), "Complete");
    rclcpp::shutdown();
}

void BagToImu::ReadBag(){
    rosbag2_storage::StorageOptions storage_options;
    rosbag2_cpp::ConverterOptions conver_options;
    storage_options.uri = input_path_;    
    storage_options.storage_id = storage_id_;
    conver_options.output_serialization_format = bag_format_;
    CreateDirectories();

    rosbag2_cpp::Reader reader(std::make_unique<rosbag2_cpp::readers::SequentialReader>());
    reader.open(storage_options, conver_options);

    rosbag2_storage::StorageFilter storage_filter;
    storage_filter.topics = input_topics_;
    reader.set_filter(storage_filter);

    auto topics_types = reader.get_all_topics_and_types();
    
    std::string fname; 
    fname = output_path_ + "/" + "imu.csv";
    std::ofstream outfile;
    outfile.open(fname);
    if(!outfile){
        std::cout << "open file failed" << std::endl;
        return;
    }
    outfile << "time ax ay az gx gy gz" << std::endl;
    while(reader.has_next()){
        auto bag_message = reader.read_next();

        if(std::find(input_topics_.begin(), input_topics_.end(), bag_message->topic_name)
            != input_topics_.end()){
            std::string curr_topic_type;
            for(const auto &topic_type : topics_types){
                if(topic_type.name == bag_message->topic_name){
                    curr_topic_type = topic_type.type;
                }
            }   
            if(curr_topic_type.empty()){
                RCLCPP_WARN_STREAM(get_logger(), "No Imu or CompressedImu topic types available in the rosbag");
                break;
            }

            auto imu_msg = MessageToImu(bag_message, curr_topic_type);
                        
            if(imu_msg.sec == 0.0 && imu_msg.ax == 0.0 && imu_msg.gx == 0.0){
                RCLCPP_INFO_STREAM(get_logger(), "Could not convert the message to Imu type: "
                    << bag_message->topic_name << "at "<< bag_message->time_stamp);
                continue;
            }
            
            std::string target_topic_name = bag_message->topic_name;
            if(target_topic_name.substr(0, 1) == "/"){
                target_topic_name = target_topic_name.substr(1);
            }
            target_topic_name = std::regex_replace(target_topic_name, std::regex("/"), "_");
             
            double time = imu_msg.sec + imu_msg.nanosec * 1e-9;

            outfile << std::fixed << time <<" " 
                    << imu_msg.ax << " " << imu_msg.ay << " " << imu_msg.az << " "
                    << imu_msg.gx << " " << imu_msg.gy << " " << imu_msg.gz << std::endl;
            RCLCPP_INFO_STREAM(get_logger(), "Imu: " << std::fixed << time);
            }
    }
    outfile.close();   
}



void BagToImu::CreateDirectories(){
    for(const auto &target_topic : input_topics_){
        RCLCPP_INFO_STREAM(get_logger(), target_topic);

        rcpputils::fs::path o_dir(output_path_);
        auto target_topic_name = target_topic;
        if(target_topic_name.substr(0, 1) == "/"){
            target_topic_name = target_topic_name.substr(1);
        }
        target_topic_name = std::regex_replace(target_topic_name, std::regex("/"), "_");
        o_dir = o_dir / rcpputils::fs::path(target_topic_name);
        if(rcpputils::fs::create_directories(o_dir)){
            std::cout << "created: " << o_dir << std::endl;
        }
    }
}

BagToImu::Imu_bridge BagToImu::MessageToImu(std::shared_ptr<
    rosbag2_storage::SerializedBagMessage> bag_message, const std::string &topic_type){

    BagToImu::Imu_bridge imu_ptr;
    // RCLCPP_INFO_STREAM(get_logger(), "topic_type: " << topic_type);
    if(topic_type == "sensor_msgs/msg/Imu"){
        sensor_msgs::msg::Imu extracted_msg;
        rclcpp::Serialization<sensor_msgs::msg::Imu> serialization;
        rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
        serialization.deserialize_message(&extracted_serialized_msg, &extracted_msg);

        imu_ptr.sec = extracted_msg.header.stamp.sec;
        imu_ptr.nanosec = extracted_msg.header.stamp.nanosec;
        imu_ptr.ax = extracted_msg.linear_acceleration.x; 
        imu_ptr.ay = extracted_msg.linear_acceleration.y; 
        imu_ptr.az = extracted_msg.linear_acceleration.z; 
        imu_ptr.gx = extracted_msg.angular_velocity.x; 
        imu_ptr.gy = extracted_msg.angular_velocity.y; 
        imu_ptr.gz = extracted_msg.angular_velocity.z;
    }  

    return imu_ptr; 
}

void BagToImu::CheckParams(){
    if(input_topics_.empty()){
        RCLCPP_ERROR_STREAM(get_logger(), 
                            "At least one topic required, none provided. Terminating...");
        rclcpp::shutdown(nullptr, "Invalid Input Topics"); 
    }
    if(input_path_.empty()){
        RCLCPP_ERROR_STREAM(get_logger(), 
                            "No input path provided. Terminating...");
        rclcpp::shutdown(nullptr, "Invalid Rosbag Path");
    }
    if(output_path_.empty()){
        RCLCPP_ERROR_STREAM(get_logger(), 
                            "Invalid Path provided:" << output_path_ << ". Terminating...");
        rclcpp::shutdown(nullptr, "Invalid Output Path");
    }
    if(!std::filesystem::exists(output_path_)){
        RCLCPP_INFO_STREAM(get_logger(), 
                            "The Provied path [" << output_path_ << "] doesn't exist. Try to create");
        if(std::filesystem::create_directories(output_path_)){
            RCLCPP_INFO_STREAM(get_logger(), "The Provided Path was created successfully.");
        }
        else{
            RCLCPP_ERROR_STREAM(get_logger(),
                    "Could not create the output directory: " << output_path_ << ". Terminating...");
            rclcpp::shutdown(nullptr, "Missing Permissions on the output path");
        }
    }
    RCLCPP_INFO_STREAM(get_logger(), "Saving IMU data to:" << output_path_);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(BagToImu)