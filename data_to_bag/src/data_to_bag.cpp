#include "data_to_bag/data_to_bag.hpp"

DataToBag::DataToBag(const rclcpp::NodeOptions &options) : Node("data_to_bag", options){
    input_path_ = this->declare_parameter<std::string>("input/path");
    output_bag_path_ = this->declare_parameter<std::string>("output/bag_path");
    img_topics_ = this->declare_parameter<std::vector<std::string>>("input/img_topics");
    imu_topic_ = this->declare_parameter<std::string>("input/imu_topic");

    CheckParams();
    WriteBag();

    RCLCPP_INFO_STREAM(get_logger(), "Completed");
    rclcpp::shutdown();
}

void DataToBag::WriteBag(){
    
    writer_->open(output_bag_path_+"out");
    if(0){
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
            imu_msg = ImuToBag(line);
            RCLCPP_INFO_STREAM(get_logger(), "Writing imu message: " << 
                imu_msg.header.stamp.sec << "." << imu_msg.header.stamp.nanosec);
            writer_->write(imu_msg, imu_topic_, imu_msg.header.stamp);
        }
        imu_file.close();
    }

    if(convert_img_){
        for(auto &topic : img_topics_){
            writer_->create_topic({
                topic,
                "sensor_msgs/msg/Image",
                rmw_get_serialization_format(),
                ""
            });
            std::vector<std::string> img_names = GetImagesPath(topic);
            for(auto &img : img_names){
                sensor_msgs::msg::Image img_msg;
                img_msg = ImageToBag(img);
                RCLCPP_INFO_STREAM(get_logger(), "Writing image topic " << topic << " message: " <<
                    img_msg.header.stamp.sec << "." << img_msg.header.stamp.nanosec);
                writer_->write(img_msg, topic, img_msg.header.stamp); 
            }
        }
    }

    if(convert_img_ == false && convert_imu_ == false){
        RCLCPP_ERROR_STREAM(get_logger(), "No topics to convert");
        rclcpp::shutdown();
    }
}

sensor_msgs::msg::Imu DataToBag::ImuToBag(const std::string &line){
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

sensor_msgs::msg::Image DataToBag::ImageToBag(std::string &img){
    cv::Mat image = cv::imread(img, cv::IMREAD_COLOR);
    if(image.empty()){
        RCLCPP_ERROR_STREAM(get_logger(), "Error reading image");
        rclcpp::shutdown();
    }
    RCLCPP_INFO_STREAM(get_logger(), "Image read: " << img);
    std::string img_name = img.substr(img.find_last_of('/')+1);
    cv_bridge::CvImage cv_img;
    cv_img.image = image;
    cv_img.encoding = sensor_msgs::image_encodings::BGR8;
    cv_img.header.stamp.sec = std::stoll(img_name.substr(0, img_name.find('.')));
    cv_img.header.stamp.nanosec = std::stoll(img_name.substr(img_name.find('.')+1, img_name.find_last_of('.')));
    cv_img.header.frame_id = "camera";
    return *cv_img.toImageMsg();
}

std::vector<std::string> DataToBag::GetImagesPath(std::string &topic){
    std::vector<std::string> img_names;
    std::string img_file(input_path_+topic);
    if(!std::filesystem::exists(img_file)){
        RCLCPP_ERROR_STREAM(get_logger(), "Images file does not exist");
        rclcpp::shutdown();
    }
    for(const auto& entry : std::filesystem::directory_iterator(img_file)){
        img_names.push_back(entry.path().string());
    } 
    return img_names;
}



void DataToBag::CreateDirectories(){
    rcpputils::fs::path o_dir(output_bag_path_);
    try{
        rcpputils::fs::create_directories(o_dir);
    }
    catch(const std::exception& e){
        RCLCPP_ERROR_STREAM(get_logger(), "Error creating directories: " << e.what());
        rclcpp::shutdown();
    }       
}

void DataToBag::CheckParams(){
    if(input_path_.empty()){
        RCLCPP_ERROR_STREAM(get_logger(), "No input path provided");
        rclcpp::shutdown();
    }
    if(output_bag_path_.empty()){
        RCLCPP_ERROR_STREAM(get_logger(), "No output bag path provided");
        rclcpp::shutdown();
        // CreateDirectories();
    } 
    if(img_topics_.empty()){
        RCLCPP_INFO_STREAM(get_logger(), "No image topics provided");
        convert_img_ = false;
    }
    if(imu_topic_.empty()){
        RCLCPP_INFO_STREAM(get_logger(), "No imu topic provided");
        convert_imu_ = false;
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(DataToBag)
