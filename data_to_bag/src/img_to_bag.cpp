#include "img_to_bag/img_to_bag.hpp"

ImgToBag::ImgToBag(const rclcpp::NodeOptions &options) : Node("data_to_bag", options){
    input_path_ = this->declare_parameter<std::string>("input/path");
    output_bag_path_ = this->declare_parameter<std::string>("output/bag_path");
    img_topics_ = this->declare_parameter<std::vector<std::string>>("input/img_topics");

    CheckParams();
    WriteBag();

    RCLCPP_INFO_STREAM(get_logger(), "Completed");
    rclcpp::shutdown();
}

void ImgToBag::WriteBag(){
    
    writer_->open(output_bag_path_+"out");
    
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


sensor_msgs::msg::Image ImgToBag::ImageToBag(std::string &img){
    cv::Mat image = cv::imread(img, cv::IMREAD_COLOR);
    if(image.empty()){
        RCLCPP_ERROR_STREAM(get_logger(), "Error reading image");
        rclcpp::shutdown();
    }
    std::string img_name = img.substr(img.find_last_of('/')+1);
    cv_bridge::CvImage cv_img;
    cv_img.image = image;
    cv_img.encoding = sensor_msgs::image_encodings::BGR8;
    cv_img.header.stamp.sec = std::stoll(img_name.substr(0, img_name.find('.')));
    cv_img.header.stamp.nanosec = std::stoll(img_name.substr(img_name.find('.')+1, img_name.find_last_of('.')));
    cv_img.header.frame_id = "camera";
    return *cv_img.toImageMsg();
}

std::vector<std::string> ImgToBag::GetImagesPath(std::string &topic){
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



void ImgToBag::CreateDirectories(){
    rcpputils::fs::path o_dir(output_bag_path_);
    try{
        rcpputils::fs::create_directories(o_dir);
    }
    catch(const std::exception& e){
        RCLCPP_ERROR_STREAM(get_logger(), "Error creating directories: " << e.what());
        rclcpp::shutdown();
    }       
}

void ImgToBag::CheckParams(){
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
    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ImgToBag)
