//
// Created by zuoxin on 18-10-5.
//

#include "InstanceSeg.h"
#include <iostream>

bool InstanceSeg::InitModel(const std::string model_path, const std::string config_path) {

    try {
        network_ = cv::dnn::readNet(model_path, config_path);
        network_.setPreferableBackend(0);
        network_.setPreferableTarget(1);
    }catch(...){
        std::cout<<"init model from "<<model_path<<" failed!"<<std::endl;
        return false;
    }
    std::cout<<"init model from "<<model_path<<" sucessed!"<<std::endl;
    return true;
}

bool InstanceSeg::RunInstanceSeg(cv::Mat &img, std::vector<cv::Rect> &bboxes, std::vector<cv::Mat> &masks) {
    cv::Size in_size(img_width_ > 0 ? img_width_: img.cols,
                     img_height_ > 0 ? img_height_: img.rows);
    cv::Mat input_data;
    cv::dnn::blobFromImage(img, input_data, scale_, in_size, mean_, true, false);
    network_.setInput(input_data);
    std::vector<std::vector<cv::Mat>> outputs;
    std::vector<cv::String> output_names{"detection_out_final", "detection_masks"};
    network_.forward(outputs, output_names);
}

bool InstanceSeg::PostProcess(std::vector<std::vector<cv::Mat>>& outputs) {

}
