//
// Created by zuoxin on 18-10-5.
//

#include "InstanceSeg.h"
#include <iostream>
#include <fstream>

bool InstanceSeg::InitModel(const std::string model_path, const std::string config_path) {

    try {
        network_ = cv::dnn::readNet(model_path, config_path);
        network_.setPreferableBackend(0);
        network_.setPreferableTarget(0);
    }catch(...){
        std::cout<<"init model from "<<model_path<<" failed!"<<std::endl;
        return false;
    }
    std::cout<<"init model from "<<model_path<<" sucessed!"<<std::endl;
    return true;
}

bool InstanceSeg::RunInstanceSeg(cv::Mat &img, std::vector<ObjectItem>& objects) {
    cv::Size in_size(img_width_ > 0 ? img_width_: img.cols,
                     img_height_ > 0 ? img_height_: img.rows);
    cv::Mat input_data;
    cv::dnn::blobFromImage(img, input_data, scale_, in_size, mean_, true, false);
    network_.setInput(input_data);
    std::vector<cv::Mat> outputs;
    std::vector<cv::String> output_names{"detection_out_final", "detection_masks"};
    try {
        network_.forward(outputs, output_names);
    }
    catch (...)
    {
        std::cerr<<"there are something wrong in the process of net forward"<<std::endl;
        return false;
    }
    if(outputs.size()<output_names.size() )
        return false;
    PostProcess(outputs, img.size(), objects);
}

bool InstanceSeg::PostProcess(const std::vector<cv::Mat> &outputs, cv::Size img_size, std::vector<ObjectItem>& objects){
    auto box_out = outputs.at(0);
    auto mask_out = outputs.at(1);
    float* box_data = (float*)box_out.data;
    float* mask_data = (float*)mask_out.data;
    for (size_t i= 0; i<box_out.total(); i += 7)
    {
        int class_id = (int)box_data[i + 1];
        int left = static_cast<int>(box_data[i + 3] * img_size.width);
        int top = static_cast<int>(box_data[i + 4] * img_size.height);
        int right = static_cast<int>(box_data[i + 5] * img_size.width);
        int bottom = static_cast<int>(box_data[i + 6] * img_size.height);
        float score = box_data[i + 2];
        if (score > conf_th_)
        {
            ObjectItem new_item;
            new_item.box = cv::Rect(cv::Point(left,top),cv::Point(right, bottom));
            new_item.img_size = img_size;
            new_item.score = score;
            new_item.class_label = class_id;
            cv::Mat tmp_mask = cv::Mat(15, 15, CV_32F, mask_data+90*15*15*i/7+15*15*class_id);
            cv::Mat tmp_maskb = cv::Mat(15, 15, CV_8U);
            int box_width = right - left;
            int box_height = bottom - top;
            cv::resize(tmp_mask, tmp_mask, cv::Size(box_width, box_height));
            cv::inRange(tmp_mask,  0.5, 100, tmp_maskb);
            new_item.mask = tmp_maskb;
            objects.push_back(new_item);
        }
    }

}

cv::Mat InstanceSeg::DrawObjects(cv::Mat img, std::vector<ObjectItem> &objects) {
    for(auto object : objects)
    {
        auto box = object.box;
        cv::rectangle(img, box.tl(), box.br(), (255,0,0));
        std::string label = std::to_string(object.score);
        int baseLine;
        cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
        auto top = object.box.y;
        auto left = object.box.x;
        top = std::max(top, labelSize.height);
        cv::rectangle(img, cv::Point(left, top - labelSize.height),
                  cv::Point(left + labelSize.width, top + baseLine), cv::Scalar::all(255), cv::FILLED);
        putText(img, label, cv::Point(left, top), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar());
        cv::Mat roi_img = img(box);
        cv::Mat mask = object.mask;
        std::cout<<roi_img.size()<<std::endl;
        std::cout<<mask.size()<<std::endl;
        cv::cvtColor(object.mask, mask, CV_GRAY2BGR);
        cv::addWeighted(roi_img, 0.5, mask, 0.5, 0, roi_img);
    }
    return img;
}



bool InstanceSeg::ReadNamesFromFile(const std::string name_file_path, std::vector<std::string>& class_names) {

    std::fstream fin(name_file_path);
    if (!fin.is_open())
    {
        std::cerr<<"file does not exist: "<<name_file_path<<std::endl;
        return false;
    }
    std::string tmp_s;
    class_names.clear();
    while(std::getline(fin,tmp_s))
    {
        class_names.push_back(tmp_s);
    }
    return true;
}
