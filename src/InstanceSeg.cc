//
// Created by zuoxin on 18-10-5.
//

#include "InstanceSeg.h"
#include <iostream>
#include <fstream>
#include "mxnet_utils.h"

bool InstanceSeg::InitModel(const std::string model_path, const std::string config_path) {

    if(net_type_ == "opencv") {
        try {
            network_ = cv::dnn::readNet(model_path, config_path);
            network_.setPreferableBackend(0);
            network_.setPreferableTarget(0);
        } catch (...) {
            std::cout << "init model from " << model_path << " failed!" << std::endl;
            return false;
        }
        std::cout << "init model from " << model_path << " sucessed!" << std::endl;
    }
    else if (net_type_ == "mxnet"){
        if(use_gpu_)
        {
            ctx_ = Context::gpu(0);
        }
        try {
            LoadCheckpoint(config_path, model_path, &mx_net_, &args_, &auxs_, ctx_);
        }catch(...)
        {
            std::cout << "init model from " << model_path << " failed!" << std::endl;
            return false;
        }
        std::cout << "init model from " << model_path << " sucessed!" << std::endl;
    }
    return true;
}

bool InstanceSeg::RunInstanceSeg(cv::Mat img, std::vector<std::shared_ptr<ImgObjectInfo>>& objects) {
    auto raw_img_size = img.size();
    img = ResizeShortWithin(img, min_size_, max_size_,8);
    float x_scale = float(raw_img_size.width)/img.size().width;
    float y_scale = float(raw_img_size.height)/img.size().height;
    if(net_type_ == "opencv") {
        cv::Size in_size = img.size();
        cv::Mat input_data;
        cv::dnn::blobFromImage(img, input_data, scale_, in_size, mean_, true, false);
        network_.setInput(input_data);
        std::vector<cv::Mat> outputs;
        std::vector<cv::String> output_names{"detection_out_final", "detection_masks"};
        try {
            network_.forward(outputs, output_names);
        }
        catch (...) {
            std::cerr << "there are something wrong in the process of net forward" << std::endl;
            return false;
        }
        if (outputs.size() < output_names.size())
            return false;
        PostProcessOnOpenCV(outputs, raw_img_size, objects);
    }
    else if (net_type_ == "mxnet")
    {
        auto data = AsData(img, ctx_);
        args_["data"] = data;
        Executor* mx_executor_ = mx_net_.SimpleBind(ctx_, args_, std::map<std::string, NDArray>(),
                                          std::map<std::string, OpReqType>(), auxs_);
        NDArray::WaitAll();
        mx_executor_->Forward(false);
        auto ids = mx_executor_->outputs[0].Copy(Context(kCPU,0));
        auto scores = mx_executor_->outputs[1].Copy(Context(kCPU,0));
        auto bboxes = mx_executor_->outputs[2].Copy(Context(kCPU,0));
        auto masks = mx_executor_->outputs[3].Copy(Context(kCPU,0));
        NDArray::WaitAll();
        PostProcessOnMxnet(ids,scores,bboxes,masks,raw_img_size,x_scale,y_scale,objects);
        delete(mx_executor_);
        //MXNotifyShutdown();

    }
}


bool InstanceSeg::PostProcessOnOpenCV(const std::vector<cv::Mat> &outputs, cv::Size img_size,
                                      std::vector<std::shared_ptr<ImgObjectInfo>> &objects){
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
            std::shared_ptr<ImgObjectInfo> new_item = std::make_shared<ImgObjectInfo>();

            new_item->bbox = cv::Rect(cv::Point(left,top),cv::Point(right, bottom));
            new_item->img_size = img_size;
            new_item->score = score;
            new_item->class_id = class_id;
            cv::Mat tmp_mask = cv::Mat(mask_size_, mask_size_, CV_32F,
                                       mask_data+num_classes_*mask_size_*mask_size_*i/7+mask_size_*mask_size_*class_id);
            cv::Mat tmp_maskb = cv::Mat(mask_size_, mask_size_, CV_8U);
            int box_width = right - left;
            int box_height = bottom - top;
            cv::resize(tmp_mask, tmp_mask, cv::Size(box_width, box_height));
            cv::inRange(tmp_mask,  0.5, 100, tmp_maskb);
            new_item->mask = tmp_maskb;
            std::vector<std::vector<cv::Point>> contours;
            std::vector<cv::Vec4i> hierarchy;
            cv::findContours(tmp_maskb, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));
            new_item->mask_contours = contours;
            objects.push_back(new_item);
        }
    }
    return true;

}
 bool InstanceSeg::PostProcessOnMxnet(NDArray &ids, NDArray &scores, NDArray &bboxes, NDArray &masks, cv::Size img_size,
                                      float x_scale, float y_scale,
                                      std::vector<std::shared_ptr<ImgObjectInfo>> &objects) {
     int obj_num = bboxes.GetShape()[1];
     const float* mask_data = masks.GetData();
     for (int i = 0; i< obj_num; ++i)
     {
         float score = scores.At(0,0,i);
         float label = ids.At(0,0,i);
         if(score<conf_th_ || label < 0)
             continue;
         int cls_id = static_cast<int>(label);
         int left = bboxes.At(0, i, 0)*x_scale;
         int top = bboxes.At(0, i, 1)*y_scale;
         int right = bboxes.At(0, i, 2)*x_scale;
         int bottom = bboxes.At(0, i, 3)*y_scale;
         cv::Mat mask_mat = cv::Mat(mask_size_, mask_size_, CV_32F,
                                    const_cast<float*>(mask_data+mask_size_*mask_size_*i));
         int box_width = right - left;
         int box_height = bottom - top;
         cv::Rect box = cv::Rect(cv::Point(left, top),cv::Point(right,bottom));
         cv::resize(mask_mat, mask_mat, cv::Size(box_width, box_height));
         auto clipped_box = clip_bbox(box, img_size.width,img_size.height);
         cv::Rect offset = cv::Rect(clipped_box.x-box.x,clipped_box.y-box.y,clipped_box.width,clipped_box.height);
         std::cout<<box<<clipped_box<<offset<<std::endl;
         auto clipped_mask = mask_mat(offset);
         cv::Mat mask_b;
         cv::inRange(clipped_mask, 0.5, 2, mask_b);
         std::shared_ptr<ImgObjectInfo> new_item = std::make_shared<ImgObjectInfo>();
         new_item->bbox = clipped_box;
         new_item->img_size = img_size;
         new_item->score = score;
         new_item->class_id = cls_id;
         std::vector<std::vector<cv::Point>> contours;
         std::vector<cv::Vec4i> hierarchy;
         new_item->mask = mask_b;
         cv::findContours(mask_b, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));
         new_item->mask_contours = contours;
         objects.push_back(new_item);
     }
     return true;
 }

cv::Mat InstanceSeg::DrawObjects(cv::Mat img, std::vector<std::shared_ptr<ImgObjectInfo>> &objects) {
    std::mt19937 eng;
    std::uniform_real_distribution<float> rng(0, 1);
    float hue = rng(eng);
    for(auto object : objects)
    {
        auto class_label = object->class_id;
        if (colors_.find(class_label) == colors_.end()) {
            // create a new color
            int csize = static_cast<int>(num_classes_);
            if (csize > 0) {
                float hue =  class_label/ csize;
                colors_[class_label] = HSV2BGR(cv::Scalar(hue * 255, 0.75, 0.95));
            } else {
                // generate color for this id
                hue += 0.618033988749895;  // golden ratio
                hue = fmod(hue, 1.0);
                colors_[class_label] = HSV2BGR(cv::Scalar(hue * 255, 0.75, 0.95));
            }
        }
        auto color = colors_[class_label];
        auto box = object->bbox;
        cv::rectangle(img, box.tl(), box.br(), color, 2);
        cv::putText(img,std::to_string(object->instance_id),box.tl(),0,1,cv::Scalar(0,0,0));
        cv::Mat roi_img = img(box);
        cv::Mat mask = object->mask;
        cv::cvtColor(object->mask, mask, CV_GRAY2BGR);
        if(mask.size() != box.size())
        {
            std::cout<<mask.size()<<" vs "<<box.size()<<std::endl;

        }
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
