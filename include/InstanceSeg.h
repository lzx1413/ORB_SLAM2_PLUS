//
// Created by zuoxin on 18-10-5.
//

#ifndef ORB_SLAM2_INSTANCESEG_H
#define ORB_SLAM2_INSTANCESEG_H

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <string>
#include <vector>

struct ObjectItem{
    cv::Rect box;
    cv::Mat mask;
    float score;
    int class_label;
    cv::Size img_size;
};


class InstanceSeg {
public:
    InstanceSeg();
    InstanceSeg(const std::string model_path, const std::string config_path, cv::Scalar mean, float scale, float nms_th, \
     float conf_th, const std::string name_file_path, int img_width = -1, int img_height = -1, int num_classes = 90):nms_th_(nms_th), \
     conf_th_(conf_th),scale_(scale),mean_(mean),img_height_(img_height), img_width_(img_width), num_classes_(num_classes) {
        ReadNamesFromFile(name_file_path, class_names_);
        InitModel(model_path, config_path);
    };
    ~InstanceSeg(){};
    bool InitModel(const std::string model_path, const std::string config_path);
    bool RunInstanceSeg(cv::Mat& img, std::vector<ObjectItem>& objects);
    cv::Mat DrawObjects(cv::Mat img, std::vector<ObjectItem>& objects);

private:
    bool ReadNamesFromFile(const std::string name_file_path, std::vector<std::string>& class_names);
    bool PostProcess(const std::vector<cv::Mat>& outputs, cv::Size img_size, std::vector<ObjectItem>& objects);
    float conf_th_ =  0.01;
    std::vector<std::string> class_names_;
    cv::Scalar mean_;
    float scale_;
    float nms_th_;
    int img_height_;
    int img_width_;
    int num_classes_;
    cv::dnn::Net network_;
};


#endif //ORB_SLAM2_INSTANCESEG_H
