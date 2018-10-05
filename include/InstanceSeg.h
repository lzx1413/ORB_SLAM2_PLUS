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
};


class InstanceSeg {
public:
    InstanceSeg();
    InstanceSeg(const std::string model_path, const std::string config_path, cv::Scalar mean, float scale, float nms_th, \
     float conf_th, const std::string name_file_path, int img_width = -1, int img_height = -1):nms_th_(nms_th), \
     conf_th_(conf_th),scale_(scale),mean_(mean),img_height_(img_height), img_width_(img_width) {
        class_names_ =  ReadNamesFromFile(name_file_path);
        InitModel(model_path, config_path);
    };
    ~InstanceSeg();
    bool InitModel(const std::string model_path, const std::string config_path);
    bool RunInstanceSeg(cv::Mat& img, std::vector<cv::Rect>& bboxes, std::vector<cv::Mat>& masks);

private:
    std::vector<std::string> ReadNamesFromFile(const std::string name_file_path);
    bool PostProcess(std::vector<std::vector<cv::Mat>>& outputs, std::vector<cv::Rect>);
    float conf_th_ =  0.01;
    std::vector<std::string> class_names_;
    cv::Scalar mean_;
    float scale_;
    float nms_th_;
    int img_height_;
    int img_width_;
    cv::dnn::Net network_;

};


#endif //ORB_SLAM2_INSTANCESEG_H
