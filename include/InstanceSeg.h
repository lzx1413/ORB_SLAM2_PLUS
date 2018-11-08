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
#include "mxnet-cpp/MxNetCpp.h"
#include "utils.h"
using namespace mxnet::cpp;


class InstanceSeg {
public:
    InstanceSeg() = default;
    InstanceSeg(const std::string net_type, const std::string model_path, const std::string config_path, bool use_gpu,
                cv::Scalar mean, float scale, float nms_th, float conf_th, const std::string name_file_path,
                int min_size = -1, int max_size = -1, int num_classes = 90, int mask_size = 14):nms_th_(nms_th), \
     conf_th_(conf_th),scale_(scale),mean_(mean),min_size_(min_size), max_size_(max_size), num_classes_(num_classes),
                                                                                           net_type_(net_type),
                                                                                           use_gpu_(use_gpu),
                                                                                           mask_size_(mask_size){
        ReadNamesFromFile(name_file_path, class_names_);
        InitModel(model_path, config_path);
    };
    ~InstanceSeg(){};
    bool InitModel(const std::string model_path, const std::string config_path);
    bool RunInstanceSeg(cv::Mat img, std::vector<std::shared_ptr<ImgObjectInfo>>& objects);
    cv::Mat DrawObjects(cv::Mat img, std::vector<std::shared_ptr<ImgObjectInfo>>& objects);

private:
    bool ReadNamesFromFile(const std::string name_file_path, std::vector<std::string>& class_names);
    bool PostProcessOnOpenCV(const std::vector<cv::Mat> &outputs, cv::Size img_size, std::vector<std::shared_ptr<ImgObjectInfo>> &objects);
    bool PostProcessOnMxnet(NDArray& ids,NDArray& scores, NDArray& bboxes, NDArray& masks,cv::Size img_size,float x_scale, float y_scale, std::vector<std::shared_ptr<ImgObjectInfo>> &objects);
    float conf_th_ =  0.01;
    std::vector<std::string> class_names_;
    cv::Scalar mean_;
    float scale_;
    float nms_th_;
    int min_size_;
    int max_size_;
    int num_classes_;
    int mask_size_;
    // opencv
    std::string net_type_;
    cv::dnn::Net network_;
    // mxnet
    bool use_gpu_;
    Context ctx_ = Context::cpu();
    std::map<std::string, NDArray> args_, auxs_;
    Symbol mx_net_;
    //display
    std::map<int,cv::Scalar> colors_;

};


#endif //ORB_SLAM2_INSTANCESEG_H
