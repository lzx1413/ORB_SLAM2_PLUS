//
// Created by zuoxin on 18-3-13.
//

#ifndef ORB_SLAM2_UTILS_H
#define ORB_SLAM2_UTILS_H
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

struct ImgObjectInfo{
    int class_id;
    float socre;
    cv::Rect bbox;
    std::vector<std::vector<cv::Point2f>> mask_contours;

};

int ParsingObjectInfoFromFile(const std::string & file_path,std::vector<ImgObjectInfo>& objs_info);

#endif //ORB_SLAM2_UTILS_H
