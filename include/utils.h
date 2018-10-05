//
// Created by zuoxin on 18-3-13.
//

#ifndef ORB_SLAM2_UTILS_H
#define ORB_SLAM2_UTILS_H
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <memory>
//std::vector<int> allow_classes{};
struct ImgObjectInfo{
    int class_id;
    float socre;
    cv::Rect bbox;
    std::vector<std::vector<cv::Point>> mask_contours;
    std::vector<cv::KeyPoint> key_points;
    long unsigned int frame_id;
    int object_id = -1;
    void AppendMaskContour(const std::vector<cv::Point>& mask_c)
    {
        std::vector<cv::Point> poly_c;
        cv::approxPolyDP(mask_c,poly_c,0.1,false);
        mask_contours.push_back(poly_c);

    }

};

int ParsingObjectInfoFromFile(const std::string & file_path,std::vector<std::shared_ptr<ImgObjectInfo>>& objs_info);
void ShowObjectOnOneImage(cv::Mat& img,const std::vector<std::shared_ptr<ImgObjectInfo>>& objs_info);

#endif //ORB_SLAM2_UTILS_H
