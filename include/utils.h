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
    std::vector<std::vector<cv::Point>> mask_contours;
    void AppendMaskContour(const std::vector<cv::Point>& mask_c)
    {
        std::vector<cv::Point> poly_c;
        cv::approxPolyDP(mask_c,poly_c,0.1,false);
        std::cout<<"raw "<<mask_c.size()<<" poly "<<poly_c.size()<<std::endl;
        mask_contours.push_back(poly_c);

    }

};

int ParsingObjectInfoFromFile(const std::string & file_path,std::vector<ImgObjectInfo>& objs_info);
void ShowObjectOnOneImage(cv::Mat& img,const std::vector<ImgObjectInfo>& objs_info);

#endif //ORB_SLAM2_UTILS_H
