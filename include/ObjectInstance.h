//
// Created by zuoxin on 18-3-12.
//

#ifndef ORB_SLAM2_OBJECTINSTANCE_H
#define ORB_SLAM2_OBJECTINSTANCE_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <queue>
class ObjectInstance {
public:
    explicit ObjectInstance(const int index,const int class_id,const float score,const cv::Rect& rect,const int kf_index,const std::vector<cv::Point2f>& img_kpts);
    void UpdateObjectInfo(const int class_id,const float score,const cv::Rect rect,const int kf_index,const std::vector<cv::Point2f>& img_kpts);

private:
    int mIndex;
    std::vector<int> mvKeyframeIndexs;
    std::vector<float> mvScores;
    std::vector<int> mvClassIds;
    std::vector<cv::Rect> mvRects;
    std::vector<std::vector<cv::Point2f>> mvvImgKpts;
    std::vector<cv::Point3f> mvClpoints;

};


#endif //ORB_SLAM2_OBJECTINSTANCE_H
