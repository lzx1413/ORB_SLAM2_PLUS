//
// Created by zuoxin on 18-3-12.
//

#ifndef ORB_SLAM2_OBJECTINSTANCE_H
#define ORB_SLAM2_OBJECTINSTANCE_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <queue>
#include "MapPoint.h"
using namespace ORB_SLAM2;
class ObjectInstance {
public:
    explicit ObjectInstance(int index);
    explicit ObjectInstance(const int index,const int class_id,const float score,const cv::Rect& rect,const long unsigned int kf_index,\
    const std::vector<cv::KeyPoint>& img_kpts,const std::vector<MapPoint*>& pvMapPoints);
    void UpdateObjectInfo(const int class_id,const float score,const cv::Rect rect,const long unsigned int kf_index,const std::vector<cv::KeyPoint>& img_kpts,const std::vector<MapPoint*>& \
pvMapPoints);
    int GetClassId()
    {
        return mClassId;
    }
    std::vector<MapPoint*> GetMapPoints()
    {
        return mvMapPoints;
    }
    int GetObjIndex()
    {
        return mIndex;
    }


private:
    int mIndex;
    int mClassId;
    std::vector<long unsigned int> mvKeyframeIndexs;
    std::vector<float> mvScores;
    std::vector<int> mvClassIds;
    std::vector<cv::Rect> mvRects;
    std::vector<cv::KeyPoint> mvvImgKpts;
    std::vector<MapPoint*> mvMapPoints;

};


#endif //ORB_SLAM2_OBJECTINSTANCE_H
