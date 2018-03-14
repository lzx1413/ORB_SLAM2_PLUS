//
// Created by zuoxin on 18-3-12.
//

#include "ObjectInstance.h"
#include "MapPoint.h"
ObjectInstance::ObjectInstance(int index,const int class_id, const float score, const cv::Rect &rect,
                               const long unsigned int kf_index, const std::vector<cv::KeyPoint> &img_kpts,const std::vector<MapPoint*>& pvMapPoints) {
    mvClassIds.push_back(class_id);
    mvScores.push_back(score);
    mvRects.push_back(rect);
    mvKeyframeIndexs.push_back(kf_index);
    mvvImgKpts=img_kpts;
    mvMapPoints=pvMapPoints;
    mIndex = index;
}
ObjectInstance::ObjectInstance(int index) {
    mIndex = index;
}
void ObjectInstance::UpdateObjectInfo(const int class_id, const float score, const cv::Rect rect, const long unsigned int kf_index,
                                      const std::vector<cv::KeyPoint> &img_kpts,const std::vector<MapPoint*>& pMapPoints) {
    mvClassIds.push_back(class_id);
    mvScores.push_back(score);
    mvRects.push_back(rect);
    mvKeyframeIndexs.push_back(kf_index);
    mvvImgKpts=img_kpts;
    mvMapPoints = pMapPoints;

}
