//
// Created by zuoxin on 18-3-12.
//

#include "ObjectInstance.h"

ObjectInstance::ObjectInstance(const int index, const int class_id, const float score, const cv::Rect &rect,
                               const int kf_index, const std::vector<cv::Point2f> &img_kpts):mIndex(index) {
    mvClassIds.push_back(class_id);
    mvScores.push_back(score);
    mvRects.push_back(rect);
    mvKeyframeIndexs.push_back(kf_index);
    mvvImgKpts.push_back(img_kpts);

}
void ObjectInstance::UpdateObjectInfo(const int class_id, const float score, const cv::Rect rect, const int kf_index,
                                      const std::vector<cv::Point2f> &img_kpts) {
    mvClassIds.push_back(class_id);
    mvScores.push_back(score);
    mvRects.push_back(rect);
    mvKeyframeIndexs.push_back(kf_index);
    mvvImgKpts.push_back(img_kpts);

}
