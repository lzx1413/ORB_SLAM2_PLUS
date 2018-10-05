//
// Created by zuoxin on 18-3-12.
//

#ifndef ORB_SLAM2_OBJECTINSTANCE_H
#define ORB_SLAM2_OBJECTINSTANCE_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <queue>
#include "MapPoint.h"
#include "utils.h"
using namespace ORB_SLAM2;
class ObjectInstance {
public:
    explicit ObjectInstance(const int index);
    explicit ObjectInstance(const int index,const std::shared_ptr<ImgObjectInfo>& pImgObjectInfo, \
    const long unsigned int kf_index,const std::vector<MapPoint*>& pvMapPoints);

    void UpdateObjectInfo(const std::shared_ptr<ImgObjectInfo>& pImgObjectInfo, \
    const long unsigned int kf_index,const std::vector<MapPoint*>&pvMapPoints);
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
    std::vector<long unsigned int> mvKeyframeIndexes;
    std::set<long unsigned  int>mvMapPointIndexes;
    std::vector<MapPoint*> mvMapPoints;
    std::unordered_map<long unsigned int,std::shared_ptr<ImgObjectInfo>> mmKeyFrameOBjectInfo;

};


#endif //ORB_SLAM2_OBJECTINSTANCE_H
