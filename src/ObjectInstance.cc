//
// Created by zuoxin on 18-3-12.
//

#include "ObjectInstance.h"
#include "MapPoint.h"
ObjectInstance::ObjectInstance(const int index,
                               const long unsigned int kf_index,
                                const std::shared_ptr<ImgObjectInfo>& pImgObjectInfo) {
    mIndex = index;
    mvKeyframeIndexes.push_back(kf_index);
    mmKeyFrameObjectInfo.insert(std::make_pair(kf_index,pImgObjectInfo));
    mClassId = pImgObjectInfo->class_id;

}
ObjectInstance::ObjectInstance(const int index,const std::shared_ptr<ImgObjectInfo>& pImgObjectInfo,
                               const long unsigned int kf_index,const std::vector<MapPoint*>& pvMapPoints) {
    mmKeyFrameObjectInfo.insert(std::make_pair(kf_index,pImgObjectInfo));
    mvKeyframeIndexes.push_back(kf_index);
    mvMapPoints=pvMapPoints;
    mIndex = index;
    for(auto map_pt:pvMapPoints) {
        mvMapPointIndexes.insert(map_pt->mnId);
    }
}
ObjectInstance::ObjectInstance(const int index) {
    mIndex = index;
}
void ObjectInstance::UpdateObjectInfo(const std::shared_ptr<ImgObjectInfo>& pImgObjectInfo, \
    const long unsigned int kf_index,const std::vector<MapPoint*>&pvMapPoints) {

    mmKeyFrameObjectInfo.insert(std::make_pair(kf_index,pImgObjectInfo));
    mvKeyframeIndexes.push_back(kf_index);
    for(auto map_pt:pvMapPoints)
    {
        if(mvMapPointIndexes.find(map_pt->mnId)==mvMapPointIndexes.end())
        {
            //mmKeyFrameObjectInfo.insert(std::make_pair(kf_index,map_pt));
            mvMapPointIndexes.insert(map_pt->mnId);
        }
    }
}
void ObjectInstance::UpdateObjectInfo(const std::shared_ptr<ImgObjectInfo>& pImgObjectInfo, \
    const long unsigned int kf_index) {

    mmKeyFrameObjectInfo.insert(std::make_pair(kf_index,pImgObjectInfo));
    mvKeyframeIndexes.push_back(kf_index);
}
