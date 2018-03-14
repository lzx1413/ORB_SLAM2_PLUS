//
// Created by zuoxin on 18-3-12.
//
#include <algorithm>
#include "ObjectManager.h"
#include "ObjectInstance.h"
#include "Frame.h"
#include "ORBmatcher.h"
#include <queue>
using namespace ORB_SLAM2;

ObjectManager::ObjectManager(int max_obj_num):mMaxObjNum(max_obj_num) {}

int ObjectManager::InsertNewObject(std::shared_ptr<ObjectInstance> obj) {
    if(mvObjectIntances.size()>=mMaxObjNum)
        return -1;
    else
    {
        if(msObjectClasses.find(obj->GetClassId())==msObjectClasses.end()) {
            msObjectClasses.insert(obj->GetClassId());
            std::vector<int> new_obj_set;
            new_obj_set.push_back(obj->GetObjIndex());
            mClassInstanceIdMap[obj->GetClassId()] = new_obj_set;
        } else{
            mClassInstanceIdMap[obj->GetClassId()].push_back(obj->GetObjIndex());
        }

        mvObjectIntances.push_back(obj);
    }
}
/*
 * -1 for new object,-2 for error,positive number for matched existing objects.
 * project the 3d points in the former frame to the current frame, match them to identify the same object instance with\
 * same label_id;
 */
int ObjectManager::MatchObjectInstances(std::shared_ptr<ObjectInstance> obj,Frame& pCurrentFrame,const Frame& pLastFrame,ORBmatcher& matcher) {
    if(mvObjectIntances.size()==0)
        return -1;
    if(msObjectClasses.find(obj->GetClassId())==msObjectClasses.end())
        return -1;
}

static std::vector<int>  GetTopKNum(const std::vector<int>& nums,int k = 2)
{
    std::unordered_map<int,int> hist;
    std::priority_queue<std::pair<int,int>> q;
    std::vector<int> res;
    for(auto num:nums)
        hist[num]++;
    for(auto it:hist)
        q.push({it.second,it.first});
    for(int i= 0;i<k;++i)
    {
        res.push_back(q.top().second);
        q.pop();
    }
    return res;
}
int ObjectManager::BuildNewObjects(const Frame& pCurrentFrame,const std::vector<ImgObjectInfo>& pImgObjsInfo, \
    std::vector<shared_ptr<ObjectInstance>>& pObjInstances) {
    for (auto img_obj:pImgObjsInfo) {
        std::vector<cv::KeyPoint> ORB_kps;
        std::vector<MapPoint *> map_points;
        //TODO:fast way to find the related keypoints
        std::vector<int> mappoint_labels;
        for (auto mask : img_obj.mask_contours) {
            for (int i = 0; i < pCurrentFrame.mvKeys.size(); ++i) {
                cv::Point2f kp = pCurrentFrame.mvKeys[i].pt;
                if (cv::pointPolygonTest(mask, kp, false) < 1)//判断mask内部特征点
                {
                    if (pCurrentFrame.mvpMapPoints[i] != NULL) {
                        if (!pCurrentFrame.mvbOutlier[i] && pCurrentFrame.mvpMapPoints[i]->Observations() > 0) {
                            ORB_kps.push_back(pCurrentFrame.mvKeys[i]);
                            map_points.push_back(pCurrentFrame.mvpMapPoints[i]);
                            if(!mFirstFrame)
                                mappoint_labels.push_back(pCurrentFrame.mvpMapPoints[i]->mObjectIndex);
                        }
                    }


                }
            }
        }
        if (mFirstFrame){
            auto newObjectInstance = std::make_shared<ObjectInstance>(mCurrentObjectIndex++);
            for(auto map_point:map_points)
                map_point->mObjectIndex = newObjectInstance->GetObjIndex();
            newObjectInstance->UpdateObjectInfo(img_obj.class_id,img_obj.socre,img_obj.bbox,pCurrentFrame.mnId,ORB_kps,map_points);
        } else{
            auto top_labels = GetTopKNum(mappoint_labels,2);
            //TODO: more judge to the fuse condition
            int match_label = top_labels[0];
            if(match_label>0 && img_obj.class_id == )
            {

            }
        }

            std::cout << pCurrentFrame.mvKeys.size() << " kp num " << ORB_kps.size() << std::endl;
    }
    if(mFirstFrame&&mvObjectIntances.size()>0)
        mFirstFrame = false;
}

