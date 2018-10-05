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

static void ShowNewObject(cv::Mat img, const std::vector<std::shared_ptr<ImgObjectInfo>>& img_obj,int index)
{
    cv::Mat show_img = img.clone();
    for(auto obj:img_obj) {
        for (auto pt:obj->key_points) {
            cv::circle(show_img, pt.pt, 2, cv::Scalar(255, 0, 0), -1);
        }
        cv::putText(show_img,std::to_string(obj->object_id),obj->bbox.tl(),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,0,0),1,8);
    }
    ShowObjectOnOneImage(show_img,img_obj);
    cv::imwrite(std::to_string(index)+".jpg",show_img);
    //cv::waitKey(-1);
}
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
    if(nums.size()<k)
        return std::vector<int>{-1,-1};
    std::unordered_map<int,int> hist;
    std::priority_queue<std::pair<int,int>> q;
    std::vector<int> res;
    for(auto num:nums)
        hist[num]++;
    for(auto it:hist)
        q.push({it.second,it.first});
    if(q.size()>=k) {
        for (int i = 0; i < k; ++i) {
            res.push_back(q.top().second);
            q.pop();
        }
    } else{
        res.push_back(q.top().second);
        res.push_back(q.top().second);
    }
    return res;
}
int ObjectManager::BuildNewObjects(const cv::Mat& img,const Frame& pCurrentFrame,const std::vector<std::shared_ptr<ImgObjectInfo>>& pImgObjsInfo) {
    if(pImgObjsInfo.size()==0)
        return -1;
    for (auto img_obj:pImgObjsInfo) {
        std::vector<cv::KeyPoint> ORB_kps;
        std::vector<MapPoint *> map_points;
        //TODO:fast way to find the related keypoints
        std::vector<int> mappoint_indexes;
        for (auto mask : img_obj->mask_contours) {
            for (int i = 0; i < pCurrentFrame.mvKeys.size(); ++i) {
                cv::Point2f kp = pCurrentFrame.mvKeys[i].pt;
                if (cv::pointPolygonTest(mask, kp, false) >-1)//判断mask内部特征点
                {
                    //TODO: handle the situation where the pints are rare
                    if (pCurrentFrame.mvpMapPoints[i] != NULL) {
                        if (!pCurrentFrame.mvbOutlier[i] && pCurrentFrame.mvpMapPoints[i]->Observations() > 0) {
                            ORB_kps.push_back(pCurrentFrame.mvKeys[i]);
                            map_points.push_back(pCurrentFrame.mvpMapPoints[i]);
                            if(!mFirstFrame)
                                mappoint_indexes.push_back(pCurrentFrame.mvpMapPoints[i]->mObjectIndex);
                        }
                    }


                }

            }
        }
        img_obj->frame_id = pCurrentFrame.mnId;
        img_obj->key_points = ORB_kps;
        if (mFirstFrame){
            for(auto map_point:map_points) {
                map_point->mObjectIndex = mCurrentObjectIndex;
            }
            std::shared_ptr<ObjectInstance> newObjectInstance = std::make_shared<ObjectInstance>(
                    mCurrentObjectIndex, img_obj, pCurrentFrame.mnId, map_points);
            mvObjectIntances.push_back(newObjectInstance);
            img_obj->object_id = mCurrentObjectIndex;
            std::cout<<"add new object "<<mCurrentObjectIndex<<std::endl;
            mCurrentObjectIndex++;

        } else{
            auto top_indexes = GetTopKNum(mappoint_indexes,2);
            //TODO: more judgement to the fuse condition
            int match_index = top_indexes[0];
            //existing object instance,fuse them together
            if(match_index>=0 && img_obj->class_id ==mvObjectIntances[match_index]->GetClassId() )
            {
                mvObjectIntances[match_index]->UpdateObjectInfo(img_obj,pCurrentFrame.mnId,map_points);
                img_obj->object_id = match_index;

            } else{
                //new object instance
                for(auto map_point:map_points) {
                    map_point->mObjectIndex = mCurrentObjectIndex;
                }
                std::shared_ptr<ObjectInstance> newObjectInstance = std::make_shared<ObjectInstance>(
                        mCurrentObjectIndex, img_obj, pCurrentFrame.mnId, map_points);
                mvObjectIntances.push_back(newObjectInstance);
                img_obj->object_id = mCurrentObjectIndex;
                std::cout<<"add new object "<<mCurrentObjectIndex<<std::endl;
                mCurrentObjectIndex++;
            }
        }

            std::cout << pCurrentFrame.mvKeys.size() << " kp num " << ORB_kps.size() << std::endl;
    }
    if(mFirstFrame&&mvObjectIntances.size()>0)
        mFirstFrame = false;
    ShowNewObject(img,pImgObjsInfo,pCurrentFrame.mnId);
}

