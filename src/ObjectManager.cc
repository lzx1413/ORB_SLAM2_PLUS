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

float ObjectManager::CalculateIOU(const cv::Rect& a, const cv::Rect& b)
{
    int x_min = a.x>b.x?a.x:b.x;
    int x_max = a.x+a.width>b.x+b.width?b.x+b.width:a.x+a.width;
    int y_min = a.y>b.y?a.y:b.y;
    int y_max = a.y+a.height>b.y+b.height?b.y+b.height:a.y+a.height;
    float i_area = 0;
    if (x_min < x_max && y_min < y_max)
        i_area = (x_max-x_min)*(y_max-y_min);
    if(a.area()+b.area()-i_area == 0)
        return 0;
    return i_area/(a.area()+b.area()-i_area);

}

float ObjectManager::CalculateDist(const cv::Rect &a, const cv::Rect &b) {
    return sqrt((b.x-a.x)^2+(b.y-a.y)^2);
}
int ObjectManager::UpdateObjectInstances(Frame &pCurrentFrame,
                                         std::vector<std::shared_ptr<ImgObjectInfo>> &pImgObjsInfo) {
    if (pImgObjsInfo.size() == 0)
        return -1;
    // no object in the manager
    if (mCurrentObjectIndex == 0)
    {
        for(auto img_obj:pImgObjsInfo)
        {
            std::shared_ptr<ObjectInstance> newObjectInstance = std::make_shared<ObjectInstance>(
                    mCurrentObjectIndex,pCurrentFrame.mnId,img_obj);
            mTrackingInstances[mCurrentObjectIndex] = newObjectInstance;
            mIndexLostnumMap[mCurrentObjectIndex] = 0;
            img_obj->instance_id = mCurrentObjectIndex;
            mCurrentObjectIndex ++;
            if(msObjectClasses.find(newObjectInstance->GetClassId())==msObjectClasses.end()) {
                msObjectClasses.insert(newObjectInstance->GetClassId());
                std::vector<int> new_obj_set;
                new_obj_set.push_back(newObjectInstance->GetObjIndex());
                mClassInstanceIdMap[newObjectInstance->GetClassId()] = new_obj_set;
            } else{
                mClassInstanceIdMap[newObjectInstance->GetClassId()].push_back(newObjectInstance->GetObjIndex());
            }
        }
        return 0;
    }
    for(auto img_obj:pImgObjsInfo)
    {
        if(msObjectClasses.find(img_obj->class_id) == msObjectClasses.end())
        {
            std::cout<<"add now classes "<<img_obj->class_id<<std::endl;
            std::shared_ptr<ObjectInstance> newObjectInstance = std::make_shared<ObjectInstance>(
                    mCurrentObjectIndex,pCurrentFrame.mnId,img_obj);
            mTrackingInstances[mCurrentObjectIndex] = newObjectInstance;
            mIndexLostnumMap[mCurrentObjectIndex] = 0;
            img_obj->instance_id = mCurrentObjectIndex;
            mCurrentObjectIndex ++;
            msObjectClasses.insert(newObjectInstance->GetClassId());
            std::vector<int> new_obj_set;
            new_obj_set.push_back(newObjectInstance->GetObjIndex());
            mClassInstanceIdMap[newObjectInstance->GetClassId()] = new_obj_set;
        } else
        {
            int max_index = 0;
            float max_iou = 0;
            for(int ins_id : mClassInstanceIdMap[img_obj->class_id])
            {
                if (mTrackingInstances.find(ins_id) == mTrackingInstances.end())
                    continue;
                float iou = CalculateIOU(mTrackingInstances[ins_id]->GetLastRect(),img_obj->bbox);
                if (iou>max_iou)
                {
                    max_iou = iou;
                    max_index = ins_id;
                }


            }
            if (max_iou>mMatchTHr)
            {
                mTrackingInstances[max_index]->UpdateObjectInfo(img_obj,pCurrentFrame.mnId);
                img_obj->instance_id = max_index;
            } else
            {
                std::shared_ptr<ObjectInstance> newObjectInstance = std::make_shared<ObjectInstance>(
                        mCurrentObjectIndex,pCurrentFrame.mnId,img_obj);
                mTrackingInstances[mCurrentObjectIndex] = newObjectInstance;
                mIndexLostnumMap[mCurrentObjectIndex] = 0;
                img_obj->instance_id = mCurrentObjectIndex;
                mCurrentObjectIndex ++;
                mClassInstanceIdMap[newObjectInstance->GetClassId()].push_back(newObjectInstance->GetObjIndex());
            }
        }

    }
    for(auto ins : mTrackingInstances)
    {
        if(ins.second->GetLastFrameID() != pCurrentFrame.mnId)
        {
            mIndexLostnumMap[ins.second->GetObjIndex()] ++;
        } else
            mIndexLostnumMap[ins.second->GetObjIndex()] ==0;
    }

    for (auto item : mIndexLostnumMap)
    {
        if(item.second > mLostNumThr)
        {
            mDroppedInstances[item.first] = mTrackingInstances[item.first];
            mTrackingInstances.erase(item.first);
        }
    }
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

void ObjectManager::ObjectProjection(KeyFrame* pKF, cv::Mat Scw, const std::vector<MapPoint *> &vpPoints,
                                     std::vector<cv::Point> &imPoints) {
    const float &fx = pKF->fx;
    const float &fy = pKF->fy;
    const float &cx = pKF->cx;
    const float &cy = pKF->cy;
    cv::Mat sRcw = Scw.rowRange(0,3).colRange(0,3);
    const float scw = sqrt(sRcw.row(0).dot(sRcw.row(0)));
    cv::Mat Rcw = sRcw/scw;
    cv::Mat tcw = Scw.rowRange(0,3).col(3)/scw;
    cv::Mat Ow = -Rcw.t()*tcw;
    for(int iMP=0, iendMP=vpPoints.size(); iMP<iendMP; iMP++) {
        MapPoint *pMP = vpPoints[iMP];

        // Get 3D Coords.
        cv::Mat p3Dw = pMP->GetWorldPos();

        // Transform into Camera Coords.
        cv::Mat p3Dc = Rcw * p3Dw + tcw;

        // Depth must be positive
        if (p3Dc.at<float>(2) < 0.0)
            continue;

        // Project into Image
        const float invz = 1 / p3Dc.at<float>(2);
        const float x = p3Dc.at<float>(0) * invz;
        const float y = p3Dc.at<float>(1) * invz;

        const float u = fx * x + cx;
        const float v = fy * y + cy;
        if (!pKF->IsInImage(u, v))
            continue;
        imPoints.push_back(cv::Point(u, v));
    }
}

void ObjectManager::CaculateWorldPoints(KeyFrame *pKF, cv::Mat Scw, const std::vector<cv::Point> &imPoints,
                                        std::vector<MapPoint *> &vpPoints, cv::Mat &depth) {
   ;


}

