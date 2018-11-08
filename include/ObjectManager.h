//
// Created by zuoxin on 18-3-12.
//

#ifndef ORB_SLAM2_OBJECTMANAGER_H
#define ORB_SLAM2_OBJECTMANAGER_H

#include <vector>
#include <memory>
#include <set>
#include <unordered_map>
#include "Frame.h"
#include "ORBmatcher.h"
#include "utils.h"
using namespace ORB_SLAM2;

class ObjectInstance;

class ObjectManager {
public:
    ObjectManager()= default;
    explicit ObjectManager(int max_obj_num, int lost_num_thr, float match_thr)
    {
        mMaxObjNum = max_obj_num;
        mLostNumThr = lost_num_thr;
        mMatchTHr = match_thr;
    };
    int UpdateObjectInstances(Frame &pCurrentFrame, std::vector<std::shared_ptr<ImgObjectInfo>>& pImgObjsInfo);
    int InsertNewObject(std::shared_ptr<ObjectInstance> obj);
    int MatchObjectInstances(std::shared_ptr<ObjectInstance> obj,Frame& pCurrentFrame,const Frame& nLastFrame,ORBmatcher& matcher);
    int BuildNewObjects(const cv::Mat& img,const Frame& pCurrentFrame,const std::vector<std::shared_ptr<ImgObjectInfo>>& pImgObjsInfo);

private:
    std::vector<std::shared_ptr<ObjectInstance>> mvObjectIntances;
    std::unordered_map<int,std::shared_ptr<ObjectInstance>> mDroppedInstances;
    std::unordered_map<int,std::shared_ptr<ObjectInstance>> mTrackingInstances;
    int mMaxObjNum;
    std::set<int> msObjectClasses;
    std::unordered_map<int,std::vector<int>> mClassInstanceIdMap;//class->list of instance id
    std::unordered_map<int,int> mIndexLostnumMap;
    bool mFirstFrame = true;
    int mCurrentObjectIndex = 0;
    int mLostNumThr = 5;
    float mMatchTHr = 0.5;
    void ObjectProjection(KeyFrame* pKF, cv::Mat Scw, const std::vector<MapPoint*> & vpPoints, std::vector<cv::Point>& imPoints);
    void CaculateWorldPoints(KeyFrame* pKF, cv::Mat Scw, const std::vector<cv::Point>& imPoints, std::vector<MapPoint*> &vpPoints, cv::Mat & depth);
    float CalculateIOU(const cv::Rect& a, const cv::Rect& b);
};


#endif //ORB_SLAM2_OBJECTMANAGER_H
