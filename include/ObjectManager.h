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
    explicit ObjectManager(int max_obj_num);
    ObjectManager(const ObjectInstance&) = delete;
    ObjectManager& operator=(const ObjectManager) = delete;
    int InsertNewObject(std::shared_ptr<ObjectInstance> obj);
    int MatchObjectInstances(std::shared_ptr<ObjectInstance> obj,Frame& pCurrentFrame,const Frame& nLastFrame,ORBmatcher& matcher);
    int BuildNewObjects(const Frame& pCurrentFrame,const std::vector<ImgObjectInfo>& pImgObjsInfo, \
    std::vector<shared_ptr<ObjectInstance>>& pObjInstances);

private:
    std::vector<std::shared_ptr<ObjectInstance>> mvObjectIntances;
    int mMaxObjNum;
    std::set<int> msObjectClasses;
    std::unordered_map<int,std::vector<int>> mClassInstanceIdMap;
    bool mFirstFrame = true;
    int mCurrentObjectIndex = 0;

};


#endif //ORB_SLAM2_OBJECTMANAGER_H
