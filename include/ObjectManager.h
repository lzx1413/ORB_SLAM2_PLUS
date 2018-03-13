//
// Created by zuoxin on 18-3-12.
//

#ifndef ORB_SLAM2_OBJECTMANAGER_H
#define ORB_SLAM2_OBJECTMANAGER_H

#include <vector>
#include <memory>

class ObjectInstance;

class ObjectManager {
public:
    explicit ObjectManager(int max_obj_num);
    ObjectManager(const ObjectInstance&) = delete;
    ObjectManager& operator=(const ObjectManager) = delete;
    int InsertNewObject(std::shared_ptr<ObjectInstance> obj);
    int MatchObjectInstances(std::shared_ptr<ObjectInstance> obj);

private:
    std::vector<std::shared_ptr<ObjectInstance>> mvObjectIntances;
    int mMaxObjNum;


};


#endif //ORB_SLAM2_OBJECTMANAGER_H
