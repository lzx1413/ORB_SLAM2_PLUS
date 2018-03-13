//
// Created by zuoxin on 18-3-12.
//

#include "ObjectManager.h"
#include "ObjectInstance.h"

ObjectManager::ObjectManager(int max_obj_num):mMaxObjNum(max_obj_num) {}

int ObjectManager::InsertNewObject(std::shared_ptr<ObjectInstance> obj) {
    if(mvObjectIntances.size()>=mMaxObjNum)
        return -1;
    else
    {
        mvObjectIntances.push_back(obj);
    }
}
