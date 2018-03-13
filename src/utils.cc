//
// Created by zuoxin on 18-3-13.
//

#include "utils.h"
#include <jsoncpp/json/json.h>
#include <fstream>
#include <iostream>

int ParsingObjectInfoFromFile(const std::string & file_path,std::vector<ImgObjectInfo>& objs_info)
{
    std::ifstream ifs;
    ifs.open(file_path);
    if(!ifs.is_open())
    {
        std::cout<<"failed to open file "<<file_path<<std::endl;
        return -1;
    }
    Json::Reader reader;
    Json::Value root;
    if(!reader.parse(ifs,root,false))
    {
        std::cout<<"parsing error "<< file_path<<std::endl;
        return -2;
    }
    assert(root.isArray());
    for(int i = 0; i < root.size(); ++i)
    {
        Json::Value current = root[i];
        ImgObjectInfo new_obj_info;
        new_obj_info.class_id = current["class_id"].asInt();
        new_obj_info.socre = current["score"].asFloat();
        float x_min = current["bbox"][0].asFloat();
        float y_min = current["bbox"][1].asFloat();
        float x_max = current["bbox"][2].asFloat();
        float y_max = current["bbox"][3].asFloat();
        new_obj_info.bbox = cv::Rect(x_min,y_min,x_max-x_min,y_max-y_min);
        Json::Value json_masks = current["mask"];
        assert(json_masks.isArray());
        for(int mask_index = 0;mask_index<json_masks.size();++mask_index)
        {
            std::vector<cv::Point2f> contour;
            Json::Value json_mask = json_masks[mask_index];
            for(int j = 0;j<json_mask.size();++j)
            {
                Json::Value point = json_mask[j][0];
                cv::Point2f pt(point[0].asFloat(),point[1].asFloat());
                contour.push_back(pt);
            }
            new_obj_info.mask_contours.push_back(contour);
        }
        objs_info.push_back(new_obj_info);
    }
    return 0;

}
