//
// Created by zuoxin on 18-3-13.
//

#include "utils.h"
#include <jsoncpp/json/json.h>
#include <fstream>
#include <iostream>
#include <memory>
#include "mxnet_utils.h"
std::map<int,cv::Scalar> colors_;
int num_classes_ = 9;
int ParsingObjectInfoFromFile(const std::string & file_path,std::vector<std::shared_ptr<ImgObjectInfo>>& objs_info)
{
    std::vector<int> allow_classes{40,67,66};
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
        int class_id = current["class_id"].asInt();
        if(std::find(allow_classes.begin(),allow_classes.end(),class_id)==allow_classes.end())
            continue;
        std::shared_ptr<ImgObjectInfo> new_obj_info = std::make_shared<ImgObjectInfo>();
        new_obj_info->class_id = class_id;
        new_obj_info->score = current["score"].asFloat();
        float x_min = current["bbox"][0].asFloat();
        float y_min = current["bbox"][1].asFloat();
        float x_max = current["bbox"][2].asFloat();
        float y_max = current["bbox"][3].asFloat();
        new_obj_info->bbox = cv::Rect(x_min,y_min,x_max-x_min,y_max-y_min);
        Json::Value json_masks = current["mask"];
        assert(json_masks.isArray());
        for(int mask_index = 0;mask_index<json_masks.size();++mask_index)
        {
            std::vector<cv::Point> contour;
            Json::Value json_mask = json_masks[mask_index];
            for(int j = 0;j<json_mask.size();++j)
            {
                Json::Value point = json_mask[j][0];
                cv::Point pt(point[0].asInt(),point[1].asInt());
                contour.push_back(pt);
            }
            new_obj_info->AppendMaskContour(contour);
        }
        objs_info.push_back(new_obj_info);
    }
    return 0;
}

cv::Scalar GetLabelColor(int class_label)
{
    std::mt19937 eng;
    std::uniform_real_distribution<float> rng(0, 1);
    float hue = rng(eng);
    if (colors_.find(class_label) == colors_.end()) {
        // create a new color
        int csize = static_cast<int>(num_classes_);
        if (csize > 0) {
            float hue =  float(class_label)/ float(csize);
            colors_[class_label] = HSV2BGR(cv::Scalar(hue * 255, 0.75, 0.95));
        } else {
            // generate color for this id
            hue += 0.618033988749895;  // golden ratio
            hue = fmod(hue, 1.0);
            colors_[class_label] = HSV2BGR(cv::Scalar(hue * 255, 0.75, 0.95));
        }
    }
    auto color = colors_[class_label];
    return color;

}
void ShowObjectOnOneImage(cv::Mat& img,const std::vector<std::shared_ptr<ImgObjectInfo>>& objects)
{
    for(auto object : objects)
    {
        auto class_label = object->class_id;
        auto box = object->bbox;
        auto color = GetLabelColor(class_label);
        cv::rectangle(img, box.tl(), box.br(), color, 2);
        //cv::putText(img,std::to_string(object->instance_id),box.tl(),0,1,cv::Scalar(0,0,0));
        cv::Mat roi_img = img(box);
        cv::drawContours(roi_img,object->mask_contours,-1,color,-1);
        cv::Mat mask = object->mask;
        cv::cvtColor(object->mask, mask, CV_GRAY2BGR);
        if(mask.size() != box.size())
        {
            std::cout<<mask.size()<<" vs "<<box.size()<<std::endl;

        }
        cv::addWeighted(roi_img, 0.5, mask, 0.5, 0, roi_img);
    }
}
