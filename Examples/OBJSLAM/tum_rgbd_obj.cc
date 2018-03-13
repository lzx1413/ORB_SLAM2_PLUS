//
// Created by zuoxin on 18-3-13.
//
#include "utils.h"
void TestJsonParser()
{
    std::string json_path = "/home/zuoxin/workspace/detections/detectron/tum2_dk2_results/1305031543.743399.png.json";
    std::vector<ImgObjectInfo> img_obj_infos;
    ParsingObjectInfoFromFile(json_path,img_obj_infos);
    for(auto obj_info:img_obj_infos)
    {
        std::cout<<obj_info.socre<<std::endl<<obj_info.class_id<<std::endl<<obj_info.bbox<<std::endl;
        for(auto mask :obj_info.mask_contours)
            std::cout<<mask[0]<<std::endl;
    }

}
int main()
{
    std::cout<<"hello world"<<std::endl;
    TestJsonParser();
    return 0;

}