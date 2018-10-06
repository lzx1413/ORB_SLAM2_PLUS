//
// Created by zuoxin on 18-10-6.
//

#include "InstanceSeg.h"
#include <opencv2/opencv.hpp>

int main()
{
    cv::Mat img = cv::imread("/home/zuoxin/workspace/project/ORB_SLAM2_PLUS/Examples/RGB-D/2.jpg");
    cv::imshow("img",img);
    cv::waitKey();
    std::string model_path = "/home/zuoxin/workspace/project/ORB_SLAM2_PLUS/Examples/RGB-D/model/maskrcnn_inceptionv2_coco.pb";
    std::string config_path = "/home/zuoxin/workspace/project/ORB_SLAM2_PLUS/Examples/RGB-D/model/mask_rcnn_inception_v2_coco.pxtxt";
    cv::Scalar mean{0,0,0};
    std::string name_file_path = "/home/zuoxin/workspace/project/ORB_SLAM2_PLUS/Examples/RGB-D/coco_names.txt";
    InstanceSeg model(model_path, config_path, mean,1, 0.5, 0.5, name_file_path, 800, 800);
    std::vector<ObjectItem> objects;
    model.RunInstanceSeg(img, objects);
    auto show_img = model.DrawObjects(img, objects);
    cv::imshow("result", show_img);
    cv::waitKey();
}

