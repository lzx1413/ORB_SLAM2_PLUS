//
// Created by zuoxin on 18-10-6.
//

#include "InstanceSeg.h"
#include <opencv2/opencv.hpp>

void test_opencv()
{
    cv::Mat img = cv::imread("/home/zuoxin/workspace/project/ORB_SLAM2_PLUS/Examples/RGB-D/2.jpg");
    cv::imshow("img",img);
    cv::waitKey();
    std::string model_path = "/home/zuoxin/workspace/project/ORB_SLAM2_PLUS/Examples/RGB-D/model/maskrcnn_inceptionv2_coco.pb";
    std::string config_path = "/home/zuoxin/workspace/project/ORB_SLAM2_PLUS/Examples/RGB-D/model/mask_rcnn_inception_v2_coco.pxtxt";
    cv::Scalar mean{0,0,0};
    std::string name_file_path = "/home/zuoxin/workspace/project/ORB_SLAM2_PLUS/Examples/RGB-D/coco_names.txt";
    InstanceSeg model("opencv", model_path,config_path,false,mean,1,0.45,0.3,name_file_path,600,800,90,15);
    std::vector<ObjectItem> objects;
    model.RunInstanceSeg(img, objects);
    auto show_img = model.DrawObjects(img, objects);
    cv::imshow("result", show_img);
    cv::waitKey();
}
void test_mxnet()
{
    cv::Mat img = cv::imread("/home/zuoxin/workspace/project/ORB_SLAM2_PLUS/data/TUM/rgbd_dataset_freiburg2_desk/rgbd_dataset_freiburg2_desk/rgb/1311868164.463055.png");
    std::string model_path = "/home/zuoxin/workspace/project/ORB_SLAM2_PLUS/Examples/RGB-D/mask_rcnn_resnet18_v1b_tum-0000.params";
    std::string config_path = "/home/zuoxin/workspace/project/ORB_SLAM2_PLUS/Examples/RGB-D/mask_rcnn_resnet18_v1b_tum-symbol.json";
    cv::Scalar mean{0,0,0};
    std::string name_file_path = "/home/zuoxin/workspace/project/ORB_SLAM2_PLUS/Examples/RGB-D/coco_names.txt";
    InstanceSeg model("mxnet", model_path,config_path,true,mean,1,0.45,0.4,name_file_path,500,800,9,14);
    std::vector<ObjectItem> objects;
    auto start = std::chrono::steady_clock::now();
    model.RunInstanceSeg(img, objects);
    auto end  = std::chrono::steady_clock::now();
    std::cout<<"time cost: "<<std::chrono::duration<double ,std::milli>(end - start).count()<<" ms";
    auto show_img = model.DrawObjects(img, objects);
    cv::imshow("result", show_img);
    cv::waitKey();


}
int main()
{
    test_mxnet();
}

