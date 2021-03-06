//
// Created by zuoxin on 18-3-13.
//
#include "utils.h"
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<unistd.h>

#include<opencv2/core/core.hpp>
#include "InstanceSeg.h"

#include<System.h>

void TestJsonParser()
{
    std::string json_path = "/home/zuoxin/workspace/detections/detectron/tum2_dk2_results/1305031543.743399.png.json";
    std::vector<std::shared_ptr<ImgObjectInfo>> img_obj_infos;
    ParsingObjectInfoFromFile(json_path,img_obj_infos);
    for(auto obj_info:img_obj_infos)
    {
        std::cout<<obj_info->score<<std::endl<<obj_info->class_id<<std::endl<<obj_info->bbox<<std::endl;
        for(auto mask :obj_info->mask_contours)
            std::cout<<mask[0]<<std::endl;
    }

}

using namespace std;

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    if(argc != 5)
    {
        cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association" << endl;
        return 1;
    }
    argv[1] = "/home/zuoxin/workspace/project/ORB_SLAM2_PLUS/Vocabulary/ORBvoc.bin";
    argv[2] = "/home/zuoxin/workspace/project/ORB_SLAM2_PLUS/Examples/RGB-D/TUM2.yaml";
    argv[3] = "/home/zuoxin/workspace/project/ORB_SLAM2_PLUS/data/TUM/rgbd_dataset_freiburg2_desk/rgbd_dataset_freiburg2_desk";
    argv[4] = "/home/zuoxin/workspace/project/ORB_SLAM2_PLUS/Examples/RGB-D/associations/fr2_desk.txt";

    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename = string(argv[4]);
    //string objResultPath = "/home/zuoxin/workspace/detections/detectron/tum2_dk2_results";

    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true,false, false);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imRGB, imD;

    // instance seg model

    for(int ni=0; ni<nImages; ni++) {
        // Read image and depthmap from file
        imRGB = cv::imread(string(argv[3]) + "/" + vstrImageFilenamesRGB[ni], CV_LOAD_IMAGE_UNCHANGED);
        imD = cv::imread(string(argv[3]) + "/" + vstrImageFilenamesD[ni], CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if (imRGB.empty()) {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }
        vector<std::shared_ptr<ImgObjectInfo>> objs_info;
        /*
        auto start = std::chrono::steady_clock::now();
        std::vector<std::shared_ptr<ImgObjectInfo>> objects;
        model.RunInstanceSeg(imRGB, objects);
        auto end = std::chrono::steady_clock::now();
        std::cout << "time cost: " << std::chrono::duration<double, std::milli>(end - start).count() << " ms";
        auto show_img = model.DrawObjects(imRGB, objects);
        cv::imshow("result", show_img);
        cv::waitKey();
         */

       //string json_path = objResultPath+"/"+vstrImageFilenamesRGB[ni]+".json";
       //if(ParsingObjectInfoFromFile(json_path,objs_info)==0) {
           //auto imObj = imRGB.clone();
           //ShowObjectOnOneImage(imObj,objs_info);
           //cv::imshow("objs",imObj);
           //cv::waitKey(10);
           //SLAM.SetCurrentObjsInfo(objs_info);
       //}

#ifdef COMPILEDWITHC11
       std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#endif

       // Pass the image to the SLAM system

       SLAM.TrackRGBD(imRGB,imD,tframe);

#ifdef COMPILEDWITHC11
       std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
       std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

       double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

       vTimesTrack[ni]=ttrack;

       // Wait to load the next frame
       double T=0;
       if(ni<nImages-1)
           T = vTimestamps[ni+1]-tframe;
       else if(ni>0)
           T = tframe-vTimestamps[ni-1];

       if(ttrack<T)
           std::this_thread::sleep_for(std::chrono::microseconds(static_cast<size_t>((T-ttrack)*1e6)));
   }

   // Stop all threads
   getchar();
   SLAM.Shutdown();

   // Tracking time statistics
   sort(vTimesTrack.begin(),vTimesTrack.end());
   float totaltime = 0;
   for(int ni=0; ni<nImages; ni++)
   {
       totaltime+=vTimesTrack[ni];
   }
   cout << "-------" << endl << endl;
   cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
   cout << "mean tracking time: " << totaltime/nImages << endl;

   // Save camera trajectory
   SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
   SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");


    return 0;
}

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}