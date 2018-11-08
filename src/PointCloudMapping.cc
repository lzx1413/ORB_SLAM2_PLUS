/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

#include "PointCloudMapping.h"
#include <KeyFrame.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include "Converter.h"
namespace ORB_SLAM2 {
    PointCloudMapping::PointCloudMapping(double resolution_, int enable_object_) {
        this->resolution = resolution_;
        voxel.setLeafSize(resolution, resolution, resolution);
        enable_object = enable_object_;
        globalMap = boost::make_shared<PointCloud>();

        viewerThread = make_shared<thread>(bind(&PointCloudMapping::viewer, this));
    }

    void PointCloudMapping::shutdown() {
        {
            unique_lock<mutex> lock(shutDownMutex);
            shutDownFlag = true;
            keyFrameUpdated.notify_one();
        }
        viewerThread->join();
    }

    void PointCloudMapping::insertKeyFrame(KeyFrame *kf, cv::Mat &color, cv::Mat &depth) {
        //cout << "receive a keyframe, id = " << kf->mnId << endl;
        unique_lock<mutex> lock(keyframeMutex);
        //cout << "kf push_back" << endl;
        keyframes.push_back(kf);
        //cout << "color push_back" << endl;
        colorImgs.push_back(color.clone());
        //cout << "depth push_back" << endl;
        depthImgs.push_back(depth.clone());

        keyFrameUpdated.notify_one();
    }
    void PointCloudMapping::insertKeyFrame(KeyFrame *kf, cv::Mat &color, cv::Mat &depth, std::vector<std::shared_ptr<ImgObjectInfo>>& objects) {
        //cout << "receive a keyframe, id = " << kf->mnId << endl;
        unique_lock<mutex> lock(keyframeMutex);
        //cout << "kf push_back" << endl;
        keyframes.push_back(kf);
        //cout << "color push_back" << endl;
        colorImgs.push_back(color.clone());
        //cout << "depth push_back" << endl;
        depthImgs.push_back(depth.clone());
        object_list.push_back(objects);

        keyFrameUpdated.notify_one();
    }

    pcl::PointCloud<PointCloudMapping::PointT>::Ptr
    PointCloudMapping::generatePointCloud(KeyFrame *kf, cv::Mat &color, cv::Mat &depth) {
        PointCloud::Ptr tmp(new PointCloud());
        // point cloud is null ptr
        for (int m = 0; m < depth.rows; m += 3) {
            for (int n = 0; n < depth.cols; n += 3) {
                float d = depth.ptr<float>(m)[n];
                if (d < 0.01 || d > 10)
                    continue;
                PointT p;
                p.z = d;
                p.x = (n - kf->cx) * p.z / kf->fx;
                p.y = (m - kf->cy) * p.z / kf->fy;
                p.b = color.ptr<uchar>(m)[n * 3];
                p.g = color.ptr<uchar>(m)[n * 3 + 1];
                p.r = color.ptr<uchar>(m)[n * 3 + 2];
                tmp->points.push_back(p);
            }
        }

        Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat(kf->GetPose());
        PointCloud::Ptr cloud(new PointCloud);
        pcl::transformPointCloud(*tmp, *cloud, T.inverse().matrix());
        cloud->is_dense = false;

        cout << "generate point cloud for kf " << kf->mnId << ", size=" << cloud->points.size() << endl;
        return cloud;
    }

    pcl::PointCloud<PointCloudMapping::PointT>::Ptr
    PointCloudMapping::generatePointCloud(KeyFrame *kf, cv::Mat &color, cv::Mat &depth, std::vector<std::shared_ptr<ImgObjectInfo>> &objects) {
        PointCloud::Ptr tmp(new PointCloud());
        //1.draw object on the image
        for(auto object:objects)
        {
            auto contour = object->mask_contours;
            auto label_color = GetLabelColor(object->class_id);
            cv::drawContours(color,contour,-1,label_color,-1);
        }
        //2.generate cloud points
        for (int m = 0; m < depth.rows; m += 3) {
            for (int n = 0; n < depth.cols; n += 3) {
                float d = depth.ptr<float>(m)[n];
                if (d < 0.01 || d > 10)
                    continue;
                PointT p;
                p.z = d;
                p.x = (n - kf->cx) * p.z / kf->fx;
                p.y = (m - kf->cy) * p.z / kf->fy;
                p.b = color.ptr<uchar>(m)[n * 3];
                p.g = color.ptr<uchar>(m)[n * 3 + 1];
                p.r = color.ptr<uchar>(m)[n * 3 + 2];
                tmp->points.push_back(p);
            }
        }

    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat(kf->GetPose());
    PointCloud::Ptr cloud(new PointCloud);
    pcl::transformPointCloud(*tmp, *cloud, T.inverse().matrix());
    cloud->is_dense = false;

    cout << "generate point cloud for kf " << kf->mnId << ", size=" << cloud->points.size() << endl;
    return cloud;
}


    void PointCloudMapping::viewer() {
        pcl::visualization::CloudViewer viewer("viewer");
        while (1) {
            {
                unique_lock<mutex> lck_shutdown(shutDownMutex);
                if (shutDownFlag) {
                    break;
                }
            }
            {
                unique_lock<mutex> lck_keyframeUpdated(keyFrameUpdateMutex);
                keyFrameUpdated.wait(lck_keyframeUpdated);
            }

            // keyframe is updated
            size_t N = 0;
            {
                unique_lock<mutex> lock(keyframeMutex);
                N = keyframes.size();
            }

            for (size_t i = lastKeyframeSize; i < N; i++) {
                if (enable_object == 1)
                {
                    PointCloud::Ptr p = generatePointCloud(keyframes[i], colorImgs[i], depthImgs[i], object_list[i]);
                *globalMap += *p;
                } else {
                PointCloud::Ptr p = generatePointCloud(keyframes[i], colorImgs[i], depthImgs[i]);
                *globalMap += *p;
                }
            }
            PointCloud::Ptr tmp(new PointCloud());
            voxel.setInputCloud(globalMap);
            voxel.filter(*tmp);
            globalMap->swap(*tmp);
            viewer.showCloud(globalMap);
            cout << "show global map, size=" << globalMap->points.size() << endl;
            lastKeyframeSize = N;
        }
    }
}

