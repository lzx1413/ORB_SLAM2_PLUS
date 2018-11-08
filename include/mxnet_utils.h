//
// Created by zuoxin on 18-10-22.
//

#ifndef ORB_SLAM2_MXNET_UTILS_H
#define ORB_SLAM2_MXNET_UTILS_H
#include "mxnet-cpp/MxNetCpp.h"
#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <map>
#include <cmath>
#include <random>
#include <iomanip>

using namespace mxnet::cpp;

// resize short within
inline cv::Mat ResizeShortWithin(cv::Mat src, int short_size, int max_size, int mult_base) {
    double h = src.rows;
    double w = src.cols;
    double im_size_min = h;
    double im_size_max = w;
    if (w < h) {
        im_size_min = w;
        im_size_max = h;
    }
    double mb = mult_base;  // this is the factor of the output shapes
    double scale = static_cast<double>(short_size) / static_cast<double>(im_size_min);
    if ((std::round(scale * im_size_max / mb) * mb) > max_size) {
        // fit in max_size
        scale = std::floor(static_cast<double>(max_size) / mb) * mb / im_size_max;
    }
    int new_w = static_cast<int>(std::round(w * scale / mb) * mb);
    int new_h = static_cast<int>(std::round(h * scale / mb) * mb);
    cv::Mat dst;
    cv::resize(src, dst, cv::Size(new_w, new_h));
    return dst;
}
//clip the box
/*
inline cv::Rect clip_bbox(cv::Rect box, int img_w ,int img_h)
{
    int x,y,width, height;
    x = box.x<0?0:box.x;
    y = box.y<0?0:box.y;
    width = box.width>img_w-box.x?img_w-box.x:box.width;
    height = box.height>img_h-box.y?img_h-box.y:box.height;
    return cv::Rect(x,y,width,height);
}
 */

// Load data from CV BGR image
inline NDArray AsData(cv::Mat bgr_image, Context ctx = Context::cpu()) {
    // convert BGR image from OpenCV to RGB in MXNet.
    cv::Mat rgb_image;
    cv::cvtColor(bgr_image, rgb_image, cv::COLOR_BGR2RGB);
    // convert to float32 from uint8
    rgb_image.convertTo(rgb_image, CV_32FC3);
    // flatten to single channel, and single row.
    cv::Mat flat_image = rgb_image.reshape(1, 1);
    // a vector of raw pixel values, no copy
    std::vector<float> data_buffer;
    data_buffer.insert(
            data_buffer.end(),
            flat_image.ptr<float>(0),
            flat_image.ptr<float>(0) + flat_image.cols);
    // construct NDArray from data buffer
    return NDArray(data_buffer, Shape(1, rgb_image.rows, rgb_image.cols, 3), ctx);
}

// Load data from filename
inline NDArray AsData(std::string filename, Context ctx = Context::cpu()) {
    cv::Mat bgr_image = cv::imread(filename, 1);
    return AsData(bgr_image, ctx);
}

inline void LoadCheckpoint(const std::string config_path, const std::string weight_path,
                           Symbol* symbol, std::map<std::string, NDArray>* arg_params,
                           std::map<std::string, NDArray>* aux_params,
                           Context ctx = Context::cpu()) {
    // load symbol from JSON
    Symbol new_symbol = Symbol::Load(config_path);
    // load parameters
    std::map<std::string, NDArray> params = NDArray::LoadToMap(weight_path);
    std::map<std::string, NDArray> args;
    std::map<std::string, NDArray> auxs;
    for (auto iter : params) {
        std::string type = iter.first.substr(0, 4);
        std::string name = iter.first.substr(4);
        if (type == "arg:")
            args[name] = iter.second.Copy(ctx);
        else if (type == "aux:")
            auxs[name] = iter.second.Copy(ctx);
        else
            continue;
    }
    NDArray::WaitAll();

    *symbol = new_symbol;
    *arg_params = args;
    *aux_params = auxs;
}

inline bool EndsWith(std::string const & value, std::string const & ending)
{
    if (ending.size() > value.size()) return false;
    return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

inline std::vector<std::string> LoadClassNames(std::string filename) {
    std::vector<std::string> classes;
    std::string line;
    std::ifstream infile(filename);
    while(infile >> line) {
        classes.emplace_back(line);
    }
    return classes;
}

// convert color from hsv to bgr for plotting
inline cv::Scalar HSV2BGR(cv::Scalar hsv) {
    cv::Mat from(1, 1, CV_32FC3, hsv);
    cv::Mat to;
    cv::cvtColor(from, to, cv::COLOR_HSV2BGR);
    auto pixel = to.at<cv::Vec3f>(0, 0);
    unsigned char b = static_cast<unsigned char>(pixel[0] * 255);
    unsigned char g = static_cast<unsigned char>(pixel[1] * 255);
    unsigned char r = static_cast<unsigned char>(pixel[2] * 255);
    return cv::Scalar(b, g, r);
}

inline void PutLabel(cv::Mat &im, const std::string label, const cv::Point & orig, cv::Scalar color) {
    int fontface = cv::FONT_HERSHEY_DUPLEX;
    double scale = 0.5;
    int thickness = 1;
    int baseline = 0;
    double alpha = 0.6;

    cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
    // make sure roi inside image region
    cv::Rect blend_rect = cv::Rect(orig + cv::Point(0, baseline),
                                   orig + cv::Point(text.width, -text.height)) & cv::Rect(0, 0, im.cols, im.rows);
    cv::Mat roi = im(blend_rect);
    cv::Mat blend(roi.size(), CV_8UC3, color);
    // cv::rectangle(im, orig + cv::Point(0, baseline), orig + cv::Point(text.width, -text.height), CV_RGB(0, 0, 0), CV_FILLED);
    cv::addWeighted(blend, alpha, roi, 1.0 - alpha, 0.0, roi);
    cv::putText(im, label, orig, fontface, scale, cv::Scalar(255, 255, 255), thickness, 8);
}

inline cv::Rect clip_bbox(cv::Rect box, int img_w ,int img_h)
{
    int x,y,width, height;
    if(box.x < 0)
        x = 0;
    else
        x = box.x;
    if(box.y < 0)
        y = 0;
    else
        y = box.y;

    if(box.x+box.width>img_w)
        width = img_w - x;
    else
        width = box.width - (x-box.x);
    if(box.y+box.height>img_h)
        height = img_h - y;
    else
        height = box.height - (y-box.y);
    return cv::Rect(x,y,width,height);
}
#endif //ORB_SLAM2_MXNET_UTILS_H
