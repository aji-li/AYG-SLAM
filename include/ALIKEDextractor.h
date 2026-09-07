#pragma once

#include <memory>
#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

namespace ORB_SLAM2
{

class ALIKEDextractor
{
public:
    ALIKEDextractor(const std::string& model_name = "aliked-n32",
                    const std::string& device = "cuda");
    ~ALIKEDextractor();

    ALIKEDextractor(const ALIKEDextractor&) = delete;
    ALIKEDextractor& operator=(const ALIKEDextractor&) = delete;

    bool Detect(const cv::Mat& image,
                std::vector<cv::KeyPoint>& keypoints,
                int max_num) const;

    bool DetectAndCompute(const cv::Mat& image,
                          std::vector<cv::KeyPoint>& keypoints,
                          cv::Mat& descriptors,
                          int max_num) const;

private:
    class Impl;
    std::unique_ptr<Impl> mpImpl;
};

} // namespace ORB_SLAM2
