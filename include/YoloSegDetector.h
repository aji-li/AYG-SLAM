#ifndef ORB_SLAM2_YOLOSEGDETECTOR_H
#define ORB_SLAM2_YOLOSEGDETECTOR_H

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

#include "YoloDetector.h"

namespace ORB_SLAM2
{

class YoloSegDetector
{
public:
    YoloSegDetector(const std::string& modelPath,
                    const std::string& labelsPath,
                    bool useGPU,
                    bool useMOT = false,
                    const std::string& motTrackerType = "sort");
    ~YoloSegDetector();

    YoloSegDetector(const YoloSegDetector&) = delete;
    YoloSegDetector& operator=(const YoloSegDetector&) = delete;

    bool IsValid() const;
    const std::string& GetErrorMessage() const;

    bool Warmup(const cv::Size& imageSize,
                int iterations = 3,
                float confThreshold = 0.25f,
                float iouThreshold = 0.45f) const;

    const std::string& GetTrackerType() const;
    YoloRuntimeTiming GetLastTiming() const;

    std::vector<YoloDetection> Segment(const cv::Mat& image,
                                       float confThreshold = 0.25f,
                                       float iouThreshold = 0.45f);

private:
    class Impl;
    std::unique_ptr<Impl> mpImpl;
};

} // namespace ORB_SLAM2

#endif // ORB_SLAM2_YOLOSEGDETECTOR_H
