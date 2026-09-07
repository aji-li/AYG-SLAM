#ifndef ORB_SLAM2_YOLODETECTOR_H
#define ORB_SLAM2_YOLODETECTOR_H

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>

namespace ORB_SLAM2
{

struct YoloDetection
{
    cv::Rect box;
    float confidence = 0.0f;
    int classId = -1;
    std::string className;
    cv::Mat mask;
    int trackId = -1;
    int trackDetIndex = -1;

    bool HasMask() const { return !mask.empty(); }
    bool HasTrack() const { return trackId >= 0; }
};

struct YoloRuntimeTiming
{
    double inferenceMs = -1.0;
    double trackerMs = -1.0;
    double postprocessMs = -1.0;
    double totalMs = -1.0;
};

class YoloDetector
{
public:
    YoloDetector(const std::string& modelPath,
                 const std::string& labelsPath,
                 bool useGPU,
                 bool useMOT = false,
                 const std::string& motTrackerType = "sort");
    ~YoloDetector();

    YoloDetector(const YoloDetector&) = delete;
    YoloDetector& operator=(const YoloDetector&) = delete;

    bool IsValid() const;
    const std::string& GetErrorMessage() const;

    bool Warmup(const cv::Size& imageSize,
                int iterations = 3,
                float confThreshold = 0.25f,
                float iouThreshold = 0.45f) const;

    const std::string& GetTrackerType() const;
    YoloRuntimeTiming GetLastTiming() const;

    std::vector<YoloDetection> Detect(const cv::Mat& image,
                                      float confThreshold = 0.25f,
                                      float iouThreshold = 0.45f) const;

private:
    class Impl;
    std::unique_ptr<Impl> mpImpl;
};

} // namespace ORB_SLAM2

#endif // ORB_SLAM2_YOLODETECTOR_H
