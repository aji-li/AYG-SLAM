#include "YoloSegDetector.h"
#include "YoloMotTrackerFactory.h"

#include "yolos/tasks/segmentation.hpp"

#include <chrono>
#include <exception>
#include <utility>

namespace ORB_SLAM2
{

class YoloSegDetector::Impl
{
public:
    Impl(const std::string& modelPath,
         const std::string& labelsPath,
         const bool useGPU,
         const bool useMOT,
         const std::string& motTrackerType)
    {
        try
        {
            detector.reset(new yolos::seg::YOLOSegDetector(modelPath, labelsPath, useGPU));
#ifdef YOLOS_HAS_MOTCPP
            motEnabled = useMOT;
            trackerType = NormalizeYoloMOTTrackerType(motTrackerType);
            if(motEnabled)
                tracker = CreateYoloMOTTracker(trackerType);
#else
            (void)useMOT;
            (void)motTrackerType;
#endif
            valid = true;
        }
        catch(const std::exception& e)
        {
            errorMessage = e.what();
            valid = false;
        }
    }

    std::unique_ptr<yolos::seg::YOLOSegDetector> detector;
#ifdef YOLOS_HAS_MOTCPP
    std::unique_ptr<motcpp::BaseTracker> tracker;
#endif
    bool valid = false;
    bool motEnabled = false;
    std::string errorMessage;
    std::string trackerType = "sort";
    YoloRuntimeTiming lastTiming;
};

YoloSegDetector::YoloSegDetector(const std::string& modelPath,
                                 const std::string& labelsPath,
                                 const bool useGPU,
                                 const bool useMOT,
                                 const std::string& motTrackerType)
    : mpImpl(new Impl(modelPath, labelsPath, useGPU, useMOT, motTrackerType))
{
}

YoloSegDetector::~YoloSegDetector() = default;

bool YoloSegDetector::IsValid() const
{
    return mpImpl && mpImpl->valid && mpImpl->detector;
}

const std::string& YoloSegDetector::GetErrorMessage() const
{
    static const std::string kEmpty;
    if(!mpImpl)
        return kEmpty;
    return mpImpl->errorMessage;
}

bool YoloSegDetector::Warmup(const cv::Size& imageSize,
                             const int iterations,
                             const float confThreshold,
                             const float iouThreshold) const
{
    if(!IsValid() || imageSize.width <= 0 || imageSize.height <= 0)
        return false;

    const int warmupIters = std::max(1, iterations);
    cv::Mat warmupImage(imageSize, CV_8UC3, cv::Scalar::all(0));
    for(int i = 0; i < warmupIters; ++i)
        (void)mpImpl->detector->segment(warmupImage, confThreshold, iouThreshold);

    return true;
}

const std::string& YoloSegDetector::GetTrackerType() const
{
    static const std::string kEmpty;
    if(!mpImpl)
        return kEmpty;
    return mpImpl->trackerType;
}

YoloRuntimeTiming YoloSegDetector::GetLastTiming() const
{
    if(!mpImpl)
        return YoloRuntimeTiming();
    return mpImpl->lastTiming;
}

std::vector<YoloDetection> YoloSegDetector::Segment(const cv::Mat& image,
                                                    const float confThreshold,
                                                    const float iouThreshold)
{
    std::vector<YoloDetection> detections;
    if(!IsValid() || image.empty())
        return detections;

    const auto t0 = std::chrono::steady_clock::now();
    auto rawSegments = mpImpl->detector->segment(image, confThreshold, iouThreshold);
    const auto t1 = std::chrono::steady_clock::now();
    double trackerMs = 0.0;
#ifdef YOLOS_HAS_MOTCPP
    if(mpImpl->motEnabled && mpImpl->tracker)
    {
        const auto tTrack0 = std::chrono::steady_clock::now();
        yolos::seg::assignTracks(rawSegments, *mpImpl->tracker, image);
        const auto tTrack1 = std::chrono::steady_clock::now();
        trackerMs =
            std::chrono::duration<double, std::milli>(tTrack1 - tTrack0).count();
    }
#endif
    const std::vector<std::string>& classNames = mpImpl->detector->getClassNames();

    const auto tPost0 = std::chrono::steady_clock::now();
    detections.reserve(rawSegments.size());
    for(const auto& seg : rawSegments)
    {
        YoloDetection out;
        out.box = cv::Rect(seg.box.x, seg.box.y, seg.box.width, seg.box.height);
        out.confidence = seg.conf;
        out.classId = seg.classId;
        if(seg.classId >= 0 && static_cast<size_t>(seg.classId) < classNames.size())
            out.className = classNames[seg.classId];
        else
            out.className = "unknown";
        out.mask = seg.mask.clone();
        out.trackId = seg.trackId;
        out.trackDetIndex = seg.trackDetIndex;
        detections.push_back(std::move(out));
    }
    const auto tPost1 = std::chrono::steady_clock::now();

    mpImpl->lastTiming.inferenceMs =
        std::chrono::duration<double, std::milli>(t1 - t0).count();
    mpImpl->lastTiming.trackerMs = trackerMs;
    mpImpl->lastTiming.postprocessMs =
        std::chrono::duration<double, std::milli>(tPost1 - tPost0).count();
    mpImpl->lastTiming.totalMs =
        std::chrono::duration<double, std::milli>(tPost1 - t0).count();

    return detections;
}

} // namespace ORB_SLAM2
