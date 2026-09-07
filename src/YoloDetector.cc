#include "YoloDetector.h"
#include "YoloMotTrackerFactory.h"

#include "yolos/tasks/detection.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <string>
#include <utility>

#ifdef YOLOS_HAS_MOTCPP
#include <motcpp/tracker.hpp>
#endif

namespace ORB_SLAM2
{

namespace
{
#ifdef YOLOS_HAS_MOTCPP
float ComputeBoxIoU(const cv::Rect& box, const Eigen::Vector4f& trackBox)
{
    const float x1 = std::max(static_cast<float>(box.x), trackBox.x());
    const float y1 = std::max(static_cast<float>(box.y), trackBox.y());
    const float x2 = std::min(static_cast<float>(box.x + box.width), trackBox.z());
    const float y2 = std::min(static_cast<float>(box.y + box.height), trackBox.w());
    const float interWidth = std::max(0.0f, x2 - x1);
    const float interHeight = std::max(0.0f, y2 - y1);
    const float interArea = interWidth * interHeight;
    const float unionArea =
        static_cast<float>(box.area()) +
        std::max(0.0f, (trackBox.z() - trackBox.x()) * (trackBox.w() - trackBox.y())) -
        interArea;

    if(unionArea <= 0.0f)
        return 0.0f;

    return interArea / unionArea;
}

void AssignTracks(std::vector<yolos::det::Detection>& results,
                  motcpp::BaseTracker& tracker,
                  const cv::Mat& image)
{
    if(results.empty() || image.empty())
        return;

    std::vector<int> detToResult;
    detToResult.reserve(results.size());
    for(size_t i = 0; i < results.size(); ++i)
    {
        const auto& box = results[i].box;
        if(box.width <= 0 || box.height <= 0 || results[i].conf <= 0.0f)
            continue;
        detToResult.push_back(static_cast<int>(i));
    }

    if(detToResult.empty())
        return;

    Eigen::MatrixXf dets(static_cast<int>(detToResult.size()), 6);
    for(size_t row = 0; row < detToResult.size(); ++row)
    {
        const auto& det = results[detToResult[row]];
        dets(static_cast<int>(row), 0) = static_cast<float>(det.box.x);
        dets(static_cast<int>(row), 1) = static_cast<float>(det.box.y);
        dets(static_cast<int>(row), 2) = static_cast<float>(det.box.x + det.box.width);
        dets(static_cast<int>(row), 3) = static_cast<float>(det.box.y + det.box.height);
        dets(static_cast<int>(row), 4) = det.conf;
        dets(static_cast<int>(row), 5) = static_cast<float>(det.classId);
    }

    const Eigen::MatrixXf tracks = tracker.update(dets, image);
    if(tracks.rows() == 0 || tracks.cols() < 5)
        return;

    if(tracks.cols() >= 8)
    {
        for(int row = 0; row < tracks.rows(); ++row)
        {
            const int detIndex = static_cast<int>(std::lround(tracks(row, 7)));
            if(detIndex < 0 || detIndex >= static_cast<int>(detToResult.size()))
                continue;

            auto& det = results[detToResult[static_cast<size_t>(detIndex)]];
            det.trackId = static_cast<int>(std::lround(tracks(row, 4)));
            det.trackDetIndex = detIndex;
        }
        return;
    }

    struct CandidateMatch
    {
        float iou;
        int trackRow;
        int detIndex;
    };

    std::vector<CandidateMatch> candidates;
    candidates.reserve(static_cast<size_t>(tracks.rows()) * detToResult.size());
    for(int row = 0; row < tracks.rows(); ++row)
    {
        const Eigen::Vector4f trackBox = tracks.row(row).head<4>();
        const int trackClass = tracks.cols() > 6 ? static_cast<int>(std::lround(tracks(row, 6))) : -1;
        for(size_t detIndex = 0; detIndex < detToResult.size(); ++detIndex)
        {
            const auto& det = results[detToResult[detIndex]];
            if(trackClass >= 0 && det.classId != trackClass)
                continue;

            const float iou = ComputeBoxIoU(
                cv::Rect(det.box.x, det.box.y, det.box.width, det.box.height),
                trackBox);
            if(iou <= 0.0f)
                continue;
            candidates.push_back({iou, row, static_cast<int>(detIndex)});
        }
    }

    std::sort(candidates.begin(), candidates.end(),
              [](const CandidateMatch& a, const CandidateMatch& b) {
                  return a.iou > b.iou;
              });

    std::vector<bool> usedTrack(static_cast<size_t>(tracks.rows()), false);
    std::vector<bool> usedDet(detToResult.size(), false);
    for(const CandidateMatch& candidate : candidates)
    {
        if(candidate.iou < 0.1f)
            break;
        if(usedTrack[static_cast<size_t>(candidate.trackRow)] ||
           usedDet[static_cast<size_t>(candidate.detIndex)])
            continue;

        auto& det = results[detToResult[static_cast<size_t>(candidate.detIndex)]];
        det.trackId = static_cast<int>(std::lround(tracks(candidate.trackRow, 4)));
        det.trackDetIndex = candidate.detIndex;
        usedTrack[static_cast<size_t>(candidate.trackRow)] = true;
        usedDet[static_cast<size_t>(candidate.detIndex)] = true;
    }
}
#endif
}

class YoloDetector::Impl
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
            detector.reset(new yolos::det::YOLODetector(modelPath, labelsPath, useGPU));
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

    std::unique_ptr<yolos::det::YOLODetector> detector;
#ifdef YOLOS_HAS_MOTCPP
    std::unique_ptr<motcpp::BaseTracker> tracker;
#endif
    bool valid = false;
    bool motEnabled = false;
    std::string errorMessage;
    std::string trackerType = "sort";
    YoloRuntimeTiming lastTiming;
};

YoloDetector::YoloDetector(const std::string& modelPath,
                           const std::string& labelsPath,
                           const bool useGPU,
                           const bool useMOT,
                           const std::string& motTrackerType)
    : mpImpl(new Impl(modelPath, labelsPath, useGPU, useMOT, motTrackerType))
{
}

YoloDetector::~YoloDetector() = default;

bool YoloDetector::IsValid() const
{
    return mpImpl && mpImpl->valid && mpImpl->detector;
}

const std::string& YoloDetector::GetErrorMessage() const
{
    static const std::string kEmpty;
    if(!mpImpl)
        return kEmpty;
    return mpImpl->errorMessage;
}

bool YoloDetector::Warmup(const cv::Size& imageSize,
                         const int iterations,
                         const float confThreshold,
                         const float iouThreshold) const
{
    if(!IsValid() || imageSize.width <= 0 || imageSize.height <= 0)
        return false;

    const int warmupIters = std::max(1, iterations);
    cv::Mat warmupImage(imageSize, CV_8UC3, cv::Scalar::all(0));
    for(int i = 0; i < warmupIters; ++i)
        (void)mpImpl->detector->detect(warmupImage, confThreshold, iouThreshold);

    return true;
}

const std::string& YoloDetector::GetTrackerType() const
{
    static const std::string kEmpty;
    if(!mpImpl)
        return kEmpty;
    return mpImpl->trackerType;
}

YoloRuntimeTiming YoloDetector::GetLastTiming() const
{
    if(!mpImpl)
        return YoloRuntimeTiming();
    return mpImpl->lastTiming;
}

std::vector<YoloDetection> YoloDetector::Detect(const cv::Mat& image,
                                                const float confThreshold,
                                                const float iouThreshold) const
{
    std::vector<YoloDetection> detections;
    if(!IsValid() || image.empty())
        return detections;

    const auto t0 = std::chrono::steady_clock::now();
    auto rawDetections = mpImpl->detector->detect(image, confThreshold, iouThreshold);
    const auto t1 = std::chrono::steady_clock::now();
    double trackerMs = 0.0;
#ifdef YOLOS_HAS_MOTCPP
    if(mpImpl->motEnabled && mpImpl->tracker)
    {
        const auto tTrack0 = std::chrono::steady_clock::now();
        AssignTracks(rawDetections, *mpImpl->tracker, image);
        const auto tTrack1 = std::chrono::steady_clock::now();
        trackerMs =
            std::chrono::duration<double, std::milli>(tTrack1 - tTrack0).count();
    }
#endif
    const std::vector<std::string>& classNames = mpImpl->detector->getClassNames();

    const auto tPost0 = std::chrono::steady_clock::now();
    detections.reserve(rawDetections.size());
    for(const auto& det : rawDetections)
    {
        YoloDetection out;
        out.box = cv::Rect(det.box.x, det.box.y, det.box.width, det.box.height);
        out.confidence = det.conf;
        out.classId = det.classId;
        if(det.classId >= 0 && static_cast<size_t>(det.classId) < classNames.size())
            out.className = classNames[det.classId];
        else
            out.className = "unknown";
        out.trackId = det.trackId;
        out.trackDetIndex = det.trackDetIndex;
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
