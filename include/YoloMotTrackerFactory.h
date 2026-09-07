#ifndef ORB_SLAM2_YOLOMOTTRACKERFACTORY_H
#define ORB_SLAM2_YOLOMOTTRACKERFACTORY_H

#include <algorithm>
#include <cctype>
#include <memory>
#include <string>

#ifdef YOLOS_HAS_MOTCPP
#include <motcpp/tracker.hpp>
#include <motcpp/trackers/boosttrack.hpp>
#include <motcpp/trackers/bytetrack.hpp>
#include <motcpp/trackers/hybridsort.hpp>
#include <motcpp/trackers/ocsort.hpp>
#include <motcpp/trackers/oracletrack.hpp>
#include <motcpp/trackers/sort.hpp>
#include <motcpp/trackers/ucmc.hpp>
#endif

namespace ORB_SLAM2
{

inline std::string NormalizeYoloMOTTrackerType(std::string trackerType)
{
    std::transform(trackerType.begin(), trackerType.end(), trackerType.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    if(trackerType != "sort" &&
       trackerType != "bytetrack" &&
       trackerType != "ocsort" &&
       trackerType != "hybridsort" &&
       trackerType != "oracletrack" &&
       trackerType != "boosttrack" &&
       trackerType != "ucmc")
        trackerType = "sort";

    return trackerType;
}

#ifdef YOLOS_HAS_MOTCPP
inline std::unique_ptr<motcpp::BaseTracker> CreateYoloMOTTracker(std::string& trackerType)
{
    trackerType = NormalizeYoloMOTTrackerType(trackerType);

    if(trackerType == "bytetrack")
    {
        return std::unique_ptr<motcpp::BaseTracker>(new motcpp::trackers::ByteTrack(
            0.25f, 30, 50, 3, 0.3f, false, 80, "iou", false,
            0.1f, 0.45f, 0.8f, 25, 30));
    }

    if(trackerType == "ocsort")
    {
        return std::unique_ptr<motcpp::BaseTracker>(new motcpp::trackers::OCSort(
            0.25f, 30, 50, 3, 0.3f, false, 80, "iou", false,
            0.1f, 3, 0.2f, false, 0.01f, 0.0001f));
    }

    if(trackerType == "hybridsort")
    {
        return std::unique_ptr<motcpp::BaseTracker>(new motcpp::trackers::HybridSort(
            "", false, true,
            0.25f, 30, 50, 3, 0.3f, false, 80, "iou", false,
            0.1f, 3, 0.05f, false, true, 30, 0.9f, false, 0.5f,
            0.0f, 0.0f, true, false, 1.0f, 0.7f,
            false, 0.0f, false, 0.4f, 0.4f, "ecc", false));
    }

    if(trackerType == "oracletrack")
    {
        return std::unique_ptr<motcpp::BaseTracker>(new motcpp::trackers::OracleTrack(
            0.25f, 30, 3, 9.21f, 4.0f));
    }

    if(trackerType == "boosttrack")
    {
        return std::unique_ptr<motcpp::BaseTracker>(new motcpp::trackers::BoostTrackTracker(
            "", false, false,
            0.25f, 30, 50, 3, 0.3f, false, 80, "iou", false,
            true, 10, 1.6f, "ecc",
            0.5f, 0.25f, 0.25f,
            true, true, 0.65f,
            false, false, false, false, false));
    }

    if(trackerType == "ucmc")
    {
        return std::unique_ptr<motcpp::BaseTracker>(new motcpp::trackers::UCMCTrack(
            0.25f, 30, 50, 3, 0.3f, false, 80, "iou", false,
            100.0, 100.0, 5.0, 5.0, 10.0, 1.0 / 30.0, 0.5f));
    }

    trackerType = "sort";
    return std::unique_ptr<motcpp::BaseTracker>(new motcpp::trackers::Sort(
        0.25f, 30, 50, 3, 0.3f, false, 80, "iou", false));
}
#endif

} // namespace ORB_SLAM2

#endif // ORB_SLAM2_YOLOMOTTRACKERFACTORY_H
