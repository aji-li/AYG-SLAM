// Instantiate PCL templates with the same Eigen alignment as this translation
// unit. The system PCL binaries use baseline alignment, whereas -march=native
// can enable wider alignment; mixing their allocators can crash on cloud cleanup.
#define PCL_NO_PRECOMPILE
// Load system FLANN before OpenCV's FLANN headers define USE_UNORDERED_MAP.
#include <flann/flann.hpp>
#include "PointCloudMapping.h"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/header.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

namespace ORB_SLAM2
{

namespace
{
constexpr int kRuntimeLogAverageWindow = 30;

struct PointCloudMapTimingAccumulator
{
    int samples = 0;
    double sumTotalMs = 0.0;
    double sumQueueMs = 0.0;
    double sumDepthMs = 0.0;
    double sumDynamicBoxesMs = 0.0;
    double sumStaticCloudMs = 0.0;
    double sumVoxelMs = 0.0;
    double sumSorMs = 0.0;
    double sumTransformMs = 0.0;
    double sumRosMsgMs = 0.0;
    double sumTfMs = 0.0;
    double sumCloudPublishMs = 0.0;
    double sumMarkerPublishMs = 0.0;
    double sumRawPoints = 0.0;
    double sumVoxelPoints = 0.0;
    double sumFilteredPoints = 0.0;
    double sumLocalPoints = 0.0;
    double sumDynamicBoxes = 0.0;
    double sumDepthFromStereo = 0.0;
};

struct RosPublishTimingAccumulator
{
    int samples = 0;
    double sumTotalMs = 0.0;
    double sumEncodeMs = 0.0;
    double sumPublishMs = 0.0;
    double sumCols = 0.0;
    double sumRows = 0.0;
    double sumChannels = 0.0;
};

struct OctomapTimingAccumulator
{
    int samples = 0;
    double sumPublishToOctomapMs = 0.0;
    double sumStampDeltaMs = 0.0;
    int stampDeltaCount = 0;
};

struct PointCloudMapTimingSummary
{
    long unsigned int keyFrameId = std::numeric_limits<long unsigned int>::max();
    double totalMs = -1.0;
    double queueMs = -1.0;
    double depthMs = -1.0;
    double dynamicBoxesMs = -1.0;
    double staticCloudMs = -1.0;
    double voxelMs = -1.0;
    double sorMs = -1.0;
    double transformMs = -1.0;
    double rosMsgMs = -1.0;
    double tfMs = -1.0;
    double cloudPublishMs = -1.0;
    double markerPublishMs = -1.0;
    std::size_t rawPoints = 0;
    std::size_t voxelPoints = 0;
    std::size_t filteredPoints = 0;
    std::size_t localPoints = 0;
    std::size_t dynamicBoxes = 0;
    bool builtDepthFromStereo = false;
    std::string exitStage;
};

void PrintPointCloudMapTimingSummary(const PointCloudMappingConfig& config,
                                     const PointCloudMapTimingSummary& timing)
{
    if(!config.runtimeLogs)
        return;

    static PointCloudMapTimingAccumulator acc;
    ++acc.samples;
    acc.sumTotalMs += std::max(0.0, timing.totalMs);
    acc.sumQueueMs += std::max(0.0, timing.queueMs);
    if(timing.depthMs >= 0.0)
        acc.sumDepthMs += timing.depthMs;
    acc.sumDynamicBoxesMs += std::max(0.0, timing.dynamicBoxesMs);
    acc.sumStaticCloudMs += std::max(0.0, timing.staticCloudMs);
    acc.sumVoxelMs += std::max(0.0, timing.voxelMs);
    acc.sumSorMs += std::max(0.0, timing.sorMs);
    acc.sumTransformMs += std::max(0.0, timing.transformMs);
    acc.sumRosMsgMs += std::max(0.0, timing.rosMsgMs);
    acc.sumTfMs += std::max(0.0, timing.tfMs);
    acc.sumCloudPublishMs += std::max(0.0, timing.cloudPublishMs);
    acc.sumMarkerPublishMs += std::max(0.0, timing.markerPublishMs);
    acc.sumRawPoints += timing.rawPoints;
    acc.sumVoxelPoints += timing.voxelPoints;
    acc.sumFilteredPoints += timing.filteredPoints;
    acc.sumLocalPoints += timing.localPoints;
    acc.sumDynamicBoxes += timing.dynamicBoxes;
    acc.sumDepthFromStereo += timing.builtDepthFromStereo ? 1.0 : 0.0;

    if(acc.samples < kRuntimeLogAverageWindow)
        return;

    std::cout << "[PointCloudMappingTiming] window=" << acc.samples
              << " avg_total_ms=" << (acc.sumTotalMs / acc.samples)
              << " avg_queue_ms=" << (acc.sumQueueMs / acc.samples)
              << " avg_depth_ms=" << (acc.sumDepthMs / acc.samples)
              << " depth_from_stereo_rate=" << (acc.sumDepthFromStereo / acc.samples)
              << " avg_dynamic_boxes_ms=" << (acc.sumDynamicBoxesMs / acc.samples)
              << " avg_static_cloud_ms=" << (acc.sumStaticCloudMs / acc.samples)
              << " avg_voxel_ms=" << (acc.sumVoxelMs / acc.samples)
              << " avg_sor_ms=" << (acc.sumSorMs / acc.samples)
              << " avg_transform_ms=" << (acc.sumTransformMs / acc.samples)
              << " avg_rosmsg_ms=" << (acc.sumRosMsgMs / acc.samples)
              << " avg_tf_ms=" << (acc.sumTfMs / acc.samples)
              << " avg_cloud_pub_ms=" << (acc.sumCloudPublishMs / acc.samples)
              << " avg_marker_pub_ms=" << (acc.sumMarkerPublishMs / acc.samples)
              << " avg_raw_points=" << (acc.sumRawPoints / acc.samples)
              << " avg_voxel_points=" << (acc.sumVoxelPoints / acc.samples)
              << " avg_filtered_points=" << (acc.sumFilteredPoints / acc.samples)
              << " avg_local_points=" << (acc.sumLocalPoints / acc.samples)
              << " avg_dynamic_boxes=" << (acc.sumDynamicBoxes / acc.samples)
              << " last_kf=" << timing.keyFrameId
              << std::endl;

    acc = PointCloudMapTimingAccumulator();
}

bool IsFiniteDepth(const float d)
{
    return std::isfinite(d) && d > 0.0f;
}

std::string ToLower(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

float Clamp01(const float value)
{
    return std::max(0.0f, std::min(1.0f, value));
}

std_msgs::msg::ColorRGBA MakeTrackColor(const int seed, const float alpha)
{
    const int value = seed >= 0 ? seed : 0;
    const float r = 0.35f + 0.55f * static_cast<float>((value * 37) % 100) / 99.0f;
    const float g = 0.35f + 0.55f * static_cast<float>((value * 57 + 13) % 100) / 99.0f;
    const float b = 0.35f + 0.55f * static_cast<float>((value * 83 + 29) % 100) / 99.0f;

    std_msgs::msg::ColorRGBA color;
    color.r = Clamp01(r);
    color.g = Clamp01(g);
    color.b = Clamp01(b);
    color.a = Clamp01(alpha);
    return color;
}

geometry_msgs::msg::Point ToPointMsg(const Eigen::Vector3f& point)
{
    geometry_msgs::msg::Point out;
    out.x = static_cast<double>(point.x());
    out.y = static_cast<double>(point.y());
    out.z = static_cast<double>(point.z());
    return out;
}

cv::Mat NormalizeDetectionMask(const cv::Mat& mask, const cv::Size& targetSize)
{
    if(mask.empty() || targetSize.width <= 0 || targetSize.height <= 0)
        return cv::Mat();

    cv::Mat resizedMask;
    if(mask.size() != targetSize)
        cv::resize(mask, resizedMask, targetSize, 0, 0, cv::INTER_NEAREST);
    else
        resizedMask = mask;

    cv::Mat singleChannelMask;
    if(resizedMask.channels() == 1)
        singleChannelMask = resizedMask;
    else if(resizedMask.channels() == 3)
        cv::cvtColor(resizedMask, singleChannelMask, cv::COLOR_BGR2GRAY);
    else if(resizedMask.channels() == 4)
        cv::cvtColor(resizedMask, singleChannelMask, cv::COLOR_BGRA2GRAY);
    else
        return cv::Mat();

    cv::Mat maskBinary;
    if(singleChannelMask.type() != CV_8U)
        singleChannelMask.convertTo(maskBinary, CV_8U);
    else
        maskBinary = singleChannelMask.clone();

    cv::threshold(maskBinary, maskBinary, 0, 255, cv::THRESH_BINARY);
    return maskBinary;
}

cv::Mat NormalizeColorImage8U(const cv::Mat& color)
{
    if(color.empty())
        return cv::Mat();

    const int channels = color.channels();
    if(channels != 1 && channels != 3 && channels != 4)
        return cv::Mat();

    if(color.depth() == CV_8U)
        return color.clone();

    cv::Mat normalized;
    double minValue = 0.0;
    double maxValue = 0.0;
    cv::minMaxLoc(color.reshape(1), &minValue, &maxValue);

    if(std::fabs(maxValue - minValue) < 1e-12)
    {
        color.convertTo(normalized, CV_MAKETYPE(CV_8U, channels));
        return normalized;
    }

    const double scale = 255.0 / (maxValue - minValue);
    const double shift = -minValue * scale;
    color.convertTo(normalized, CV_MAKETYPE(CV_8U, channels), scale, shift);
    return normalized;
}

cv::Mat ToStereoGray(const cv::Mat& image)
{
    if(image.empty())
        return cv::Mat();

    cv::Mat normalized = NormalizeColorImage8U(image);
    if(normalized.empty())
        return cv::Mat();

    if(normalized.channels() == 1)
        return normalized;

    cv::Mat gray;
    if(normalized.channels() == 3)
        cv::cvtColor(normalized, gray, cv::COLOR_BGR2GRAY);
    else if(normalized.channels() == 4)
        cv::cvtColor(normalized, gray, cv::COLOR_BGRA2GRAY);
    return gray;
}

cv::Rect ShrinkRect(const cv::Rect& rect, const float insetRatio, const cv::Rect& imageRect)
{
    if(rect.width <= 2 || rect.height <= 2 || insetRatio <= 0.0f)
        return rect & imageRect;

    const int insetX = static_cast<int>(std::round(rect.width * insetRatio));
    const int insetY = static_cast<int>(std::round(rect.height * insetRatio));

    cv::Rect inner(rect.x + insetX,
                   rect.y + insetY,
                   rect.width - 2 * insetX,
                   rect.height - 2 * insetY);
    if(inner.width < 2 || inner.height < 2)
        return rect & imageRect;
    return inner & imageRect;
}

bool ShouldUseMeshMarker(const PointCloudMappingConfig& config,
                         const std::string& className)
{
    return config.dynamicMeshEnable &&
           !config.dynamicMeshResource.empty() &&
           ToLower(className) == ToLower(config.dynamicMeshClassName);
}
}

PointCloudMapping::PointCloudMapping(const PointCloudMappingConfig& config)
    : mConfig(config)
{
    mNode = std::make_shared<rclcpp::Node>("ayg_slam_mapping");
    const auto latchedQos = rclcpp::QoS(1).transient_local();
    mLocalCloudPublisher =
        mNode->create_publisher<sensor_msgs::msg::PointCloud2>(mConfig.localCloudTopic, latchedQos);
    mRgbImagePublisher =
        mNode->create_publisher<sensor_msgs::msg::Image>(mConfig.rgbImageTopic, rclcpp::QoS(2));
    mDynamicBoxPublisher =
        mNode->create_publisher<visualization_msgs::msg::MarkerArray>(mConfig.dynamicBoxTopic, latchedQos);
    mOctomapBinarySubscriber =
        mNode->create_subscription<octomap_msgs::msg::Octomap>(
            mConfig.octomapBinaryTopic, rclcpp::QoS(1).transient_local(),
            std::bind(&PointCloudMapping::OnOctomapBinary, this, std::placeholders::_1));
    mTfBroadcaster.reset(new tf2_ros::TransformBroadcaster(mNode));
    mExecutor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    mExecutor->add_node(mNode);
    mRosSpinThread = std::thread([this] { mExecutor->spin(); });

    mVoxelLocal.setLeafSize(mConfig.voxelLeafSize,
                            mConfig.voxelLeafSize,
                            mConfig.voxelLeafSize);
    mSorLocal.setMeanK(std::max(1, mConfig.sorMeanK));
    mSorLocal.setStddevMulThresh(mConfig.sorStddevMulThresh);

    mpLocalCameraCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    mpVoxelCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    mpFilteredCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    mpRosAlignedCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

    mViewerThread = std::thread(&PointCloudMapping::MapViewer, this);
}

PointCloudMapping::~PointCloudMapping()
{
    shutdown();
}

void PointCloudMapping::SpinSome()
{
    rclcpp::spin_some(mNode);
}

void PointCloudMapping::insertKeyFrame(KeyFrame* pKF,
                                       const cv::Mat& color,
                                       const cv::Mat& depth,
                                       const std::vector<YoloDetection>& detections)
{
    if(!pKF || color.empty() || depth.empty())
        return;

    CloudKeyFrameInput input;
    input.keyFrameId = pKF->mnId;
    input.keyFramePose = pKF->GetPose().clone();
    input.fx = pKF->fx;
    input.fy = pKF->fy;
    input.cx = pKF->cx;
    input.cy = pKF->cy;
    input.bf = pKF->mbf;
    input.color = NormalizeColorImage8U(color);
    input.depth = depth.clone();
    input.stamp = rclcpp::Time(
        static_cast<int64_t>(pKF->mTimeStamp * 1e9), RCL_ROS_TIME);
    input.enqueuedAt = std::chrono::steady_clock::now();

    if(input.color.empty())
        return;

    input.detections.reserve(detections.size());
    for(const YoloDetection& detection : detections)
    {
        YoloDetection normalizedDetection = detection;
        if(normalizedDetection.HasMask())
            normalizedDetection.mask = NormalizeDetectionMask(normalizedDetection.mask, color.size());
        input.detections.push_back(std::move(normalizedDetection));
    }

    {
        std::lock_guard<std::mutex> lock(mMutexKeyFrameQueue);
        mvPendingInputs.push_back(std::move(input));
    }
    mCondNewKeyFrame.notify_one();
}

void PointCloudMapping::insertStereoKeyFrame(KeyFrame* pKF,
                                             const cv::Mat& leftColor,
                                             const cv::Mat& rightColor,
                                             const float bf,
                                             const std::vector<YoloDetection>& detections)
{
    if(!pKF || leftColor.empty() || rightColor.empty())
        return;

    CloudKeyFrameInput input;
    input.keyFrameId = pKF->mnId;
    input.keyFramePose = pKF->GetPose().clone();
    input.fx = pKF->fx;
    input.fy = pKF->fy;
    input.cx = pKF->cx;
    input.cy = pKF->cy;
    input.bf = bf;
    input.color = NormalizeColorImage8U(leftColor);
    input.rightColor = NormalizeColorImage8U(rightColor);
    input.stamp = rclcpp::Time(
        static_cast<int64_t>(pKF->mTimeStamp * 1e9), RCL_ROS_TIME);
    input.enqueuedAt = std::chrono::steady_clock::now();

    if(input.color.empty() || input.rightColor.empty())
        return;

    input.detections.reserve(detections.size());
    for(const YoloDetection& detection : detections)
    {
        YoloDetection normalizedDetection = detection;
        if(normalizedDetection.HasMask())
            normalizedDetection.mask = NormalizeDetectionMask(normalizedDetection.mask, leftColor.size());
        input.detections.push_back(std::move(normalizedDetection));
    }

    {
        std::lock_guard<std::mutex> lock(mMutexKeyFrameQueue);
        mvPendingInputs.push_back(std::move(input));
    }
    mCondNewKeyFrame.notify_one();
}

void PointCloudMapping::PublishRgbImage(const cv::Mat& color, const rclcpp::Time& stamp)
{
    if(color.empty())
        return;

    const auto t0 = std::chrono::steady_clock::now();
    const cv::Mat normalizedColor = NormalizeColorImage8U(color);
    if(normalizedColor.empty())
        return;

    std::string encoding;
    switch(normalizedColor.channels())
    {
        case 1:
            encoding = sensor_msgs::image_encodings::MONO8;
            break;
        case 3:
            encoding = sensor_msgs::image_encodings::BGR8;
            break;
        case 4:
            encoding = sensor_msgs::image_encodings::BGRA8;
            break;
        default:
            return;
    }
    sensor_msgs::msg::Image::SharedPtr rgbMsg =
        cv_bridge::CvImage(std_msgs::msg::Header(), encoding, normalizedColor).toImageMsg();
    rgbMsg->header.stamp = stamp;
    rgbMsg->header.frame_id = mConfig.cameraFrameId;
    const auto t1 = std::chrono::steady_clock::now();
    mRgbImagePublisher->publish(*rgbMsg);
    const auto t2 = std::chrono::steady_clock::now();

    if(mConfig.runtimeLogs)
    {
        static std::map<std::string, RosPublishTimingAccumulator> accumulators;
        RosPublishTimingAccumulator& acc = accumulators[mConfig.rgbImageTopic];
        const double encodeMs =
            std::chrono::duration<double, std::milli>(t1 - t0).count();
        const double publishMs =
            std::chrono::duration<double, std::milli>(t2 - t1).count();
        const double totalMs =
            std::chrono::duration<double, std::milli>(t2 - t0).count();
        ++acc.samples;
        acc.sumTotalMs += totalMs;
        acc.sumEncodeMs += encodeMs;
        acc.sumPublishMs += publishMs;
        acc.sumCols += normalizedColor.cols;
        acc.sumRows += normalizedColor.rows;
        acc.sumChannels += normalizedColor.channels();

        if(acc.samples >= kRuntimeLogAverageWindow)
        {
            std::cout << "[RosPublishTiming] topic=" << mConfig.rgbImageTopic
                      << " kind=rgb_image"
                      << " window=" << acc.samples
                      << " avg_total_ms=" << (acc.sumTotalMs / acc.samples)
                      << " avg_encode_ms=" << (acc.sumEncodeMs / acc.samples)
                      << " avg_publish_ms=" << (acc.sumPublishMs / acc.samples)
                      << " avg_cols=" << (acc.sumCols / acc.samples)
                      << " avg_rows=" << (acc.sumRows / acc.samples)
                      << " avg_channels=" << (acc.sumChannels / acc.samples)
                      << std::endl;
            accumulators[mConfig.rgbImageTopic] = RosPublishTimingAccumulator();
        }
    }
}

bool PointCloudMapping::buildDepthFromStereo(const CloudKeyFrameInput& input, cv::Mat& depth) const
{
    depth.release();
    if(input.color.empty() || input.rightColor.empty() ||
       input.color.size() != input.rightColor.size() ||
       input.bf <= 1e-6f)
    {
        return false;
    }

    cv::Mat leftGray = ToStereoGray(input.color);
    cv::Mat rightGray = ToStereoGray(input.rightColor);
    if(leftGray.empty() || rightGray.empty() || leftGray.size() != rightGray.size())
        return false;

    int numDisparities = std::max(16, mConfig.stereoNumDisparities);
    numDisparities = ((numDisparities + 15) / 16) * 16;
    int blockSize = std::max(3, mConfig.stereoBlockSize | 1);
    const int cn = 1;
    const int p1 = 8 * cn * blockSize * blockSize;
    const int p2 = 32 * cn * blockSize * blockSize;

    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(
        mConfig.stereoMinDisparity,
        numDisparities,
        blockSize,
        p1,
        p2,
        1,
        31,
        mConfig.stereoUniquenessRatio,
        50,
        2,
        cv::StereoSGBM::MODE_SGBM_3WAY);

    cv::Mat disparity16S;
    sgbm->compute(leftGray, rightGray, disparity16S);
    if(disparity16S.empty())
        return false;

    depth = cv::Mat(leftGray.size(), CV_32F, cv::Scalar(0));
    for(int v = 0; v < disparity16S.rows; ++v)
    {
        const short* disparityRow = disparity16S.ptr<short>(v);
        float* depthRow = depth.ptr<float>(v);
        for(int u = 0; u < disparity16S.cols; ++u)
        {
            const float disparity = static_cast<float>(disparityRow[u]) / 16.0f;
            if(disparity <= 0.5f)
                continue;
            depthRow[u] = input.bf / disparity;
        }
    }
    return true;
}

Eigen::Matrix4f PointCloudMapping::buildWorldRosFromCameraSlam(const CloudKeyFrameInput& input) const
{
    Eigen::Matrix4f worldFromCamera = Eigen::Matrix4f::Identity();
    if(input.keyFramePose.empty() || input.keyFramePose.rows < 4 || input.keyFramePose.cols < 4)
        return worldFromCamera;

    cv::Mat Rwc = input.keyFramePose.rowRange(0, 3).colRange(0, 3).t();
    cv::Mat twc = -Rwc * input.keyFramePose.rowRange(0, 3).col(3);

    Eigen::Matrix4f T_w_slam_from_c_slam = Eigen::Matrix4f::Identity();
    for(int r = 0; r < 3; ++r)
    {
        for(int c = 0; c < 3; ++c)
            T_w_slam_from_c_slam(r, c) = Rwc.at<float>(r, c);
        T_w_slam_from_c_slam(r, 3) = twc.at<float>(r);
    }

    return buildRosCameraFromSlamCamera() * T_w_slam_from_c_slam;
}

Eigen::Matrix4f PointCloudMapping::buildRosCameraFromSlamCamera() const
{
    Eigen::Matrix4f T_ros_from_slam = Eigen::Matrix4f::Identity();
    T_ros_from_slam << 0, 0, 1, 0,
                      -1, 0, 0, 0,
                       0,-1, 0, 0,
                       0, 0, 0, 1;
    return T_ros_from_slam;
}

void PointCloudMapping::transformSlamCameraCloudToRosCamera(
    const pcl::PointCloud<pcl::PointXYZRGB>& source,
    pcl::PointCloud<pcl::PointXYZRGB>& out) const
{
    pcl::transformPointCloud(source, out, buildRosCameraFromSlamCamera());
}

void PointCloudMapping::publishDynamicBoxes(const std::vector<DynamicBox3D>& dynamicBoxes,
                                            const rclcpp::Time& stamp)
{
    if(!mConfig.enableDynamicBoxes)
        return;

    visualization_msgs::msg::MarkerArray markerArray;

    visualization_msgs::msg::Marker clearMarker;
    clearMarker.header.frame_id = mConfig.worldFrameId;
    clearMarker.header.stamp = stamp;
    clearMarker.ns = "dynamic_boxes";
    clearMarker.action = visualization_msgs::msg::Marker::DELETEALL;
    markerArray.markers.push_back(clearMarker);

    for(const DynamicBox3D& dynamicBox : dynamicBoxes)
    {
        const int colorSeed = dynamicBox.trackId >= 0 ? dynamicBox.trackId : dynamicBox.markerId;
        const std_msgs::msg::ColorRGBA lineColor = MakeTrackColor(colorSeed, 1.0f);
        const std_msgs::msg::ColorRGBA cubeColor = MakeTrackColor(colorSeed, mConfig.dynamicBoxMarkerAlpha);
        const bool useMeshMarker = ShouldUseMeshMarker(mConfig, dynamicBox.className);

        if(useMeshMarker)
        {
            visualization_msgs::msg::Marker meshMarker;
            meshMarker.header.frame_id = mConfig.worldFrameId;
            meshMarker.header.stamp = stamp;
            meshMarker.ns = "dynamic_boxes_mesh";
            meshMarker.id = dynamicBox.markerId * 2;
            meshMarker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
            meshMarker.action = visualization_msgs::msg::Marker::ADD;
            meshMarker.pose.position = ToPointMsg(
                dynamicBox.center +
                Eigen::Vector3f(0.0f, 0.0f, dynamicBox.size.z() * mConfig.dynamicMeshZOffsetRatio));
            meshMarker.pose.orientation.w = 1.0;
            meshMarker.scale.x = mConfig.dynamicMeshScale;
            meshMarker.scale.y = mConfig.dynamicMeshScale;
            meshMarker.scale.z = mConfig.dynamicMeshScale;
            meshMarker.color = cubeColor;
            if(meshMarker.color.a <= 0.0f)
                meshMarker.color.a = 1.0f;
            meshMarker.mesh_resource = mConfig.dynamicMeshResource;
            meshMarker.mesh_use_embedded_materials = mConfig.dynamicMeshUseEmbeddedMaterials;
            markerArray.markers.push_back(meshMarker);
        }

        visualization_msgs::msg::Marker cubeMarker;
        cubeMarker.header.frame_id = mConfig.worldFrameId;
        cubeMarker.header.stamp = stamp;
        cubeMarker.ns = "dynamic_boxes_cube";
        cubeMarker.id = dynamicBox.markerId * 2;
        cubeMarker.type = visualization_msgs::msg::Marker::CUBE;
        cubeMarker.action = visualization_msgs::msg::Marker::ADD;
        cubeMarker.pose.position = ToPointMsg(dynamicBox.center);
        cubeMarker.pose.orientation.w = 1.0;
        const float visualScale = std::max(0.01f, mConfig.dynamicBoxVisualScale);
        cubeMarker.scale.x = std::max(0.01f, dynamicBox.size.x() * visualScale);
        cubeMarker.scale.y = std::max(0.01f, dynamicBox.size.y() * visualScale);
        cubeMarker.scale.z = std::max(0.01f, dynamicBox.size.z() * visualScale);
        cubeMarker.color = cubeColor;
        markerArray.markers.push_back(cubeMarker);

        visualization_msgs::msg::Marker textMarker;
        textMarker.header.frame_id = mConfig.worldFrameId;
        textMarker.header.stamp = stamp;
        textMarker.ns = "dynamic_boxes_text";
        textMarker.id = dynamicBox.markerId * 2 + 1;
        textMarker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        textMarker.action = visualization_msgs::msg::Marker::ADD;
        textMarker.pose.position = ToPointMsg(
            dynamicBox.center + Eigen::Vector3f(0.0f, 0.0f, 0.5f * dynamicBox.size.z() + 0.12f));
        textMarker.pose.orientation.w = 1.0;
        textMarker.scale.z = mConfig.dynamicBoxTextScale;
        textMarker.color = lineColor;
        textMarker.color.a = 1.0f;
        textMarker.text = dynamicBox.className;
        if(dynamicBox.trackId >= 0)
            textMarker.text += " #" + std::to_string(dynamicBox.trackId);
        markerArray.markers.push_back(textMarker);
    }

    mDynamicBoxPublisher->publish(markerArray);
}

void PointCloudMapping::publishTfForKeyFrame(const CloudKeyFrameInput& input)
{
    cv::Mat Rwc(3, 3, CV_32F);
    cv::Mat twc(3, 1, CV_32F);
    Rwc = input.keyFramePose.rowRange(0, 3).colRange(0, 3).t();
    twc = -Rwc * input.keyFramePose.rowRange(0, 3).col(3);

    Eigen::Matrix<double, 3, 3> rotationMat;
    rotationMat << Rwc.at<float>(0,0), Rwc.at<float>(0,1), Rwc.at<float>(0,2),
                   Rwc.at<float>(1,0), Rwc.at<float>(1,1), Rwc.at<float>(1,2),
                   Rwc.at<float>(2,0), Rwc.at<float>(2,1), Rwc.at<float>(2,2);
    Eigen::Quaterniond q(rotationMat);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = input.stamp;
    transform.header.frame_id = mConfig.worldFrameId;
    transform.child_frame_id = mConfig.cameraFrameId;
    transform.transform.translation.x = twc.at<float>(2);
    transform.transform.translation.y = -twc.at<float>(0);
    transform.transform.translation.z = -twc.at<float>(1);
    transform.transform.rotation.x = q.z();
    transform.transform.rotation.y = -q.x();
    transform.transform.rotation.z = -q.y();
    transform.transform.rotation.w = q.w();
    mTfBroadcaster->sendTransform(transform);
}

void PointCloudMapping::OnOctomapBinary(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
    if(!msg)
        return;

    const auto now = std::chrono::steady_clock::now();
    rclcpp::Time localCloudStamp;
    std::chrono::steady_clock::time_point localCloudPublishedAt;
    long unsigned int localCloudKeyFrameId = std::numeric_limits<long unsigned int>::max();

    {
        std::lock_guard<std::mutex> lock(mMutexOctomapTiming);
        if(mLastLocalCloudStamp.nanoseconds() == 0)
            return;

        const rclcpp::Time messageStamp(msg->header.stamp);
        if(mLastOctomapStamp.nanoseconds() != 0 && messageStamp == mLastOctomapStamp)
            return;

        mLastOctomapStamp = messageStamp;
        localCloudStamp = mLastLocalCloudStamp;
        localCloudPublishedAt = mLastLocalCloudPublishedAt;
        localCloudKeyFrameId = mLastLocalCloudKeyFrameId;
    }

    if(!mConfig.runtimeLogs)
        return;

    const double publishToMapMs =
        std::chrono::duration<double, std::milli>(now - localCloudPublishedAt).count();
    const double stampDeltaMs =
        rclcpp::Time(msg->header.stamp).nanoseconds() == 0 ? -1.0 :
        (rclcpp::Time(msg->header.stamp) - localCloudStamp).seconds() * 1000.0;

    static std::map<std::string, OctomapTimingAccumulator> accumulators;
    OctomapTimingAccumulator& acc = accumulators[mConfig.octomapBinaryTopic];
    ++acc.samples;
    acc.sumPublishToOctomapMs += publishToMapMs;
    if(stampDeltaMs >= 0.0)
    {
        acc.sumStampDeltaMs += stampDeltaMs;
        ++acc.stampDeltaCount;
    }

    if(acc.samples < kRuntimeLogAverageWindow)
        return;

    std::cout << "[OctoMapTiming] topic=" << mConfig.octomapBinaryTopic
              << " window=" << acc.samples
              << " avg_publish_to_octomap_ms=" << (acc.sumPublishToOctomapMs / acc.samples);
    if(acc.stampDeltaCount > 0)
        std::cout << " avg_stamp_delta_ms=" << (acc.sumStampDeltaMs / acc.stampDeltaCount);
    std::cout << " last_kf=" << localCloudKeyFrameId
              << std::endl;

    accumulators[mConfig.octomapBinaryTopic] = OctomapTimingAccumulator();
}

void PointCloudMapping::MapViewer()
{
    std::cout << "Start PointCloudMapping Viewer..." << std::endl;

    while(true)
    {
        CloudKeyFrameInput input;
        {
            std::unique_lock<std::mutex> lock(mMutexKeyFrameQueue);
            mCondNewKeyFrame.wait(lock, [this] {
                return mbShutdownRequested || !mvPendingInputs.empty();
            });

            if(mbShutdownRequested && mvPendingInputs.empty())
                break;

            input = std::move(mvPendingInputs.front());
            mvPendingInputs.pop_front();
        }

        const auto tMap0 = std::chrono::steady_clock::now();
        PointCloudMapTimingSummary timing;
        timing.keyFrameId = input.keyFrameId;
        if(input.enqueuedAt.time_since_epoch().count() != 0)
        {
            timing.queueMs =
                std::chrono::duration<double, std::milli>(tMap0 - input.enqueuedAt).count();
        }

        mpLocalCameraCloud->clear();
        mpVoxelCloud->clear();
        mpFilteredCloud->clear();
        mpRosAlignedCloud->clear();

        if(input.depth.empty() && !input.rightColor.empty())
        {
            const auto tDepth0 = std::chrono::steady_clock::now();
            timing.builtDepthFromStereo = buildDepthFromStereo(input, input.depth);
            const auto tDepth1 = std::chrono::steady_clock::now();
            timing.depthMs =
                std::chrono::duration<double, std::milli>(tDepth1 - tDepth0).count();
        }

        const Eigen::Matrix4f worldFromCamera = buildWorldRosFromCameraSlam(input);
        std::vector<DynamicBox3D> dynamicBoxes;
        const auto tBoxes0 = std::chrono::steady_clock::now();
        generateDynamicBoxes(input, worldFromCamera, dynamicBoxes);
        const auto tBoxes1 = std::chrono::steady_clock::now();
        timing.dynamicBoxesMs =
            std::chrono::duration<double, std::milli>(tBoxes1 - tBoxes0).count();
        timing.dynamicBoxes = dynamicBoxes.size();

        const auto tStatic0 = std::chrono::steady_clock::now();
        generateStaticPointCloud(input, mpLocalCameraCloud);
        const auto tStatic1 = std::chrono::steady_clock::now();
        timing.staticCloudMs =
            std::chrono::duration<double, std::milli>(tStatic1 - tStatic0).count();
        timing.rawPoints = mpLocalCameraCloud->size();
        if(mpLocalCameraCloud->empty())
        {
            const auto tTf0 = std::chrono::steady_clock::now();
            publishTfForKeyFrame(input);
            const auto tTf1 = std::chrono::steady_clock::now();
            timing.tfMs =
                std::chrono::duration<double, std::milli>(tTf1 - tTf0).count();
            const auto tMarker0 = std::chrono::steady_clock::now();
            publishDynamicBoxes(dynamicBoxes, input.stamp);
            const auto tMarker1 = std::chrono::steady_clock::now();
            timing.markerPublishMs =
                std::chrono::duration<double, std::milli>(tMarker1 - tMarker0).count();
            timing.exitStage = "empty_static_cloud";
            timing.totalMs =
                std::chrono::duration<double, std::milli>(tMarker1 - tMap0).count();
            PrintPointCloudMapTimingSummary(mConfig, timing);
            continue;
        }

        if(mConfig.voxelLeafSize > 0.0f)
        {
            const auto tVoxel0 = std::chrono::steady_clock::now();
            mVoxelLocal.setInputCloud(mpLocalCameraCloud);
            mVoxelLocal.filter(*mpVoxelCloud);
            const auto tVoxel1 = std::chrono::steady_clock::now();
            timing.voxelMs =
                std::chrono::duration<double, std::milli>(tVoxel1 - tVoxel0).count();
        }
        else
        {
            *mpVoxelCloud = *mpLocalCameraCloud;
            timing.voxelMs = 0.0;
        }
        timing.voxelPoints = mpVoxelCloud->size();

        if(mpVoxelCloud->empty())
        {
            const auto tTf0 = std::chrono::steady_clock::now();
            publishTfForKeyFrame(input);
            const auto tTf1 = std::chrono::steady_clock::now();
            timing.tfMs =
                std::chrono::duration<double, std::milli>(tTf1 - tTf0).count();
            const auto tMarker0 = std::chrono::steady_clock::now();
            publishDynamicBoxes(dynamicBoxes, input.stamp);
            const auto tMarker1 = std::chrono::steady_clock::now();
            timing.markerPublishMs =
                std::chrono::duration<double, std::milli>(tMarker1 - tMarker0).count();
            timing.exitStage = "empty_voxel_cloud";
            timing.totalMs =
                std::chrono::duration<double, std::milli>(tMarker1 - tMap0).count();
            PrintPointCloudMapTimingSummary(mConfig, timing);
            continue;
        }

        if(mConfig.sorMeanK > 0 &&
           mpVoxelCloud->size() >= static_cast<std::size_t>(std::max(3, mConfig.sorMeanK)))
        {
            const auto tSor0 = std::chrono::steady_clock::now();
            mSorLocal.setInputCloud(mpVoxelCloud);
            mSorLocal.filter(*mpFilteredCloud);
            const auto tSor1 = std::chrono::steady_clock::now();
            timing.sorMs =
                std::chrono::duration<double, std::milli>(tSor1 - tSor0).count();
        }
        else
        {
            *mpFilteredCloud = *mpVoxelCloud;
            timing.sorMs = 0.0;
        }
        timing.filteredPoints = mpFilteredCloud->size();

        if(mpFilteredCloud->empty())
        {
            const auto tTf0 = std::chrono::steady_clock::now();
            publishTfForKeyFrame(input);
            const auto tTf1 = std::chrono::steady_clock::now();
            timing.tfMs =
                std::chrono::duration<double, std::milli>(tTf1 - tTf0).count();
            const auto tMarker0 = std::chrono::steady_clock::now();
            publishDynamicBoxes(dynamicBoxes, input.stamp);
            const auto tMarker1 = std::chrono::steady_clock::now();
            timing.markerPublishMs =
                std::chrono::duration<double, std::milli>(tMarker1 - tMarker0).count();
            timing.exitStage = "empty_filtered_cloud";
            timing.totalMs =
                std::chrono::duration<double, std::milli>(tMarker1 - tMap0).count();
            PrintPointCloudMapTimingSummary(mConfig, timing);
            continue;
        }

        const auto tTransform0 = std::chrono::steady_clock::now();
        transformSlamCameraCloudToRosCamera(*mpFilteredCloud, *mpRosAlignedCloud);
        const auto tTransform1 = std::chrono::steady_clock::now();
        timing.transformMs =
            std::chrono::duration<double, std::milli>(tTransform1 - tTransform0).count();
        timing.localPoints = mpRosAlignedCloud->size();

        if(mpRosAlignedCloud->empty())
        {
            const auto tTf0 = std::chrono::steady_clock::now();
            publishTfForKeyFrame(input);
            const auto tTf1 = std::chrono::steady_clock::now();
            timing.tfMs =
                std::chrono::duration<double, std::milli>(tTf1 - tTf0).count();
            const auto tMarker0 = std::chrono::steady_clock::now();
            publishDynamicBoxes(dynamicBoxes, input.stamp);
            const auto tMarker1 = std::chrono::steady_clock::now();
            timing.markerPublishMs =
                std::chrono::duration<double, std::milli>(tMarker1 - tMarker0).count();
            timing.exitStage = "empty_ros_cloud";
            timing.totalMs =
                std::chrono::duration<double, std::milli>(tMarker1 - tMap0).count();
            PrintPointCloudMapTimingSummary(mConfig, timing);
            continue;
        }

        sensor_msgs::msg::PointCloud2 localMsg;
        const auto tRosMsg0 = std::chrono::steady_clock::now();
        pcl::toROSMsg(*mpRosAlignedCloud, localMsg);
        const auto tRosMsg1 = std::chrono::steady_clock::now();
        timing.rosMsgMs =
            std::chrono::duration<double, std::milli>(tRosMsg1 - tRosMsg0).count();
        localMsg.header.stamp = input.stamp;
        localMsg.header.frame_id = mConfig.cameraFrameId;

        const auto tTf0 = std::chrono::steady_clock::now();
        publishTfForKeyFrame(input);
        const auto tTf1 = std::chrono::steady_clock::now();
        timing.tfMs =
            std::chrono::duration<double, std::milli>(tTf1 - tTf0).count();
        const auto tCloudPub0 = std::chrono::steady_clock::now();
        mLocalCloudPublisher->publish(localMsg);
        const auto tCloudPub1 = std::chrono::steady_clock::now();
        timing.cloudPublishMs =
            std::chrono::duration<double, std::milli>(tCloudPub1 - tCloudPub0).count();
        {
            std::lock_guard<std::mutex> lock(mMutexOctomapTiming);
            mLastLocalCloudStamp = input.stamp;
            mLastLocalCloudPublishedAt = tCloudPub1;
            mLastLocalCloudKeyFrameId = input.keyFrameId;
        }
        const auto tMarker0 = std::chrono::steady_clock::now();
        publishDynamicBoxes(dynamicBoxes, input.stamp);
        const auto tMarker1 = std::chrono::steady_clock::now();
        timing.markerPublishMs =
            std::chrono::duration<double, std::milli>(tMarker1 - tMarker0).count();
        timing.exitStage = "published";
        timing.totalMs =
            std::chrono::duration<double, std::milli>(tMarker1 - tMap0).count();
        PrintPointCloudMapTimingSummary(mConfig, timing);

        // std::cout << "[PointCloudMapping] kf=" << input.keyFrameId
        //           << " raw_points=" << mpLocalCameraCloud->size()
        //           << " local_points=" << mpRosAlignedCloud->size()
        //           << " cloud_frame=" << mConfig.cameraFrameId
        //           << " dynamic_boxes=" << dynamicBoxes.size()
        //           << " topic=" << mConfig.localCloudTopic
        //           << std::endl;
    }
}

void PointCloudMapping::shutdown()
{
    {
        std::lock_guard<std::mutex> lock(mMutexKeyFrameQueue);
        if(mbShutdownRequested)
            return;
        mbShutdownRequested = true;
    }
    mCondNewKeyFrame.notify_all();
    if(mExecutor)
        mExecutor->cancel();

    if(mViewerThread.joinable())
        mViewerThread.join();
    if(mRosSpinThread.joinable())
        mRosSpinThread.join();
}

} // namespace ORB_SLAM2
