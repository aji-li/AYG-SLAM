#ifndef ORB_SLAM2_POINTCLOUDMAPPING_H
#define ORB_SLAM2_POINTCLOUDMAPPING_H

#include "KeyFrame.h"
#include "YoloDetector.h"

#include <condition_variable>
#include <chrono>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core/core.hpp>

#include <Eigen/Core>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <rclcpp/rclcpp.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

namespace ORB_SLAM2
{

struct PointCloudMappingConfig
{
    bool runtimeLogs = false;
    bool considerDynamic = true;
    float cameraValidDepthMin = 0.5f;
    float cameraValidDepthMax = 5.0f;
    float voxelLeafSize = 0.05f;
    int sorMeanK = 50;
    double sorStddevMulThresh = 3.0;
    std::string localCloudTopic = "/AYG/Local_Point_Clouds";
    std::string rgbImageTopic = "/AYG/RGB";
    std::string cameraFrameId = "AYG/camera_sensor";
    std::string worldFrameId = "AYG/map";
    std::string dynamicBoxTopic = "/AYG/Dynamic3DBoxes";
    std::string octomapBinaryTopic = "/octomap_binary";
    bool enableDynamicBoxes = true;
    int dynamicBoxMinPoints = 80;
    int dynamicMaskErodeSize = 3;
    float dynamicDepthTrimRatio = 0.1f;
    bool dynamicBoxUseClustering = true;
    float dynamicBoxRoiInsetRatio = 0.2f;
    float dynamicBoxClusterTolerance = 0.05f;
    int dynamicBoxClusterMinSize = 120;
    int dynamicBoxClusterMaxSize = 30000;
    float dynamicBoxClusterCompareRatio = 0.1f;
    float dynamicBoxMarkerAlpha = 0.35f;
    float dynamicBoxLineWidth = 0.04f;
    float dynamicBoxTextScale = 0.18f;
    float dynamicBoxVisualScale = 1.0f;
    bool dynamicMeshEnable = false;
    std::string dynamicMeshClassName = "uav";
    std::string dynamicMeshResource;
    bool dynamicMeshUseEmbeddedMaterials = false;
    float dynamicMeshScale = 1.0f;
    float dynamicMeshZOffsetRatio = 0.0f;
    int stereoMinDisparity = 0;
    int stereoNumDisparities = 128;
    int stereoBlockSize = 5;
    int stereoUniquenessRatio = 10;
};

class PointCloudMapping
{
public:
    explicit PointCloudMapping(const PointCloudMappingConfig& config);
    ~PointCloudMapping();

    PointCloudMapping(const PointCloudMapping&) = delete;
    PointCloudMapping& operator=(const PointCloudMapping&) = delete;

    void insertKeyFrame(KeyFrame* pKF,
                        const cv::Mat& color,
                        const cv::Mat& depth,
                        const std::vector<YoloDetection>& detections);
    void insertStereoKeyFrame(KeyFrame* pKF,
                              const cv::Mat& leftColor,
                              const cv::Mat& rightColor,
                              const float bf,
                              const std::vector<YoloDetection>& detections);
    void PublishRgbImage(const cv::Mat& color, const rclcpp::Time& stamp);
    void SpinSome();
    void shutdown();

private:
    struct CloudKeyFrameInput
    {
        long unsigned int keyFrameId = 0;
        cv::Mat keyFramePose;
        float fx = 0.0f;
        float fy = 0.0f;
        float cx = 0.0f;
        float cy = 0.0f;
        float bf = 0.0f;
        cv::Mat color;
        cv::Mat rightColor;
        cv::Mat depth;
        std::vector<YoloDetection> detections;
        rclcpp::Time stamp;
        std::chrono::steady_clock::time_point enqueuedAt;
    };

    struct DynamicBox3D
    {
        int markerId = -1;
        int trackId = -1;
        int classId = -1;
        float confidence = 0.0f;
        std::string className;
        Eigen::Vector3f minCorner = Eigen::Vector3f::Zero();
        Eigen::Vector3f maxCorner = Eigen::Vector3f::Zero();
        Eigen::Vector3f center = Eigen::Vector3f::Zero();
        Eigen::Vector3f size = Eigen::Vector3f::Zero();
    };

    void MapViewer();
    bool isInDynamicRegion(int x, int y, const std::vector<YoloDetection>& detections) const;
    bool buildDepthFromStereo(const CloudKeyFrameInput& input, cv::Mat& depth) const;
    void generateStaticPointCloud(const CloudKeyFrameInput& input,
                                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cameraCloud) const;
    Eigen::Matrix4f buildWorldRosFromCameraSlam(const CloudKeyFrameInput& input) const;
    Eigen::Matrix4f buildRosCameraFromSlamCamera() const;
    void transformSlamCameraCloudToRosCamera(const pcl::PointCloud<pcl::PointXYZRGB>& source,
                                             pcl::PointCloud<pcl::PointXYZRGB>& out) const;
    void generateDynamicBoxes(const CloudKeyFrameInput& input,
                              const Eigen::Matrix4f& worldFromCamera,
                              std::vector<DynamicBox3D>& dynamicBoxes) const;
    void publishDynamicBoxes(const std::vector<DynamicBox3D>& dynamicBoxes,
                             const rclcpp::Time& stamp);
    void publishTfForKeyFrame(const CloudKeyFrameInput& input);
    void OnOctomapBinary(const octomap_msgs::msg::Octomap::SharedPtr msg);

private:
    PointCloudMappingConfig mConfig;

    std::thread mViewerThread;
    std::thread mRosSpinThread;
    std::condition_variable mCondNewKeyFrame;
    std::mutex mMutexKeyFrameQueue;
    std::mutex mMutexShutdown;
    std::deque<CloudKeyFrameInput> mvPendingInputs;
    bool mbShutdownRequested = false;

    rclcpp::Node::SharedPtr mNode;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mLocalCloudPublisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mRgbImagePublisher;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mDynamicBoxPublisher;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr mOctomapBinarySubscriber;
    std::unique_ptr<tf2_ros::TransformBroadcaster> mTfBroadcaster;
    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> mExecutor;

    std::mutex mMutexOctomapTiming;
    rclcpp::Time mLastLocalCloudStamp;
    std::chrono::steady_clock::time_point mLastLocalCloudPublishedAt;
    long unsigned int mLastLocalCloudKeyFrameId = std::numeric_limits<long unsigned int>::max();
    rclcpp::Time mLastOctomapStamp;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mpLocalCameraCloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mpVoxelCloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mpFilteredCloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mpRosAlignedCloud;

    pcl::VoxelGrid<pcl::PointXYZRGB> mVoxelLocal;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> mSorLocal;
};

} // namespace ORB_SLAM2

#endif // ORB_SLAM2_POINTCLOUDMAPPING_H
