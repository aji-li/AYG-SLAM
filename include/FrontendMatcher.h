#ifndef FRONTENDMATCHER_H
#define FRONTENDMATCHER_H

#include <vector>

#include <opencv2/core/core.hpp>

#include "Frame.h"
#include "KeyFrame.h"
#include "MapPoint.h"

namespace ORB_SLAM2
{

class FrontendMatcher
{
public:
    FrontendMatcher(float minScore = 0.75f, float secondBestMargin = 0.02f, bool checkOri = true);

    int SearchByProjection(Frame &F, const std::vector<MapPoint*> &vpMapPoints, const float th = 3.0f);
    int SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono);
    int SearchForTriangulation(KeyFrame *pKF1, KeyFrame *pKF2, const cv::Mat &F12,
                               std::vector<std::pair<size_t, size_t> > &vMatchedPairs,
                               const bool bOnlyStereo = false);
    int Fuse(KeyFrame *pKF, const std::vector<MapPoint*> &vpMapPoints, const float th = 3.0f);

private:
    float DescriptorSimilarity(const cv::Mat &a, const cv::Mat &b) const;
    bool CheckDistEpipolarLine(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2,
                               const cv::Mat &F12, const KeyFrame *pKF) const;
    void ComputeThreeMaxima(std::vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3) const;

    float mfMinScore;
    float mfSecondBestMargin;
    bool mbCheckOrientation;

    static const int HISTO_LENGTH = 30;
};

} // namespace ORB_SLAM2

#endif // FRONTENDMATCHER_H
