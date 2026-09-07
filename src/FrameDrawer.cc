/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "FrameDrawer.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <algorithm>
#include <iomanip>
#include <sstream>
#include<mutex>

namespace ORB_SLAM2
{

FrameDrawer::FrameDrawer(Map* pMap):mpMap(pMap)
{
    mState=Tracking::SYSTEM_NOT_READY;
    mnYoloResultFrameId = 0;
    mnDynamicMaskCoreErodeSize = 9;
    mnDynamicRegionMode = Tracking::DYNAMIC_REGION_BBOX;
    mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
}

cv::Mat FrameDrawer::DrawFrame()
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbVO, vbMap, vbDynamic; // Tracked MapPoints in current frame
    std::vector<YoloDetection> vYoloDetections;
    long unsigned int yoloResultFrameId = 0;
    int coreErodeSize = 9;
    int regionMode = Tracking::DYNAMIC_REGION_BBOX;
    int state; // Tracking state

    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mIm.copyTo(im);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeys;
            vbVO = mvbVO;
            vbMap = mvbMap;
            vbDynamic = mvbDynamic;
        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeys;
            vbDynamic = mvbDynamic;
        }
        vYoloDetections = mvYoloDetections;
        yoloResultFrameId = mnYoloResultFrameId;
        coreErodeSize = mnDynamicMaskCoreErodeSize;
        regionMode = mnDynamicRegionMode;
    } // destroy scoped mutex -> release mutex

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,cv::COLOR_GRAY2BGR);

    //Draw
    if(state==Tracking::NOT_INITIALIZED) //INITIALIZING
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::line(im,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt,
                        cv::Scalar(0,255,0));
            }
        }
    }
    else if(state==Tracking::OK) //TRACKING
    {
        mnTracked=0;
        mnTrackedVO=0;
        const float r = 5;
        const int n = vCurrentKeys.size();
        for(int i=0;i<n;i++)
        {
            if(vbVO[i] || vbMap[i])
            {
                cv::Point2f pt1,pt2;
                pt1.x=vCurrentKeys[i].pt.x-r;
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;
                pt2.y=vCurrentKeys[i].pt.y+r;

                const bool isDynamic = (i < static_cast<int>(vbDynamic.size())) ? vbDynamic[i] : false;

                // This is a match to a MapPoint in the map
                if(vbMap[i])
                {
                    const cv::Scalar color = isDynamic ? cv::Scalar(0,0,255) : cv::Scalar(0,255,0);
                    cv::rectangle(im,pt1,pt2,color);
                    cv::circle(im,vCurrentKeys[i].pt,2,color,-1);
                    mnTracked++;
                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    const cv::Scalar color = isDynamic ? cv::Scalar(0,0,255) : cv::Scalar(255,0,0);
                    cv::rectangle(im,pt1,pt2,color);
                    cv::circle(im,vCurrentKeys[i].pt,2,color,-1);
                    mnTrackedVO++;
                }
            }
            else if(i < static_cast<int>(vbDynamic.size()) && vbDynamic[i])
            {
                cv::circle(im, vCurrentKeys[i].pt, 2, cv::Scalar(0,0,255), -1);
            }
        }
    }
    else if(state==Tracking::LOST)
    {
        for(int i = 0; i < static_cast<int>(vCurrentKeys.size()); ++i)
        {
            if(i < static_cast<int>(vbDynamic.size()) && vbDynamic[i])
                cv::circle(im, vCurrentKeys[i].pt, 2, cv::Scalar(0,0,255), -1);
        }
    }

    for(size_t i = 0; i < vYoloDetections.size(); ++i)
    {
        const YoloDetection& det = vYoloDetections[i];
        if(regionMode == Tracking::DYNAMIC_REGION_SEG && det.HasMask())
        {
            cv::Mat mask;
            if(det.mask.size() != im.size())
                cv::resize(det.mask, mask, im.size(), 0, 0, cv::INTER_NEAREST);
            else
                mask = det.mask;

            cv::Mat maskBinary;
            if(mask.type() != CV_8U)
                mask.convertTo(maskBinary, CV_8U);
            else
                maskBinary = mask;
            cv::threshold(maskBinary, maskBinary, 0, 255, cv::THRESH_BINARY);

            cv::Mat coreMask = maskBinary.clone();
            if(coreErodeSize > 1)
            {
                const int ksize = (coreErodeSize % 2 == 0) ? (coreErodeSize + 1) : coreErodeSize;
                const cv::Mat kernel = cv::getStructuringElement(
                    cv::MORPH_ELLIPSE, cv::Size(ksize, ksize));
                cv::erode(coreMask, coreMask, kernel);
            }

            cv::Mat boundaryMask;
            cv::subtract(maskBinary, coreMask, boundaryMask);

            cv::Mat overlay = im.clone();
            overlay.setTo(cv::Scalar(0, 0, 255), coreMask);
            overlay.setTo(cv::Scalar(0, 215, 255), boundaryMask);
            cv::addWeighted(overlay, 0.30, im, 0.70, 0.0, im);
        }
        else
        {
            cv::rectangle(im, det.box, cv::Scalar(0, 215, 255), 2);
        }

        if(det.HasTrack())
        {
            const std::string label = "ID " + std::to_string(det.trackId);
            const cv::Point origin(det.box.x, std::max(18, det.box.y - 6));
            cv::putText(im, label, origin, cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 0, 0), 3);
            cv::putText(im, label, origin, cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 255), 1);
        }
    }

    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);

    return imWithInfo;
}


void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
    stringstream s;
    if(nState==Tracking::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if(nState==Tracking::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if(nState==Tracking::OK)
    {
        if(!mbOnlyTracking)
            s << "SLAM MODE |  ";
        else
            s << "LOCALIZATION | ";
        int nKFs = mpMap->KeyFramesInMap();
        int nMPs = mpMap->MapPointsInMap();
        s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked;
        if(mnTrackedVO>0)
            s << ", + VO matches: " << mnTrackedVO;
    }
    else if(nState==Tracking::LOST)
    {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if(nState==Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    int baseline=0;
    cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);

    imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);

}

void FrameDrawer::Update(Tracking *pTracker)
{
    unique_lock<mutex> lock(mMutex);
    if(!pTracker->mImDraw.empty())
        pTracker->mImDraw.copyTo(mIm);
    else
        pTracker->mImGray.copyTo(mIm);
    mvCurrentKeys=pTracker->mCurrentFrame.mvKeys;
    N = mvCurrentKeys.size();
    mvbVO = vector<bool>(N,false);
    mvbMap = vector<bool>(N,false);
    mvbDynamic = pTracker->mCurrentFrame.mvbDynamic;
    if(static_cast<int>(mvbDynamic.size()) < N)
        mvbDynamic.resize(N, false);
    mbOnlyTracking = pTracker->mbOnlyTracking;
    mvYoloDetections = pTracker->GetCurrentYoloDetections();
    mnYoloResultFrameId = pTracker->GetLatestYoloResultFrameId();
    mnDynamicMaskCoreErodeSize = pTracker->GetDynamicMaskCoreErodeSize();
    mnDynamicRegionMode = static_cast<int>(pTracker->GetDynamicRegionMode());


    if(pTracker->mLastProcessedState==Tracking::NOT_INITIALIZED)
    {
        mvIniKeys=pTracker->mInitialFrame.mvKeys;
        mvIniMatches=pTracker->mvIniMatches;
    }
    else if(pTracker->mLastProcessedState==Tracking::OK)
    {
        for(int i=0;i<N;i++)
        {
            MapPoint* pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(!pTracker->mCurrentFrame.mvbOutlier[i])
                {
                    if(pMP->Observations()>0)
                        mvbMap[i]=true;
                    else
                        mvbVO[i]=true;
                }
            }
        }
    }
    mState=static_cast<int>(pTracker->mLastProcessedState);
}

} //namespace ORB_SLAM
