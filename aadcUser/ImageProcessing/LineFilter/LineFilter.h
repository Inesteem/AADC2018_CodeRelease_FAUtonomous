/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. Team FAUtonomous. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

**********************************************************************/

#pragma once
#define CID_LINE_FILTER_FILTER "line_filter.filter.user.aadc.cid"

// namespaces
using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;
using namespace cv;

// cLineFilter
class cLineFilter
{
    static const tFloat32 SIZE_X;
    static const tFloat32 SIZE_Y;

    tFloat32 _near;
    tFloat32 _far;

    tFloat32 _fovHorizontal;
    tFloat32 _fovVertical;

    tUInt32 _cameraWidth;
    tUInt32 _cameraHeight;

    tFloat32 _pitch;

    tFloat32 _cameraOffsetX;
    tFloat32 _cameraOffsetZ;

    cv::Mat _topDownMatrix;

    /**
      * indicates if we use a median filter on the binary image
      */
    tBool _enableMedianFiltering;

    /**
     * adaptive thresholding constant. In general negative.
     */
    tFloat32 _adaptiveThresholdConstant;
    tUInt32 _adaptiveThresholdWidth;

    /**
     * enable or disable adaptive thresholding
     */
    tBool _enableAdaptiveThresholding;

    tBool _isInitialized;

    /**
     * some temporaries
     */
    Mat _buffer;
    Mat _gray;
    Mat _validPixels;
    Mat _lastValidPixels;

    tUInt32 _maskWorkRegion;
    Mat _lastMask;
    Mat _lastlastMask;
    Mat _warp;
    Mat _affineMask;

public:


    cLineFilter();
    virtual ~cLineFilter();

    /**
     *
     * @param near
     * @param fovHorizontal
     * @param fovVertical
     * @param pitch
     * @param cameraPositionX
     * @param cameraPositionY
     * @param cameraWidth
     * @param cameraHeight
     */
    void Init(tFloat32 near, tFloat32 fovHorizontal, tFloat32 fovVertical, tFloat32 pitch, tFloat32 cameraPositionY,
                    tFloat32 cameraPositionZ, tFloat32 adaptiveThresholdConstant, tUInt32 adaptiveThresholdWidth, tUInt32 cameraWidth, tUInt32 cameraHeight);

    void EnableAdaptiveThresholding(tBool enable);
    void EnableMedianFiltering(tBool enable);
    void SetNearFar(tFloat32 near);
    void SetFOV(tFloat32 fovHorizontal, tFloat32 fovVertical);
    void SetPitch(tFloat32 pitch);
    tFloat32 GetPitch() {
            return _pitch;
    }
    void SetCameraPosition(tFloat32 cameraPositionX, tFloat32 cameraPositionY);
    void SetCameraSize(tUInt32 cameraWidth, tUInt32 cameraHeight);
    void SetAdaptiveThresholdConstant(tFloat32 adaptiveThresholdConstant);
    void SetAdaptiveThresholdWidth(tUInt32 adaptiveThresholdWidth){
            _adaptiveThresholdWidth = adaptiveThresholdWidth;
    }

    void SetValidPixels(cv::Mat &mask, Size s, tBool enable);

    virtual tResult GetTopDownView(cv::Mat &in, cv::Mat& roi, cv::Mat& out, tBool use_CV_rgb2gray);

    cv::Point2f GetTopLeftWorldCoordinates();

private:
    void CalculatePerspectiveTransformationMatrix();
    void CalculateSourcePoints(cv::Point2f * buffer);


}; // cLineFilter
