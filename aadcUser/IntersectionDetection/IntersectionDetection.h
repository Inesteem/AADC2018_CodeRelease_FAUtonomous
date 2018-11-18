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
#define CID_INTERSECTION_DETECTION_FILTER "intersection_detection.filter.user.aadc.cid"

// namespaces
using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;

// IntersectionDetection
class IntersectionDetection : public cTriggerFunction
{
private:

    // ------- PINS -------------
    // input
    cPinReader m_ReaderVideo;

    // output
    cPinWriter m_WriterDebugVideo;
    cPinWriter m_WriterDebugBirdVideo;


    // The clock
    object_ptr<adtf::services::IReferenceClock> m_pClock;

    // image formats for video pins
    adtf::streaming::tStreamImageFormat m_sInputImageFormat;
    adtf::streaming::tStreamImageFormat m_sOutputImageFormat;


    // points
    Point2f m_p2fROIVertices[4];              // ROI
    Point2f m_p2fTransformedImageVertices[4]; //
    
    // output debug image
    Mat m_MatOutputDebugImage;       // for ROIs

    // transformed image after bird's eye view transform
    Mat m_MatTransformedImage;

    // input image
    Mat m_MatInputImage;

    // check free lane
    bool m_bFreeRois[10] = {false};


    // -------- PROPERTIES -------
    // ROI Properties: 4 points with x and y values
    adtf::base::property_variable<tUInt32> m_propROIPoint1X;
    adtf::base::property_variable<tUInt32> m_propROIPoint1Y;
    adtf::base::property_variable<tUInt32> m_propROIPoint2X;
    adtf::base::property_variable<tUInt32> m_propROIPoint2Y;
    adtf::base::property_variable<tUInt32> m_propROIPoint3X;
    adtf::base::property_variable<tUInt32> m_propROIPoint3Y;
    adtf::base::property_variable<tUInt32> m_propROIPoint4X;
    adtf::base::property_variable<tUInt32> m_propROIPoint4Y;

    // Transformed Image Properties: width, length
    adtf::base::property_variable<tUInt32> m_propTransformedImageWidth;
    adtf::base::property_variable<tUInt32> m_propTransformedImageHeight;

    // ImageProcessing
    adtf::base::property_variable<tBool>   m_propEnableGaussianBlur;
    adtf::base::property_variable<tUInt32> m_propGaussianKernelSize;
    adtf::base::property_variable<tBool>   m_propEnableBinaryThreshold;
    adtf::base::property_variable<tUInt32> m_propBinaryThreshold;
    adtf::base::property_variable<tBool>   m_propEnableCornerHarris;
    adtf::base::property_variable<tUInt32> m_propHarrisBlockSize;
    adtf::base::property_variable<tUInt32> m_propHarrisAperture;
    adtf::base::property_variable<tUInt32> m_propHarrisThreshold;

    adtf::base::property_variable<tUInt32> m_propLaneROIWidth;
    adtf::base::property_variable<tUInt32> m_propFrame;

    tUInt32 m_ui32FrameCounter;

public:
    /*------------ FUNCTIONS -----------*/
    /*------------ FROM AUDI -----------*/
    // constructor
    IntersectionDetection();

    // destructor
    ~IntersectionDetection() = default;

    // Configure
    virtual tResult Configure() override;

    // Process
    virtual tResult Process(tTimeStamp tmTimeOfTrigger) override;


    // setTypeFromMat
    void setTypeFromMat(adtf::streaming::cSampleWriter& writer, const cv::Mat& outputImage, tBool keepFormat = false);

    // writeMatToPin
    void writeMatToPin(adtf::streaming::cSampleWriter& writer,
                                           const cv::Mat& outputImage, tTimeStamp streamTime);

    // ChangeType
    tResult ChangeType(adtf::streaming::cDynamicSampleReader& inputPin,
        const adtf::streaming::ant::IStreamType& oType);

    // setRoiOfLanes
    void setRoiOfLanes();

    // transformToBirdsEyeView
    void transformToBirdsEyeView();

    // give me some cookies
    tResult LoadProperties();
}; // IntersectionDetection
