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
#define CID_LANE_DETECTION_FILTER "lane_detection.filter.user.aadc.cid"

// namespaces
using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;

// LaneDetection
class LaneDetection : public cTriggerFunction
{
private:

    // ------- PINS -------------
    // input
    cPinReader m_ReaderVideo;
    cPinReader m_ReaderAction;
    cPinReader m_ReaderLaserScanner;

    // output
    cPinWriter m_WriterDebugVideo;
    cPinWriter m_WriterDebugBirdVideo;
    cPinWriter m_WriterDebugWarpVideo;
    cPinWriter m_WriterDebugMaskVideo;
    cPinWriter m_WriterSteering;
    cPinWriter m_WriterSpeed;
    cPinWriter m_WriterLDLines;
    cPinWriter m_WriterFeedback;


    // ------- VARIABLES --------
    // state: are we in a curve or not
    enum state
    {
        LEFT_CURVE,
        LEFT_WEAK_CURVE,
        STRAIGHT,
        RIGHT_WEAK_CURVE,
        RIGHT_CURVE
    };

    state m_currentState   = STRAIGHT;

    // laneState: are we too far left/right on lane
    enum laneState
    {
        LEFT,
        MIDDLE,
        RIGHT
    };

    laneState m_currentLaneState = MIDDLE;
    laneState m_lastLaneState    = MIDDLE;

    // actioncommands
    TActionStruct m_ActionId;
    TActionStruct::Data m_Action;

    // steering value
    TSignalValue m_SteeringId;

    // speed value
    TSignalValue m_SpeedId;

    // lineStruct
    TLaneDetectionLineStruct m_LDLineId;

    // feedback
    TFeedbackStruct m_FeedbackId;

    // laser scanner
    TLaserScannerData m_LaserScannerId;

    // The clock
    object_ptr<adtf::services::IReferenceClock> m_pClock;

    // image formats for video pins
    adtf::streaming::tStreamImageFormat m_sInputImageFormat;
    adtf::streaming::tStreamImageFormat m_sOutputImageFormat;
    
    // output debug image
    Mat m_MatOutputDebugImage;       // for ROIs
    Mat m_MatOutputDebugWarpedImage; // Image after warp transform

    // input image
    Mat m_MatInputImage;

    // transformed image after bird's eye view transform
    Mat m_MatTransformedImage;
    Mat m_MatTransformedImageOut;
    Mat m_MatPSEImage;

    // mask
    Mat m_MatMask;

    // laserscanner data
    std::vector<TPolarCoordiante::Data> m_vecLaserScanner;

    // points
    Point2f m_p2fROIVertices[4];              // ROI
    Point2f m_p2fTransformedImageVertices[4]; // Transformed Image

    // lines
    std::vector<Vec4i> m_vecLines;
    std::vector<Point2f> m_vecMiddleLinePoints, m_vecRightLinePoints;
    Vec4f m_v4iMiddleLane, m_v4iRightLane;

    // calc angle
    tFloat32 m_f32AvgAngleOnLeftSide;
    tFloat32 m_f32AvgAngleOnRightSide;
    tFloat32 m_f32LastSteeringAngle;
    tFloat32 m_f32NextSteeringAngle;
    tFloat32 m_f32AngleSumLeft      = 0;
    tFloat32 m_f32AngleSumRight     = 0;
    tFloat32 m_f32LastAvgAngle      = 0;

    // speed
    tFloat32 m_f32LastSpeed;
    tFloat32 m_f32NextSpeed;

    // frames
    tUInt32 m_ui32FrameCounter = 0;
    tUInt32 m_ui32LaneFrameCounter = 0;

    // found lanes
    tBool m_bFoundMiddleLane, m_bFoundRightLane;

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

    // ImageProcessing Properties: Gaussian blur, Canny edge detection, Hough transform
    adtf::base::property_variable<tUInt32> m_propGaussianKernelSize;
    adtf::base::property_variable<tUInt32> m_propCannyHighThreshold;
    adtf::base::property_variable<tUInt32> m_propCannyKernelSize;
    adtf::base::property_variable<tUInt32> m_propDilationSize;
    adtf::base::property_variable<tUInt32> m_propDilationPoint;
    adtf::base::property_variable<tUInt32> m_propHoughThreshold;
    adtf::base::property_variable<tUInt32> m_propHoughMinLineLength;
    adtf::base::property_variable<tUInt32> m_propHoughMaxLineGap;
    adtf::base::property_variable<tBool>   m_propEnableGaussianBlur;
    adtf::base::property_variable<tBool>   m_propEnableCannyEdge;
    adtf::base::property_variable<tBool>   m_propEnableDilation;
    adtf::base::property_variable<tBool>   m_propEnableHough;
    adtf::base::property_variable<tUInt32> m_propNoCannyThreshold;

    // LaneFinding
    adtf::base::property_variable<tInt> m_propLaneAnglePointLeftX;
    adtf::base::property_variable<tInt> m_propLaneAnglePointLeftY;
    adtf::base::property_variable<tInt> m_propLaneAnglePointRightX;
    adtf::base::property_variable<tInt> m_propLaneAnglePointRightY;
    adtf::base::property_variable<tInt> m_propLaneMiddlePointY;
    // ILC = in left curve
    adtf::base::property_variable<tInt> m_propLaneMiddlePointILCY;
    // IRC = in right curve
    adtf::base::property_variable<tInt> m_propLaneMiddlePointIRCY;

    adtf::base::property_variable<tInt> m_propLaneDistanceRight;
    adtf::base::property_variable<tInt> m_propLaneDistanceRightILC;
    adtf::base::property_variable<tInt> m_propLaneDistanceLeftIRC;
    adtf::base::property_variable<tInt> m_propLaneDistanceMiddle;

    // Limits
    adtf::base::property_variable<tInt> m_propLaneDistanceLimitRight;
    adtf::base::property_variable<tInt> m_propLaneDistanceLimitLeft;
    adtf::base::property_variable<tInt> m_propLaneDistanceRightFar;
    adtf::base::property_variable<tInt> m_propLaneDistanceLeftFar;

    // Steering: Wheels have a little bit of a offset :(
    adtf::base::property_variable<tFloat32> m_propSteeringOffset;
    adtf::base::property_variable<tFloat32> m_propSteeringWeakRight;
    adtf::base::property_variable<tFloat32> m_propSteeringHeavyRight;
    adtf::base::property_variable<tFloat32> m_propSteeringWeakLeft;
    adtf::base::property_variable<tFloat32> m_propSteeringHeavyLeft;
    adtf::base::property_variable<tInt> m_propSteeringWeakRightLimit;
    adtf::base::property_variable<tInt> m_propSteeringHeavyRightLimit;
    adtf::base::property_variable<tInt> m_propSteeringWeakLeftLimit;
    adtf::base::property_variable<tInt> m_propSteeringHeavyLeftLimit;

    adtf::base::property_variable<tFloat32> m_propSteeringLaneOffsetRight;
    adtf::base::property_variable<tFloat32> m_propSteeringLaneOffsetLeft;
    adtf::base::property_variable<tFloat32> m_propSteeringLaneOffsetRightFar;
    adtf::base::property_variable<tFloat32> m_propSteeringLaneOffsetLeftFar;

    // Speed: speed in curve and straight
    adtf::base::property_variable<tFloat32>  m_propSpeedStraight;
    adtf::base::property_variable<tFloat32>  m_propSpeedWeakCurve;
    adtf::base::property_variable<tFloat32>  m_propSpeedHeavyCurve;

    // Frame
    adtf::base::property_variable<tUInt32> m_propFramesPerAction;

    // Debug
    adtf::base::property_variable<tBool> m_propEnableDebugging;
    adtf::base::property_variable<tBool> m_propEnableDebuggingVideo;

    // Car
    adtf::base::property_variable<tInt> m_propCarPositionInImageX;
    adtf::base::property_variable<tInt> m_propCarPositionInImageY;

    // LaserScanner
    adtf::base::property_variable<tFloat32> m_F32InCarViewXmin; // in mm
    adtf::base::property_variable<tFloat32> m_F32InCarViewXmax;
    adtf::base::property_variable<tFloat32> m_F32InCarViewYmin;
    adtf::base::property_variable<tFloat32> m_F32InCarViewYmax;


public:
    /*------------ FUNCTIONS -----------*/
    /*------------ FROM AUDI -----------*/
    // constructor
    LaneDetection();

    // destructor
    ~LaneDetection() = default;

    // Configure
    virtual tResult Configure() override;

    // Process
    virtual tResult Process(tTimeStamp tmTimeOfTrigger) override;

    // ---------- FROM USER ------------*/
    // setTypeFromMat
    void setTypeFromMat(adtf::streaming::cSampleWriter& writer, const cv::Mat& outputImage, tBool keepFormat = false);

    // writeMatToPin
    void writeMatToPin(adtf::streaming::cSampleWriter& writer,
					   const cv::Mat& outputImage, tTimeStamp streamTime);
    
    // ChangeType
    tResult ChangeType(adtf::streaming::cDynamicSampleReader& inputPin,
        const adtf::streaming::ant::IStreamType& oType);

    // runImageProcessingPipeline
    void runImageProcessingPipeline();

    // setRoiOfLanes
    void setRoiOfLanes();

    // transformToBirdsEyeView
    void transformToBirdsEyeView();

    // detectLanes
    void detectLanes();

    // calculateLineAngle
    tFloat32 calculateLineAngle(Vec4i l);

    // calculateSteeringAngleAndSpeed
    void calculateSteeringAngleAndSpeed();

    // calculateLaneHoldingSteeringAngle
    void calculateLaneHoldingSteeringAngle();

    // addLaneHoldingSteeringAngle
    void addLaneHoldingSteeringAngle(tInt inputDistanceToGoal);

    // transmitAll
    tResult transmitAll(tBool inputStop);

    // ProcessLIDARInput
    tResult ProcessLIDARInput(TLaserScannerData::Data inputLaserData);

    // setMask
    void setMask();

    // Laserscanner functions
    void ObstacleDetectionOtherLane(TLaserScannerData::Data inputLaser);
    tFloat32 AngleCompensation(tFloat32 angle);

    // give me some cookies
    tResult LoadProperties();

}; // LaneDetection
