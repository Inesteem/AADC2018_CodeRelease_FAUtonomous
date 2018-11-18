/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. Team FAUtonomous. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

This filter will detect lanes and propose a steering angle to hold lanes.
1. Set the ROI, where the lanes are located.
2. Transform image to bird's eye view.
2. Detect lanes with Hough Transform.
4. Calculate steering angle

**********************************************************************/

#include "stdafx.h"
#include "LaneDetection.h"
#include <common_helper.h>
#include <property_structs.h>
#include "ScmCommunication.h"
#include <cmath>
#include "tinyxml2.h"

// This will define the filter and expose it via plugin class factory.
// Class cTemplateFilter will be embedded to the filter and called by
// data trigger of "speed" by default.
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(
    CID_LANE_DETECTION_FILTER,		// references to header file
    "LaneDetection",              // label
    LaneDetection,                // class
    adtf::filter::pin_trigger({"inputVideo"}));	// set trigger pin


// -------------------------------
// --------- CONSTRUCTOR ---------
// -------------------------------
// in the constructor, we will set the adtf streamtypes for signal values and pins
LaneDetection::LaneDetection()
{
    // ------------- PINS -------------
    m_ActionId      .registerPin(this, m_ReaderAction      , "inputAction" );
    m_LaserScannerId.registerPin(this, m_ReaderLaserScanner, "inputLaserScanner");

    m_FeedbackId.registerPin(this, m_WriterFeedback, "outputFeedback");
    m_SteeringId.registerPin(this, m_WriterSteering, "outputSteering" );
    m_SpeedId   .registerPin(this, m_WriterSpeed   , "outputSpeed");
    m_LDLineId  .registerPin(this, m_WriterLDLines , "outputLines");

    // --------- VIDEO PINS ----------
    //create and set inital input format type
    m_sInputImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    adtf::ucom::object_ptr<IStreamType> pVideoDataType = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pVideoDataType, m_sInputImageFormat);

    // set output type
    adtf::ucom::object_ptr<IStreamType> pTypeOutput = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pTypeOutput, m_sOutputImageFormat);

    // Register input pin
    Register(m_ReaderVideo, "inputVideo" , pVideoDataType);
    // Register output pin
    Register(m_WriterDebugVideo    , "outputDebugVideo"       , pTypeOutput);
    Register(m_WriterDebugWarpVideo, "outputDebugWarpedVideo" , pTypeOutput);
    Register(m_WriterDebugBirdVideo, "outputDebugBirdVideo"   , pTypeOutput);
    Register(m_WriterDebugMaskVideo, "outputDebugMaskVideo"   , pTypeOutput);

    //register callback for type changes
    m_ReaderVideo.SetAcceptTypeCallback([this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pVideoDataType) -> tResult
    {
        return ChangeType(m_ReaderVideo, *pVideoDataType.Get());
    });


    // ---------- PROPERTIES -----------
    // Debug
    RegisterPropertyVariable("Debug/enableText" , m_propEnableDebugging);
    RegisterPropertyVariable("Debug/enableVideo", m_propEnableDebuggingVideo);

    // ROI
    RegisterPropertyVariable("ROI/Point1/X", m_propROIPoint1X); // upper left
    RegisterPropertyVariable("ROI/Point1/Y", m_propROIPoint1Y);
    RegisterPropertyVariable("ROI/Point2/X", m_propROIPoint2X); // upper right
    RegisterPropertyVariable("ROI/Point2/Y", m_propROIPoint2Y);
    RegisterPropertyVariable("ROI/Point3/X", m_propROIPoint3X); // lower right
    RegisterPropertyVariable("ROI/Point3/Y", m_propROIPoint3Y);
    RegisterPropertyVariable("ROI/Point4/X", m_propROIPoint4X); // lower left
    RegisterPropertyVariable("ROI/Point4/Y", m_propROIPoint4Y);

    // Transformed Image
    RegisterPropertyVariable("Transformed Image/width ", m_propTransformedImageWidth );
    RegisterPropertyVariable("Transformed Image/height", m_propTransformedImageHeight);

    // ImageProcessing
    RegisterPropertyVariable("Imageprocessing/GaussianBlur/enable"    , m_propEnableGaussianBlur);
    RegisterPropertyVariable("Imageprocessing/GaussianBlur/kernelSize", m_propGaussianKernelSize);
    RegisterPropertyVariable("Imageprocessing/Canny/enable"           , m_propEnableCannyEdge);
    RegisterPropertyVariable("Imageprocessing/Canny/highThreshold"    , m_propCannyHighThreshold);
    RegisterPropertyVariable("Imageprocessing/Canny/kernelSize"       , m_propCannyKernelSize);
    RegisterPropertyVariable("Imageprocessing/Hough/enable"           , m_propEnableHough);
    RegisterPropertyVariable("Imageprocessing/Hough/threshold"        , m_propHoughThreshold);
    RegisterPropertyVariable("Imageprocessing/Hough/minLineLength"    , m_propHoughMinLineLength);
    RegisterPropertyVariable("Imageprocessing/Hough/maxLineGap"       , m_propHoughMaxLineGap);
    RegisterPropertyVariable("Imageprocessing/binaryThreshold"        , m_propNoCannyThreshold);

    // LaneFinding
    RegisterPropertyVariable("LaneFinding/AnglePoint1/X", m_propLaneAnglePointLeftX);
    RegisterPropertyVariable("LaneFinding/AnglePoint1/Y", m_propLaneAnglePointLeftY);
    RegisterPropertyVariable("LaneFinding/AnglePoint2/X", m_propLaneAnglePointRightX);
    RegisterPropertyVariable("LaneFinding/AnglePoint2/Y", m_propLaneAnglePointRightY);
    RegisterPropertyVariable("LaneFinding/MiddlePointY" , m_propLaneMiddlePointY);
    RegisterPropertyVariable("LaneFinding/MiddlePointIRCY" , m_propLaneMiddlePointIRCY);
    RegisterPropertyVariable("LaneFinding/MiddlePointILCY" , m_propLaneMiddlePointILCY);

    RegisterPropertyVariable("LaneFinding/DistanceRight" , m_propLaneDistanceRight);
    RegisterPropertyVariable("LaneFinding/DistanceMiddle", m_propLaneDistanceMiddle);
    RegisterPropertyVariable("LaneFinding/Limit/Right"   , m_propLaneDistanceLimitRight);
    RegisterPropertyVariable("LaneFinding/Limit/Left"    , m_propLaneDistanceLimitLeft);
    RegisterPropertyVariable("LaneFinding/Limit/RightFar", m_propLaneDistanceRightFar);
    RegisterPropertyVariable("LaneFinding/Limit/LeftFar" , m_propLaneDistanceLeftFar);

    // Steering
    RegisterPropertyVariable("Steering/offset"    , m_propSteeringOffset);
    RegisterPropertyVariable("Steering/weakRight" , m_propSteeringWeakRight);
    RegisterPropertyVariable("Steering/HeavyRight", m_propSteeringHeavyRight);
    RegisterPropertyVariable("Steering/weakLeft"  , m_propSteeringWeakLeft);
    RegisterPropertyVariable("Steering/heavyLeft" , m_propSteeringHeavyLeft);

    RegisterPropertyVariable("Steering/weakRightLimit" , m_propSteeringWeakRightLimit);
    RegisterPropertyVariable("Steering/HeavyRightLimit", m_propSteeringHeavyRightLimit);
    RegisterPropertyVariable("Steering/weakLeftLimit"  , m_propSteeringWeakLeftLimit);
    RegisterPropertyVariable("Steering/heavyLeftLimit" , m_propSteeringHeavyLeftLimit);

    RegisterPropertyVariable("Steering/LaneOffsetRight"   , m_propSteeringLaneOffsetRight);
    RegisterPropertyVariable("Steering/LaneOffsetLeft"    , m_propSteeringLaneOffsetLeft);
    RegisterPropertyVariable("Steering/LaneOffsetRightFar", m_propSteeringLaneOffsetRightFar);
    RegisterPropertyVariable("Steering/LaneOffsetLeftFar" , m_propSteeringLaneOffsetLeftFar);

    // Speed
    RegisterPropertyVariable("Speed/straight"  , m_propSpeedStraight);
    RegisterPropertyVariable("Speed/weakCurve" , m_propSpeedWeakCurve);
    RegisterPropertyVariable("Speed/heavyCurve", m_propSpeedHeavyCurve);

    // Frames
    RegisterPropertyVariable("Frames/number" , m_propFramesPerAction);

    // Car, just the position in our image, where car will be stored
    RegisterPropertyVariable("CarPosInImage/X" , m_propCarPositionInImageX);
    RegisterPropertyVariable("CarPosInImage/Y" , m_propCarPositionInImageY);

    // LaserScanner
    RegisterPropertyVariable("LaserScanner/XMax" , m_F32InCarViewXmax);
    RegisterPropertyVariable("LaserScanner/YMax" , m_F32InCarViewYmax);
    RegisterPropertyVariable("LaserScanner/XMin" , m_F32InCarViewXmin);
    RegisterPropertyVariable("LaserScanner/YMin" , m_F32InCarViewYmin);

    // ---------- VARIABLES ------------
    // first steering angle will be 'straight'
    m_f32NextSteeringAngle = m_propSteeringOffset;

    // first speed is 0
    m_f32NextSpeed = 0;

    // first actioncommand is a stop
    m_Action.ui8FilterId = 0;
    m_Action.ui32Command = 0;

    // bools
    m_bFoundMiddleLane = false;
    m_bFoundRightLane  = false;
}


// ------------------------------
// --------- CONFIGURE ----------
// ------------------------------
// implement configure() to read ALL properties
tResult LaneDetection::Configure()
{
    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

    LoadProperties();

    RETURN_NOERROR;
}


// ------------------------------
// ---------- PROCESS -----------
// ------------------------------
// process() is always called each time a trigger occured (new data sample arrive).
// IMPORTANT: Make sure to read ALL samples of ALL readers!
// This is how to read a sample
tResult LaneDetection::Process(tTimeStamp)
{
    // action input
    TActionStruct::Data actionIn;
    if(IS_OK(m_ActionId.readPin(m_ReaderAction, (void *) &actionIn)))
    {
        if(actionIn.ui8FilterId == F_LINE_SPECIFIER || actionIn.ui8FilterId == F_LANE_DETECTION)
        {
            m_Action = actionIn;
            if(m_propEnableDebugging)
            {
                LOG_INFO("LaneDetection: action (%d)", actionIn.ui32Command);
            }
        }
    }

    if(m_Action.ui32Command == AC_LS_STOP || m_Action.ui32Command == AC_LD_STOP)
    {
        transmitAll(true);
    }
    if(actionIn.ui32Command == RELOAD_XML_FILES)
    {
        LoadProperties();
    }

    // laserScanner input
    TLaserScannerData::Data laserIn;
    if(IS_OK(m_LaserScannerId.readPin(m_ReaderLaserScanner, (void *) &laserIn)))
    {
        if(m_propEnableDebugging)
        {
            LOG_INFO("LaneDetection: Processing LaserScanner");
        }
        RETURN_IF_FAILED(ProcessLIDARInput(laserIn));
    }

    // video input
    object_ptr<const ISample> pReadVideoSample;
    if (IS_OK(m_ReaderVideo.GetNextSample(pReadVideoSample)))
    {
        object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
        //lock read buffer
        if (IS_OK(pReadVideoSample->Lock(pReadBuffer)))
        {
            m_ui32FrameCounter++;
            // run every x frames
            if(m_ui32FrameCounter == m_propFramesPerAction)
            {
                // create a opencv matrix from the media sample buffer
                Mat inputImage (cv::Size(m_sInputImageFormat.m_ui32Width, m_sInputImageFormat.m_ui32Height),
                                CV_8UC3, (uchar*)pReadBuffer->GetPtr());

                // set our global mat to input image
                m_MatInputImage       = inputImage;
                m_MatOutputDebugImage = m_MatInputImage.clone();

                // start image processing
                runImageProcessingPipeline();

                // calculate steering and speed
                calculateSteeringAngleAndSpeed();

                // calculateLaneHoldingSteeringAngle
                calculateLaneHoldingSteeringAngle();

                // only run if command says so
                if(m_Action.ui32Command == AC_LD_DRIVE || m_Action.ui32Command == AC_LS_SLOW_RIGHTLANE || m_Action.ui32Command == AC_LS_SLOW_SLOW_RIGHTLANE)
                {
                    // send values to speed, steering and line pin
                    transmitAll(false);
                }
                m_ui32FrameCounter = 0;
            }
        }

        // debug mode
        if(m_propEnableDebuggingVideo)
        {
            // output//Write processed Image to Output Pin
            if (!m_MatOutputDebugImage.empty())
            {
                //update output format if matrix size does not fit to
                if (m_MatOutputDebugImage.total() * m_MatOutputDebugImage.elemSize() != m_sOutputImageFormat.m_szMaxByteSize)
                {
                    setTypeFromMat(m_WriterDebugVideo, m_MatOutputDebugImage);
                }
                // write to pin
                writeMatToPin(m_WriterDebugVideo, m_MatOutputDebugImage, m_pClock->GetStreamTime());
            }
            // output//Write processed Image to Output Pin
            if (!m_MatOutputDebugWarpedImage.empty())
            {
                //update output format if matrix size does not fit to
                if (m_MatOutputDebugWarpedImage.total() * m_MatOutputDebugWarpedImage.elemSize() != m_sOutputImageFormat.m_szMaxByteSize)
                {
                    setTypeFromMat(m_WriterDebugWarpVideo, m_MatOutputDebugWarpedImage);
                }

                // write to pin
                writeMatToPin(m_WriterDebugWarpVideo, m_MatOutputDebugWarpedImage, m_pClock->GetStreamTime());
            }
            // output//Write processed Image to Output Pin
            if (!m_MatTransformedImageOut.empty())
            {
                //update output format if matrix size does not fit to
                if (m_MatTransformedImageOut.total() * m_MatTransformedImageOut.elemSize() != m_sOutputImageFormat.m_szMaxByteSize)
                {
                    setTypeFromMat(m_WriterDebugBirdVideo, m_MatTransformedImageOut);
                }
                // write to pin
                writeMatToPin(m_WriterDebugBirdVideo, m_MatTransformedImageOut, m_pClock->GetStreamTime());
            }
            // output//Write processed Image to Output Pin
            if (!m_MatMask.empty())
            {
                //update output format if matrix size does not fit to
                if (m_MatMask.total() * m_MatMask.elemSize() != m_sOutputImageFormat.m_szMaxByteSize)
                {
                    setTypeFromMat(m_WriterDebugMaskVideo, m_MatMask);
                }
                // write to pin
                writeMatToPin(m_WriterDebugMaskVideo, m_MatMask, m_pClock->GetStreamTime());
            }
        }
    }
    if(!m_vecLines.empty())
    {
        m_vecLines.clear();
    }

    // done
    RETURN_NOERROR;
}


// ------------------------------
// ------ OTHER FUNCTIONS -------
// ------------------------------

// ------------- Functions for VIDEO stuff
// setTypeFromMat
// we need this for video outputs
void LaneDetection::setTypeFromMat(adtf::streaming::cSampleWriter& writer, const cv::Mat& outputImage, tBool keepFormat)
{
    adtf::streaming::tStreamImageFormat outputFormat;
    outputFormat.m_ui32Height = outputImage.rows;
    outputFormat.m_ui32Width = outputImage.cols;
    outputFormat.m_szMaxByteSize = outputImage.cols * outputImage.rows * outputImage.channels();

    if (!keepFormat)
    {
        if (outputImage.channels() == 1)
        {
            outputFormat.m_strFormatName = ADTF_IMAGE_FORMAT(GREYSCALE_8);
        }
        if (outputImage.channels() == 3)
        {
            outputFormat.m_strFormatName = ADTF_IMAGE_FORMAT(BGR_24);
        }
    }
    outputFormat.m_ui8DataEndianess = PLATFORM_BYTEORDER;

    adtf::ucom::object_ptr<IStreamType> pTypeOutput = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pTypeOutput, outputFormat);

    writer << pTypeOutput;
}

// writeMatToPin
// this function will output our image
void LaneDetection::writeMatToPin(adtf::streaming::cSampleWriter& writer, const cv::Mat& outputImage, tTimeStamp streamTime)
{
    //create write buffer
    object_ptr<ISample> pWriteSample;

    if (IS_OK(alloc_sample(pWriteSample, streamTime)))
    {
        object_ptr_locked<ISampleBuffer> pWriteBuffer;
        if (IS_OK(pWriteSample->WriteLock(pWriteBuffer, outputImage.cols * outputImage.rows * outputImage.channels())))
        {
            pWriteBuffer->Write(adtf_memory_buffer<void, tSize>((void*) outputImage.data,
                outputImage.cols * outputImage.rows * outputImage.channels()));
        }
    }

    writer << pWriteSample << flush << trigger;
}

// ChangeType
// we need this for video inputs
tResult LaneDetection::ChangeType(adtf::streaming::cDynamicSampleReader& inputPin,
    const adtf::streaming::ant::IStreamType& oType)
{
    if (oType == adtf::streaming::stream_meta_type_image())
    {
        adtf::ucom::object_ptr<const adtf::streaming::IStreamType> pTypeInput;
        // get pType from input reader
        inputPin >> pTypeInput;
        adtf::streaming::get_stream_type_image_format(m_sInputImageFormat, *pTypeInput);

        //set also output format
        adtf::streaming::get_stream_type_image_format(m_sOutputImageFormat, *pTypeInput);
        //we always have a grayscale output image
        m_sOutputImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(BGR_24);
        // and set pType also to samplewriter
        m_WriterDebugVideo     << pTypeInput;
        m_WriterDebugWarpVideo << pTypeInput;
        m_WriterDebugBirdVideo << pTypeInput;
        m_WriterDebugMaskVideo << pTypeInput;
    }
    else
    {
        RETURN_ERROR(ERR_INVALID_TYPE);
    }

    RETURN_NOERROR;
}

// ---------------------------------------

// runImageProcessingPipeline
void LaneDetection::runImageProcessingPipeline()
{
    // 1. set ROI of lanes
    setRoiOfLanes();

    // 2. transform to birds eye view
    transformToBirdsEyeView();

    // 3. detect lanes
    detectLanes();
}

// setRoiOfLanes
// This function will set our ROI, where the lanes are located
// make sure to set right coordinates of ROI in properties! :)
void LaneDetection::setRoiOfLanes()
{
    // set roi points
    m_p2fROIVertices[0] = Point(m_propROIPoint1X, m_propROIPoint1Y); // upper left
    m_p2fROIVertices[1] = Point(m_propROIPoint2X, m_propROIPoint2Y); // upper right
    m_p2fROIVertices[2] = Point(m_propROIPoint3X, m_propROIPoint3Y); // lower right
    m_p2fROIVertices[3] = Point(m_propROIPoint4X, m_propROIPoint4Y); // lower left

    // debug mode
    if(m_propEnableDebuggingVideo)
    {
        // draw it on mat for debugging
        circle(m_MatOutputDebugImage, m_p2fROIVertices[0], 4, Scalar(0, 0, 255));
        circle(m_MatOutputDebugImage, m_p2fROIVertices[1], 4, Scalar(0, 0, 255));
        circle(m_MatOutputDebugImage, m_p2fROIVertices[2], 4, Scalar(0, 0, 255));
        circle(m_MatOutputDebugImage, m_p2fROIVertices[3], 4, Scalar(0, 0, 255));
        line(m_MatOutputDebugImage, m_p2fROIVertices[0], m_p2fROIVertices[1], Scalar(0, 100, 255), 2, CV_AA);
        line(m_MatOutputDebugImage, m_p2fROIVertices[1], m_p2fROIVertices[2], Scalar(0, 100, 255), 2, CV_AA);
        line(m_MatOutputDebugImage, m_p2fROIVertices[2], m_p2fROIVertices[3], Scalar(0, 100, 255), 2, CV_AA);
        line(m_MatOutputDebugImage, m_p2fROIVertices[3], m_p2fROIVertices[0], Scalar(0, 100, 255), 2, CV_AA);
    }
}

// transformToBirdsEyeView
// This function will transform our image to bird's eye view
// transformed image is stored in m_MatTransformedImage
void LaneDetection::transformToBirdsEyeView()
{
    // image width and height
    m_p2fTransformedImageVertices[0] = Point(0, 0);
    m_p2fTransformedImageVertices[1] = Point(m_propTransformedImageWidth, 0);
    m_p2fTransformedImageVertices[2] = Point(m_propTransformedImageWidth, m_propTransformedImageHeight);
    m_p2fTransformedImageVertices[3] = Point(0, m_propTransformedImageHeight);

    Mat dst(cv::Size(m_propTransformedImageWidth, m_propTransformedImageHeight), CV_8UC3);
    // transform
    Mat M = getPerspectiveTransform(m_p2fROIVertices, m_p2fTransformedImageVertices);
    warpPerspective(m_MatInputImage, dst, M, dst.size(), INTER_LINEAR, BORDER_CONSTANT);

    // set to our global mat
    m_MatTransformedImage       = dst.clone();
    m_MatOutputDebugWarpedImage = dst.clone();

    if(m_propEnableDebuggingVideo)
    {
        m_MatTransformedImageOut = dst.clone();
    }
}

// detectLanes
// This function will detect lanes in our transformed image and
// will store the lines in m_vecLines
void LaneDetection::detectLanes()
{
    // 1. create gray image
    cvtColor(m_MatTransformedImage, m_MatTransformedImage, CV_BGR2GRAY);

    // 2. blur to smooth image
    if(m_propEnableGaussianBlur)
    {
        GaussianBlur(m_MatTransformedImage, m_MatTransformedImage,
                     cv::Size(m_propGaussianKernelSize, m_propGaussianKernelSize), 0);
    }

    // 3. detect edges
    if(m_propEnableCannyEdge)
    {
        Canny(m_MatTransformedImage, m_MatTransformedImage, m_propCannyHighThreshold / 3,
              m_propCannyHighThreshold, m_propCannyKernelSize);

        if(m_propEnableDilation)
        {
            // Dilation to get better edges
            Mat element = getStructuringElement(MORPH_RECT,
                                                Size(m_propDilationSize, m_propDilationSize),
                                                Point(m_propDilationPoint, m_propDilationPoint));
            // Apply the dilation operation
            dilate(m_MatTransformedImage, m_MatTransformedImage, element);
        }
    }
    else
    {
        threshold(m_MatTransformedImage, m_MatTransformedImage, m_propNoCannyThreshold, 255, THRESH_BINARY);// Generate Binary Image
    }

    // set Mask
    setMask();

    // 4. detect lines
    if(m_propEnableHough)
    {
        std::vector<Vec4i> vecLines;
        HoughLinesP(m_MatTransformedImage, vecLines, 1, CV_PI/180, m_propHoughThreshold,
                    m_propHoughMinLineLength, m_propHoughMaxLineGap);

        if(m_currentState == STRAIGHT)
        {
            // iterate over all lines
            for(size_t i = 0; i < vecLines.size(); i++)
            {
                Vec4i l = vecLines[i];

                if(l[0] >= m_propLaneAnglePointLeftX || l[2] >= m_propLaneAnglePointLeftX)
                {
                    m_vecLines.push_back(l);
                }

            }
            // after iteration, we now have all middle and right points saved
            // in the middle and right lane vector
        }
        else
        {
            m_vecLines = vecLines;
        }
    }

    cvtColor(m_MatTransformedImage, m_MatTransformedImage, CV_GRAY2BGR);

    m_MatOutputDebugWarpedImage = m_MatTransformedImage.clone();

    if(m_propEnableDebuggingVideo)
    {
  /*      // draw houghlines
        for( size_t i = 0; i < m_vecLines.size(); i++ )
        {
          Vec4i l = m_vecLines[i];
          line(m_MatOutputDebugWarpedImage, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,0,0), 4, CV_AA);
          circle(m_MatOutputDebugWarpedImage, Point(l[0], l[1]), 3, Scalar(123,0,111), CV_AA);
          circle(m_MatOutputDebugWarpedImage, Point(l[2], l[3]), 3, Scalar(214,30,131),CV_AA);
        }
*/
        // draw car as yellow circle
        circle(m_MatOutputDebugWarpedImage, Point(m_propCarPositionInImageX, m_propCarPositionInImageY), 5, Scalar(0, 255, 255), CV_FILLED);
    }
}


// set Mask
void LaneDetection::setMask()
{
    // white mask
    Mat maskImage (cv::Size(m_propTransformedImageWidth, m_propTransformedImageHeight),
                    CV_8UC3, Scalar(255, 255, 255));

    // set our global mat to mask
    m_MatMask = maskImage;

    for(size_t i = 0; i < m_vecLaserScanner.size(); i++)
    {
        float angle  = 270 + m_vecLaserScanner[i].f32Angle;
        float length = m_vecLaserScanner[i].f32Radius / 3;
        Point laserScannerPoint, endPoint;
        laserScannerPoint.x = (int)round(m_propCarPositionInImageX + length * cos(angle * CV_PI / 180.0));
        laserScannerPoint.y = (int)round(m_propCarPositionInImageY + length * sin(angle * CV_PI / 180.0)) + 100;
        endPoint.x          = (int)round(laserScannerPoint.x + m_propTransformedImageHeight * cos(angle * CV_PI / 180.0));
        endPoint.y          = (int)round(laserScannerPoint.y + m_propTransformedImageHeight * sin(angle * CV_PI / 180.0));

        circle(m_MatMask, laserScannerPoint, 5, Scalar(0, 0, 0), CV_FILLED);
        //line(m_MatMask, laserScannerPoint, endPoint, Scalar(0, 0, 0), 15, CV_AA);
        if(laserScannerPoint.x >= m_propCarPositionInImageX)
        {
            rectangle(m_MatMask, Rect(laserScannerPoint.x, 0, m_propTransformedImageWidth - laserScannerPoint.x, laserScannerPoint.y), Scalar(0, 0, 0), CV_FILLED);
        }
        else
        {
            rectangle(m_MatMask, Rect(0, 0, m_propTransformedImageWidth - laserScannerPoint.x, laserScannerPoint.y), Scalar(0, 0, 0), CV_FILLED);
        }

    }

    cvtColor(m_MatMask, m_MatMask, CV_BGR2GRAY);

    bitwise_and(m_MatTransformedImage, m_MatMask, m_MatTransformedImage);

    cvtColor(m_MatMask, m_MatMask, CV_GRAY2BGR);
}

// calculateLineAngle
// This helper function will calculate the angle of a line
// returns angle in [deg] (not rad)
tFloat32 LaneDetection::calculateLineAngle(Vec4i l)
{
    tFloat32 angleRad;
    Point p1, p2;
    p1 = Point(l[0], l[1]);
    p2 = Point(l[2], l[3]);
    //calculate angle in radian
    if(p1.y > p2.y)
    {
        angleRad = atan2(p1.y - p2.y, p1.x - p2.x);
    }
    else
    {
        angleRad = atan2(p2.y - p1.y, p2.x - p1.x);
    }

    // return in deg
    return angleRad * 180/CV_PI;
}

// calculateSteeringAngle
void LaneDetection::calculateSteeringAngleAndSpeed()
{
    tFloat32 avgAngle = 0;

    // first, we want to calc all angles of our houghlines, that we've found
    // we separate them in to regions, lines on the left and lines on the right
    // with that separation, we've got better result in our average values
    // i don't know why tho :D
    tUInt32  counterLeft  = 0;
    tUInt32  counterRight = 0;

    float angle;

    // iterate over all lines
    for(size_t i = 0; i < m_vecLines.size(); i++)
    {
        Vec4i l = m_vecLines[i];

        // left side
        if(l[0] < 450)
        {
            // get angle
            angle = calculateLineAngle(l);
            if(( ((angle < 10 && angle > -10) || (angle < 190 && angle >= 170))))
            {
                //LOG_INFO("stopline");
            }
            else if(((angle < 20 && angle > -20) || (angle < 200 && angle >= 160)) && m_Action.ui32Command == AC_LS_SLOW_SLOW_RIGHTLANE)
            {

            }
            else
            {
            // increase counter
            counterLeft++;

            // add angle to sum
            m_f32AngleSumLeft += angle;
            }
        }
        // right side
        else
        {
            // get angle
            angle = calculateLineAngle(l);

            if(( ((angle < 10 && angle > -10) || (angle < 190 && angle >= 170))))
            {
                //LOG_INFO("stopline");
            }
            else if(((angle < 20 && angle > -20) || (angle < 200 && angle >= 160)) && m_Action.ui32Command == AC_LS_SLOW_SLOW_RIGHTLANE)
            {

            }
            else
            {

                // increase counter
                counterRight++;
                //LOG_INFO("angle %f", angle);

                // add angle to sum
                m_f32AngleSumRight += angle;
            }

        }
    }

    // calulate average angles on both sides
    m_f32AvgAngleOnLeftSide  = m_f32AngleSumLeft  / counterLeft;
    m_f32AvgAngleOnRightSide = m_f32AngleSumRight / counterRight;

    // if both sides are valid
    if(!isnan(m_f32AvgAngleOnLeftSide) && !isnan(m_f32AvgAngleOnRightSide))
    {
        avgAngle = (m_f32AvgAngleOnLeftSide + m_f32AvgAngleOnRightSide) / 2;
    }
    // right side values valid
    else if(isnan(m_f32AvgAngleOnLeftSide))
    {
        avgAngle = m_f32AvgAngleOnRightSide;
    }
    // left side values valid
    else if(isnan(m_f32AvgAngleOnRightSide))
    {
        avgAngle = m_f32AvgAngleOnLeftSide;
    }
    // else set it so last stored angle
    else
    {
        avgAngle = m_f32LastAvgAngle;
    }

    m_f32LastSteeringAngle = m_f32NextSteeringAngle;
    m_f32LastSpeed         = m_f32NextSpeed;
    // now we want to set steering angle in dependancy to our calculated angle.
    // the image we are looking at, is in the 'future', that's why it will
    // be our 'next' steering angle.
    // if 80 <= angle <= 100, then we are driving straight, and everything is fine :)
    if(avgAngle >= m_propSteeringWeakLeftLimit && avgAngle <= m_propSteeringWeakRightLimit)
    {
        m_currentState = STRAIGHT;
        m_f32NextSteeringAngle = m_propSteeringOffset;
        m_f32NextSpeed         = m_propSpeedStraight;
        m_propLaneAnglePointLeftX  = 200;
        m_propLaneAnglePointRightX = 430;
    }
    // weak left curve
    else if(avgAngle < m_propSteeringWeakLeftLimit && avgAngle > m_propSteeringHeavyLeftLimit)
    {
        m_currentState = LEFT_WEAK_CURVE;
        m_f32NextSteeringAngle = m_propSteeringOffset - m_propSteeringWeakLeft;
        m_f32NextSpeed         = m_propSpeedWeakCurve;
        m_propLaneAnglePointLeftX  = 160;
        m_propLaneAnglePointRightX = 400;
    }
    // heavy left curve
    else if(avgAngle <= m_propSteeringHeavyLeftLimit)
    {
        m_currentState = LEFT_CURVE;
        m_f32NextSteeringAngle = m_propSteeringOffset - m_propSteeringHeavyLeft;
        m_f32NextSpeed         = m_propSpeedHeavyCurve;
        m_propLaneAnglePointLeftX  = 130;
        m_propLaneAnglePointRightX = 290;
    }
    // weak right curve
    else if(avgAngle > m_propSteeringWeakRightLimit && avgAngle < m_propSteeringHeavyRightLimit)
    {
        m_currentState = RIGHT_WEAK_CURVE;
        m_f32NextSteeringAngle = m_propSteeringOffset + m_propSteeringWeakRight;
        m_f32NextSpeed         = m_propSpeedWeakCurve;
        m_propLaneAnglePointLeftX  = 130;
        m_propLaneAnglePointRightX = 400;
    }
    // heavy right curve
    else if(avgAngle >= m_propSteeringHeavyRightLimit)
    {
        m_currentState = RIGHT_CURVE;
        m_f32NextSteeringAngle = m_propSteeringOffset + m_propSteeringHeavyRight;
        m_f32NextSpeed         = m_propSpeedHeavyCurve;
        m_propLaneAnglePointLeftX  = 250;
        m_propLaneAnglePointRightX = 500;
    }

    // reset values for next run
    m_f32AngleSumLeft  = 0;
    m_f32AngleSumRight = 0;
    counterLeft  = 0;
    counterRight = 0;
    m_f32LastAvgAngle = avgAngle;
}

// calculateLaneHoldingSteeringAngle
// This function will estimate lanes by fitting a line over points.
// Fitted lines will be saved in global variables.
// Then, we will calculate additional steering angle for
// staying on the lane.
void LaneDetection::calculateLaneHoldingSteeringAngle()
{

    // if we are driving straight, we have to stay between middle and right lane
    if(m_currentState == STRAIGHT)
    {
        // iterate over all lines
        for(size_t i = 0; i < m_vecLines.size(); i++)
        {
            Vec4i l = m_vecLines[i];

            if(l[1] > m_propLaneAnglePointLeftY)
            {
                // if point is middle lane
                if(l[0] > m_propLaneAnglePointLeftX && l[0] < m_propLaneAnglePointRightX)
                {
                    m_vecMiddleLinePoints.push_back(Point(l[0], l[1]));
                    circle(m_MatOutputDebugWarpedImage, Point(l[0], l[1]), 2, Scalar(255, 0, 125), CV_AA);
                }
                // if point is right lane
                else if(l[0] > m_propLaneAnglePointRightX)
                {
                    m_vecRightLinePoints.push_back(Point(l[0], l[1]));
                    circle(m_MatOutputDebugWarpedImage, Point(l[0], l[1]), 2, Scalar(155, 20, 125), CV_AA);
                }
            }

            if(l[3] > m_propLaneAnglePointLeftY)
            {
                // if point is middle lane
                if(l[2] > m_propLaneAnglePointLeftX && l[2] < m_propLaneAnglePointRightX)
                {
                    m_vecMiddleLinePoints.push_back(Point(l[2], l[3]));
                    circle(m_MatOutputDebugWarpedImage, Point(l[0], l[1]), 2, Scalar(255, 0, 125), CV_AA);
                }
                // if point is right lane
                else if(l[2] > m_propLaneAnglePointRightX)
                {
                    m_vecRightLinePoints.push_back(Point(l[2], l[3]));
                    circle(m_MatOutputDebugWarpedImage, Point(l[0], l[1]), 2, Scalar(155, 20, 125), CV_AA);
                }
            }

        }
        // after iteration, we now have all middle and right points saved
        // in the middle and right lane vector
    }

    // in a left curve we only have to look for distance to right lane
    else if(m_currentState == LEFT_CURVE || m_currentState == LEFT_WEAK_CURVE)
    {
        // iterate over all lines
        for(size_t i = 0; i < m_vecLines.size(); i++)
        {
            Vec4i l = m_vecLines[i];

            if(l[1] > m_propLaneAnglePointLeftY)
            {
                // if point is right lane
                if(l[0] > m_propLaneAnglePointRightX)
                {
                    m_vecRightLinePoints.push_back(Point(l[0], l[1]));
                }
            }
            if(l[3] > m_propLaneAnglePointLeftY)
            {
                // if point is right lane
                if(l[2] >  m_propLaneAnglePointRightX)
                {
                    m_vecRightLinePoints.push_back(Point(l[2], l[3]));
                }
            }
        }
    }
    // in a right curve we look at middle lane
    else if(m_currentState == RIGHT_CURVE || m_currentState == RIGHT_WEAK_CURVE)
    {
        // iterate over all lines
        for(size_t i = 0; i < m_vecLines.size(); i++)
        {
            Vec4i l = m_vecLines[i];

            if(l[1] > m_propLaneAnglePointLeftY)
            {
                // if point is middle lane
                if(l[0] > m_propLaneAnglePointLeftX && l[0] < m_propLaneAnglePointRightX)
                {
                    m_vecMiddleLinePoints.push_back(Point(l[0], l[1]));
                }
            }
            if(l[3] > m_propLaneAnglePointLeftY)
            {
                // if point is middle lane
                if(l[2] > m_propLaneAnglePointLeftX && l[2] < m_propLaneAnglePointRightX)
                {
                    m_vecMiddleLinePoints.push_back(Point(l[2], l[3]));
                }
            }
        }
    }

    // draw lane area border
    // this is the area where we look for each lanes
    if(m_propEnableDebuggingVideo)
    {
        // vertical lines
        line(m_MatOutputDebugWarpedImage, Point(m_propLaneAnglePointLeftX, m_propLaneAnglePointLeftY),
             Point(m_propLaneAnglePointLeftX, m_propTransformedImageHeight), Scalar(0, 0, 255), 2, CV_AA);
        line(m_MatOutputDebugWarpedImage, Point(m_propLaneAnglePointRightX, m_propLaneAnglePointRightY),
             Point(m_propLaneAnglePointRightX, m_propTransformedImageHeight), Scalar(0, 0, 255), 2, CV_AA);
    }

    // calc fitted line for each lane from vector of points
    Vec4f middleLane, rightLane;
    m_bFoundMiddleLane = false;
    m_bFoundRightLane  = false;

    if(!m_vecMiddleLinePoints.empty())
    {
        fitLine(m_vecMiddleLinePoints, middleLane, CV_DIST_L2, 0, 0.01, 0.01);

        m_v4iMiddleLane[0] = middleLane[2] - 500 * middleLane[0];
        m_v4iMiddleLane[1] = middleLane[3] - 500 * middleLane[1];
        m_v4iMiddleLane[2] = middleLane[2] + 500 * middleLane[0];
        m_v4iMiddleLane[3] = middleLane[3] + 500 * middleLane[1];

        // set flag
        m_bFoundMiddleLane = true;
    }

    if(!m_vecRightLinePoints.empty())
    {
        fitLine(m_vecRightLinePoints, rightLane, CV_DIST_L2, 0, 0.01, 0.01);

        m_v4iRightLane[0] = rightLane[2] - 500 * rightLane[0];
        m_v4iRightLane[1] = rightLane[3] - 500 * rightLane[1];
        m_v4iRightLane[2] = rightLane[2] + 500 * rightLane[0];
        m_v4iRightLane[3] = rightLane[3] + 500 * rightLane[1];

        // set flag
        m_bFoundRightLane = true;
    }

    // draw lanes
    if(m_propEnableDebuggingVideo)
    {
        if(m_bFoundMiddleLane)
        {
            line(m_MatOutputDebugWarpedImage, Point(m_v4iMiddleLane[0], m_v4iMiddleLane[1]),
                    Point(m_v4iMiddleLane[2], m_v4iMiddleLane[3]), Scalar(0, 255, 0), 4, CV_AA);
        }

        if(m_bFoundRightLane)
        {
            line(m_MatOutputDebugWarpedImage, Point(m_v4iRightLane[0], m_v4iRightLane[1]),
                    Point(m_v4iRightLane[2], m_v4iRightLane[3]), Scalar(0, 255, 0), 4, CV_AA);
        }

    }

    // we need a mat with lanes for line iterator
    Mat laneImage(cv::Size(m_propTransformedImageWidth, m_propTransformedImageHeight),
                  CV_8UC3, Scalar(0, 0, 0));
    line(laneImage, Point(m_v4iMiddleLane[0], m_v4iMiddleLane[1]),
            Point(m_v4iMiddleLane[2], m_v4iMiddleLane[3]), Scalar(0, 255, 0), 1, CV_AA);
    line(laneImage, Point(m_v4iRightLane[0], m_v4iRightLane[1]),
            Point(m_v4iRightLane[2], m_v4iRightLane[3]), Scalar(0, 255, 0), 1, CV_AA);

    // calculate distance from our goal position
    if(m_currentState == STRAIGHT)
    {
        Point middleLaneMiddlePoint, rightLaneMiddlePoint;
        tInt dstPointX, distanceToGoal;

        // look at right lane
        if(m_bFoundRightLane)
        {
            LineIterator itRight(laneImage, Point(m_v4iRightLane[0], m_v4iRightLane[1]),
                    Point(m_v4iRightLane[2], m_v4iRightLane[3]), 8);
            for(int i = 0; i < itRight.count; i++, ++itRight)
            {
                Point currentPixel = itRight.pos();

                if(currentPixel.y == m_propLaneMiddlePointY)
                {
                    rightLaneMiddlePoint = currentPixel;
                    break;
                }
            }
        }
        // look at middle lane
        if(m_bFoundMiddleLane)
        {
            LineIterator itMiddle(laneImage, Point(m_v4iMiddleLane[0], m_v4iMiddleLane[1]),
                    Point(m_v4iMiddleLane[2], m_v4iMiddleLane[3]), 8);
            for(int i = 0; i < itMiddle.count; i++, ++itMiddle)
            {
                Point currentPixel = itMiddle.pos();

                if(currentPixel.y == m_propLaneMiddlePointY)
                {
                    middleLaneMiddlePoint = currentPixel;
                    break;
                }
            }
        }

        // NOTE: all coordinates are still image coordinates
        // set dest point: where we want to be with the car,
        //                 which is middle of both lanes
        if(m_bFoundMiddleLane && m_bFoundRightLane)
        {
            dstPointX = (rightLaneMiddlePoint.x + middleLaneMiddlePoint.x) / 2;
        }
        // if we find only one lane, then set predefined dstpoint
        else if(m_bFoundMiddleLane && !m_bFoundRightLane)
        {
            dstPointX = middleLaneMiddlePoint.x + m_propLaneDistanceMiddle;
        }
        else if(!m_bFoundMiddleLane && m_bFoundRightLane)
        {
            dstPointX = rightLaneMiddlePoint.x + m_propLaneDistanceRight;
        }
        else
        {
            return;
        }

        // calculate distance to where we actually are and where we
        // want to be
        distanceToGoal = dstPointX - m_propCarPositionInImageX;

        // draw mona lisa
        if(m_propEnableDebuggingVideo)
        {
            line(m_MatOutputDebugWarpedImage, Point(dstPointX, m_propLaneMiddlePointY - 10),
                 Point(dstPointX, m_propLaneMiddlePointY + 10), Scalar(255, 0, 0), 2, CV_AA);
            line(m_MatOutputDebugWarpedImage, Point(dstPointX, m_propLaneMiddlePointY),
                 Point(m_propCarPositionInImageX, m_propLaneMiddlePointY), Scalar(255, 100, 0), 2, CV_AA);
            line(m_MatOutputDebugWarpedImage, Point(m_propCarPositionInImageX, m_propTransformedImageHeight),
                 Point(m_propCarPositionInImageX, m_propLaneMiddlePointY), Scalar(255, 100, 0), 2, CV_AA);
        }

        addLaneHoldingSteeringAngle(distanceToGoal);
    }
    // in a left curve
    else if(m_currentState == LEFT_WEAK_CURVE || m_currentState == LEFT_CURVE)
    {
        Point rightLaneMiddlePoint;
        tInt dstPointX, distanceToGoal;

        // look at right lane
        if(m_bFoundRightLane)
        {
            LineIterator itRight(laneImage, Point(m_v4iRightLane[0], m_v4iRightLane[1]),
                    Point(m_v4iRightLane[2], m_v4iRightLane[3]), 8);
            for(int i = 0; i < itRight.count; i++, ++itRight)
            {
                Point currentPixel = itRight.pos();

                if(currentPixel.y == m_propLaneMiddlePointILCY)
                {
                    rightLaneMiddlePoint = currentPixel;
                    break;
                }
            }

            dstPointX = rightLaneMiddlePoint.x += m_propLaneDistanceRightILC;
        }
        else
        {
            return;
        }

        // calculate distance to where we actually are and where we
        // want to be
        distanceToGoal = dstPointX - m_propCarPositionInImageX;

        // draw mona lisa
        if(m_propEnableDebuggingVideo)
        {
            line(m_MatOutputDebugWarpedImage, Point(dstPointX, m_propLaneMiddlePointILCY - 10),
                 Point(dstPointX, m_propLaneMiddlePointILCY + 10), Scalar(255, 0, 0), 2, CV_AA);
            line(m_MatOutputDebugWarpedImage, Point(dstPointX, m_propLaneMiddlePointILCY),
                 Point(m_propCarPositionInImageX, m_propLaneMiddlePointILCY), Scalar(255, 100, 0), 2, CV_AA);
            line(m_MatOutputDebugWarpedImage, Point(m_propCarPositionInImageX, m_propTransformedImageHeight),
                 Point(m_propCarPositionInImageX, m_propLaneMiddlePointILCY), Scalar(255, 100, 0), 2, CV_AA);
        }

        addLaneHoldingSteeringAngle(distanceToGoal);
    }
    // in a right curve
    else if(m_currentState == RIGHT_WEAK_CURVE || m_currentState == RIGHT_CURVE)
    {
        Point middleLaneMiddlePoint;
        tInt dstPointX, distanceToGoal;

        // look at middle lane
        if(m_bFoundMiddleLane)
        {
            LineIterator itMiddle(laneImage, Point(m_v4iMiddleLane[0], m_v4iMiddleLane[1]),
                    Point(m_v4iMiddleLane[2], m_v4iMiddleLane[3]), 8);
            for(int i = 0; i < itMiddle.count; i++, ++itMiddle)
            {
                Point currentPixel = itMiddle.pos();

                if(currentPixel.y == m_propLaneMiddlePointIRCY)
                {
                    middleLaneMiddlePoint = currentPixel;
                    break;
                }
            }

            dstPointX = middleLaneMiddlePoint.x += m_propLaneDistanceLeftIRC;
        }
        else
        {
            return;
        }


        // calculate distance to where we actually are and where we
        // want to be
        distanceToGoal = dstPointX - m_propCarPositionInImageX;

        // draw mona lisa
        if(m_propEnableDebuggingVideo)
        {
            line(m_MatOutputDebugWarpedImage, Point(dstPointX, m_propLaneMiddlePointIRCY - 10),
                 Point(dstPointX, m_propLaneMiddlePointIRCY + 10), Scalar(255, 0, 0), 2, CV_AA);
            line(m_MatOutputDebugWarpedImage, Point(dstPointX, m_propLaneMiddlePointIRCY),
                 Point(m_propCarPositionInImageX, m_propLaneMiddlePointIRCY), Scalar(255, 100, 0), 2, CV_AA);
            line(m_MatOutputDebugWarpedImage, Point(m_propCarPositionInImageX, m_propTransformedImageHeight),
                 Point(m_propCarPositionInImageX, m_propLaneMiddlePointIRCY), Scalar(255, 100, 0), 2, CV_AA);
        }

        addLaneHoldingSteeringAngle(distanceToGoal);
    }
    m_ui32LaneFrameCounter++;
    if(m_ui32LaneFrameCounter == m_propFramesPerAction * 2)
    {
        m_vecMiddleLinePoints.clear();
        m_vecRightLinePoints.clear();
        m_ui32LaneFrameCounter = 0;
    }
}


// addLaneHoldingSteeringAngle
void LaneDetection::addLaneHoldingSteeringAngle(tInt inputDistanceToGoal)
{
    // we are too far on right of lane
    if(inputDistanceToGoal <= m_propLaneDistanceLimitRight && inputDistanceToGoal > m_propLaneDistanceRightFar)
    {
        m_f32LastSteeringAngle += m_propSteeringLaneOffsetRight;
        m_currentLaneState = RIGHT;
    }
    // toooooo far on right
    else if(inputDistanceToGoal <= m_propLaneDistanceRightFar)
    {
        m_f32LastSteeringAngle += m_propSteeringLaneOffsetRightFar;
        m_currentLaneState = RIGHT;
    }
    // we are too far on left of lane
    else if(inputDistanceToGoal >= m_propLaneDistanceLimitLeft && inputDistanceToGoal < -m_propLaneDistanceLeftFar)
    {
        m_f32LastSteeringAngle += m_propSteeringLaneOffsetLeft;
        m_currentLaneState = LEFT;
    }
    // toooooo far on left
    else if(inputDistanceToGoal > m_propLaneDistanceLeftFar)
    {
        m_f32LastSteeringAngle += m_propSteeringLaneOffsetLeftFar;
        m_currentLaneState = LEFT;
    }
    // we are in middle of lane
    else
    {
        // if we have corrected our steering angle,
        // we have to steer back to drive straight
        if(m_lastLaneState == RIGHT)
        {
            //m_f32LastSteeringAngle += m_propSteeringLaneOffsetRight;
        }
        if(m_lastLaneState == LEFT)
        {
            //m_f32LastSteeringAngle -= m_propSteeringLaneOffsetLeft;
        }
        m_currentLaneState = MIDDLE;
    }

    // save last state
    m_lastLaneState = m_currentLaneState;
}

// transmitAll
tResult LaneDetection::transmitAll(tBool inputStop)
{
    // output speed SignalValue
    TSignalValue::Data speedSignalValue;
    if(inputStop)
    {
        speedSignalValue.f32Value = 0;
    }
    else
    {

        if(m_Action.ui32Command == AC_LS_SLOW_SLOW_RIGHTLANE)
        {
                speedSignalValue.f32Value = 11;
        }
        else
        {
            speedSignalValue.f32Value = m_f32LastSpeed;
        }
    }
    if(m_propEnableDebugging)
    {
        LOG_INFO("LaneDetection: speed %f", speedSignalValue.f32Value);
    }
    RETURN_IF_FAILED(m_SpeedId.writePin(m_WriterSpeed, (void *) &speedSignalValue,
                                           m_pClock->GetStreamTime()));

    // output steering SignalValue
    TSignalValue::Data steeringSignalValue;
    if(inputStop)
    {
        steeringSignalValue.f32Value = m_propSteeringOffset;
    }
    else
    {
        steeringSignalValue.f32Value = m_f32LastSteeringAngle;
    }
    if(m_propEnableDebugging)
    {
        LOG_INFO("LaneDetection: steering %f", steeringSignalValue.f32Value);
    }

    RETURN_IF_FAILED(m_SteeringId.writePin(m_WriterSteering, (void *) &steeringSignalValue,
                                           m_pClock->GetStreamTime()));

    // output line struct
    TLaneDetectionLineStruct::Data lineStruct;
    lineStruct.i8IsCurve  = 0;
    lineStruct.i32Point1X = 0;
    lineStruct.i32Point1Y = 0;
    lineStruct.i32Point2X = 0;
    lineStruct.i32Point2Y = 0;
    lineStruct.i32Point3X = 0;
    lineStruct.i32Point3Y = 0;
    lineStruct.i32Point4X = 0;
    lineStruct.i32Point4Y = 0;
    if(!inputStop)
    {
        if(m_currentState == STRAIGHT)
        {
            if(m_bFoundMiddleLane)
            {
                lineStruct.i32Point1X =  m_v4iMiddleLane[0] - m_propCarPositionInImageX;
                lineStruct.i32Point1Y = -m_v4iMiddleLane[1] + m_propCarPositionInImageY;
                lineStruct.i32Point2X =  m_v4iMiddleLane[2] - m_propCarPositionInImageX;
                lineStruct.i32Point2Y = -m_v4iMiddleLane[3] + m_propCarPositionInImageY;
            }

            if(m_bFoundRightLane)
            {
                lineStruct.i32Point3X =  m_v4iRightLane[0] - m_propCarPositionInImageX;
                lineStruct.i32Point3Y = -m_v4iRightLane[1] + m_propCarPositionInImageY;
                lineStruct.i32Point4X =  m_v4iRightLane[2] - m_propCarPositionInImageX;
                lineStruct.i32Point4Y = -m_v4iRightLane[3] + m_propCarPositionInImageY;
            }

        }
        else if(m_currentState == LEFT_WEAK_CURVE || m_currentState == LEFT_CURVE)
        {
            lineStruct.i8IsCurve  = -1;
            if(m_bFoundRightLane)
            {
                lineStruct.i32Point1X =  m_v4iRightLane[0] - m_propCarPositionInImageX;
                lineStruct.i32Point1Y = -m_v4iRightLane[1] + m_propCarPositionInImageY;
                lineStruct.i32Point2X =  m_v4iRightLane[2] - m_propCarPositionInImageX;
                lineStruct.i32Point2Y = -m_v4iRightLane[3] + m_propCarPositionInImageY;
            }
        }
        else if(m_currentState == RIGHT_WEAK_CURVE || m_currentState == RIGHT_CURVE)
        {
            lineStruct.i8IsCurve  = 1;
            if(m_bFoundMiddleLane)
            {
                lineStruct.i32Point1X =  m_v4iMiddleLane[0] - m_propCarPositionInImageX;
                lineStruct.i32Point1Y = -m_v4iMiddleLane[1] + m_propCarPositionInImageY;
                lineStruct.i32Point2X =  m_v4iMiddleLane[2] - m_propCarPositionInImageX;
                lineStruct.i32Point2Y = -m_v4iMiddleLane[3] + m_propCarPositionInImageY;
            }
        }
    }
    if(m_propEnableDebugging)
    {
            LOG_INFO(cString::Format("LaneDetection: lines [%d] (%d/%d)(%d/%d)(%d/%d)(%d/%d)",
                                     lineStruct.i8IsCurve,
                                     lineStruct.i32Point1X,
                                     lineStruct.i32Point1Y,
                                     lineStruct.i32Point2X,
                                     lineStruct.i32Point2Y,
                                     lineStruct.i32Point3X,
                                     lineStruct.i32Point3Y,
                                     lineStruct.i32Point4X,
                                     lineStruct.i32Point4Y));
    }

    RETURN_IF_FAILED(m_LDLineId.writePin(m_WriterLDLines, (void *) &lineStruct,
                                           m_pClock->GetStreamTime()));

    // feedback
    TFeedbackStruct::Data feedback;
    feedback.ui8FilterId = F_LINE_SPECIFIER;
    if(inputStop)
    {
        feedback.ui32FeedbackStatus = FB_LS_STOPPED;
    }
    else
    {
        feedback.ui32FeedbackStatus = FB_LS_SLOW_RIGHTLANE;
    }
    if(m_propEnableDebugging)
    {
        LOG_INFO("LaneDetection: feedback %d", feedback.ui32FeedbackStatus);
    }
    RETURN_IF_FAILED(m_FeedbackId.writePin(m_WriterFeedback, (void *) &feedback,
                                           m_pClock->GetStreamTime()));

    // done
    RETURN_NOERROR;
}

// ProcessLIDARInput
tResult LaneDetection::ProcessLIDARInput(TLaserScannerData::Data inputLaserData)
{
    ObstacleDetectionOtherLane(inputLaserData);

    RETURN_NOERROR;
}

void LaneDetection::ObstacleDetectionOtherLane(TLaserScannerData::Data inputLaser)
{
    TLaserScannerData::Data tmp_laser = inputLaser;
    std::vector<TPolarCoordiante::Data> vecRelevantObstacles;
    for (uint32_t i=0; (i<tmp_laser.ui32Size); i++)
    {
        tmp_laser.tScanArray[i].f32Angle = AngleCompensation(tmp_laser.tScanArray[i].f32Angle);
    }
    tFloat32 F32maxDistance = sqrt(std::pow(m_F32InCarViewXmax, 2)+std::pow(m_F32InCarViewYmax, 2));
    for (uint32_t i = 0; (i<tmp_laser.ui32Size);i++)
    {
        if ((tmp_laser.tScanArray[i].f32Radius != 0.0f) && (tmp_laser.tScanArray[i].f32Radius < F32maxDistance))
        {
            tFloat32 f32rcos = tmp_laser.tScanArray[i].f32Radius * cosf((90-tmp_laser.tScanArray[i].f32Angle) * tFloat32(CV_PI)/180.0f);
            tFloat32 f32rsin = tmp_laser.tScanArray[i].f32Radius * sinf((90-tmp_laser.tScanArray[i].f32Angle) * tFloat32(CV_PI)/180.0f);
            if((m_F32InCarViewXmin <= f32rcos) && (f32rcos <= m_F32InCarViewXmax) && (m_F32InCarViewYmin <= f32rsin) && (f32rsin <= m_F32InCarViewYmax))
            {
                vecRelevantObstacles.push_back(tmp_laser.tScanArray[i]);
            }
        }
    }

    m_vecLaserScanner = vecRelevantObstacles;
}


//to compensate the inconsistante angle range-> output car right 90° to car left -90°
tFloat32 LaneDetection::AngleCompensation(tFloat32 angle)
{
    if (angle >= 0.0 && angle <= 90)
    {
        return angle;
    }
    else if(angle >= 270 && angle <= 360)
    {
        return (angle -360);
    }
    LOG_ERROR("Angle out of range!");
    return 100000.0f;//Error value but without error handeling!
}

//this one is for the fans
tResult LaneDetection::LoadProperties()
{
    lanedetection_properties properties;
    LoadXMLData("/home/aadc/AADC/utilities/propertyXMLs/ld_properties.xml", (void*) (&properties));

    m_propROIPoint1X = properties.m_propROIPoint1X;
    m_propROIPoint1Y = properties.m_propROIPoint1Y;
    m_propROIPoint2X = properties.m_propROIPoint2X;
    m_propROIPoint2Y = properties.m_propROIPoint2Y;
    m_propROIPoint3X = properties.m_propROIPoint3X;
    m_propROIPoint3Y = properties.m_propROIPoint3Y;
    m_propROIPoint4X = properties.m_propROIPoint4X;
    m_propROIPoint4Y = properties.m_propROIPoint4Y;

    m_propTransformedImageWidth  = properties.m_propTransformedImageWidth;
    m_propTransformedImageHeight = properties.m_propTransformedImageHeight;
    m_propGaussianKernelSize     = properties.m_propGaussianKernelSize;
    m_propCannyHighThreshold     = properties.m_propCannyHighThreshold;
    m_propCannyKernelSize        = properties.m_propCannyKernelSize;
    m_propDilationSize           = properties.m_propDilationSize;
    m_propDilationPoint          = properties.m_propDilationPoint;
    m_propHoughThreshold         = properties.m_propHoughThreshold;
    m_propHoughMinLineLength     = properties.m_propHoughMinLineLength;
    m_propHoughMaxLineGap        = properties.m_propHoughMaxLineGap;
    m_propNoCannyThreshold       = properties.m_propNoCannyThreshold;
    m_propFramesPerAction        = properties.m_propFramesPerAction;

    m_propLaneAnglePointLeftX  = properties.m_propLaneAnglePointLeftX;
    m_propLaneAnglePointLeftY  = properties.m_propLaneAnglePointLeftY;
    m_propLaneAnglePointRightX = properties.m_propLaneAnglePointRightX;
    m_propLaneAnglePointRightY = properties.m_propLaneAnglePointRightY;

    m_propLaneMiddlePointY    = properties.m_propLaneMiddlePointY;
    m_propLaneMiddlePointILCY = properties.m_propLaneMiddlePointILCY;
    m_propLaneMiddlePointIRCY = properties.m_propLaneMiddlePointIRCY;

    m_propLaneDistanceRight      = properties.m_propLaneDistanceRight;
    m_propLaneDistanceRightILC   = properties.m_propLaneDistanceRightILC;
    m_propLaneDistanceLeftIRC    = properties.m_propLaneDistanceLeftIRC;
    m_propLaneDistanceMiddle     = properties.m_propLaneDistanceMiddle;
    m_propLaneDistanceLimitRight = properties.m_propLaneDistanceLimitRight;
    m_propLaneDistanceLimitLeft  = properties.m_propLaneDistanceLimitLeft;
    m_propLaneDistanceRightFar   = properties.m_propLaneDistanceRightFar;
    m_propLaneDistanceLeftFar    = properties.m_propLaneDistanceLeftFar;

    m_propSteeringWeakRightLimit  = properties.m_propSteeringWeakRightLimit;
    m_propSteeringHeavyRightLimit = properties.m_propSteeringHeavyRightLimit;
    m_propSteeringWeakLeftLimit   = properties.m_propSteeringWeakLeftLimit;
    m_propSteeringHeavyLeftLimit  = properties.m_propSteeringHeavyLeftLimit;

    m_propCarPositionInImageX = properties.m_propCarPositionInImageX;
    m_propCarPositionInImageY = properties.m_propCarPositionInImageY;

    m_propSteeringOffset     = properties.m_propSteeringOffset;
    m_propSteeringWeakRight  = properties.m_propSteeringWeakRight;
    m_propSteeringHeavyRight = properties.m_propSteeringHeavyRight;
    m_propSteeringWeakLeft   = properties.m_propSteeringWeakLeft;
    m_propSteeringHeavyLeft  = properties.m_propSteeringHeavyLeft;

    m_propSteeringLaneOffsetRight    = properties.m_propSteeringLaneOffsetRight;
    m_propSteeringLaneOffsetLeft     = properties.m_propSteeringLaneOffsetLeft;
    m_propSteeringLaneOffsetRightFar = properties.m_propSteeringLaneOffsetRightFar;
    m_propSteeringLaneOffsetLeftFar  = properties.m_propSteeringLaneOffsetLeftFar;

    m_propSpeedStraight   = properties.m_propSpeedStraight;
    m_propSpeedWeakCurve  = properties.m_propSpeedWeakCurve;
    m_propSpeedHeavyCurve = properties.m_propSpeedHeavyCurve;

    m_F32InCarViewXmin = properties.m_F32InCarViewXmin;
    m_F32InCarViewXmax = properties.m_F32InCarViewXmax;
    m_F32InCarViewYmin = properties.m_F32InCarViewYmin;
    m_F32InCarViewYmax = properties.m_F32InCarViewYmax;

    m_propEnableGaussianBlur   = properties.m_propEnableGaussianBlur;
    m_propEnableCannyEdge      = properties.m_propEnableCannyEdge;
    m_propEnableDilation       = properties.m_propEnableDilation;
    m_propEnableHough          = properties.m_propEnableHough;
    m_propEnableDebugging      = properties.m_propEnableDebugging;
    m_propEnableDebuggingVideo = properties.m_propEnableDebuggingVideo;


//    LOG_SUCCESS(cString::Format("m_propROIPoint1X: %d", (unsigned int)m_propROIPoint1X));
//    LOG_SUCCESS(cString::Format("m_propROIPoint1Y: %d", (unsigned int)m_propROIPoint1Y));
//    LOG_SUCCESS(cString::Format("m_propROIPoint2X: %d", (unsigned int)m_propROIPoint2X));
//    LOG_SUCCESS(cString::Format("m_propROIPoint2Y: %d", (unsigned int)m_propROIPoint2Y));
//    LOG_SUCCESS(cString::Format("m_propROIPoint3X: %d", (unsigned int)m_propROIPoint3X));
//    LOG_SUCCESS(cString::Format("m_propROIPoint3Y: %d", (unsigned int)m_propROIPoint3Y));
//    LOG_SUCCESS(cString::Format("m_propROIPoint4X: %d", (unsigned int)m_propROIPoint4X));
//    LOG_SUCCESS(cString::Format("m_propROIPoint4Y: %d", (unsigned int)m_propROIPoint4Y));
//    LOG_SUCCESS(cString::Format("m_propTransformedImageWidth: %d", (unsigned int)m_propTransformedImageWidth));
//    LOG_SUCCESS(cString::Format("m_propTransformedImageHeight: %d", (unsigned int)m_propTransformedImageHeight));
//    LOG_SUCCESS(cString::Format("m_propGaussianKernelSize: %d", (unsigned int)m_propGaussianKernelSize));
//    LOG_SUCCESS(cString::Format("m_propCannyHighThreshold: %d", (unsigned int)m_propCannyHighThreshold));
//    LOG_SUCCESS(cString::Format("m_propCannyKernelSize: %d", (unsigned int)m_propCannyKernelSize));
//    LOG_SUCCESS(cString::Format("m_propDilationSize: %d", (unsigned int)m_propDilationSize));
//    LOG_SUCCESS(cString::Format("m_propDilationPoint: %d", (unsigned int)m_propDilationPoint));
//    LOG_SUCCESS(cString::Format("m_propHoughThreshold: %d", (unsigned int)m_propHoughThreshold));
//    LOG_SUCCESS(cString::Format("m_propHoughMinLineLength: %d", (unsigned int)m_propHoughMinLineLength));
//    LOG_SUCCESS(cString::Format("m_propHoughMaxLineGap: %d", (unsigned int)m_propHoughMaxLineGap));
//    LOG_SUCCESS(cString::Format("m_propNoCannyThreshold: %d", (unsigned int)m_propNoCannyThreshold));
//    LOG_SUCCESS(cString::Format("m_propFramesPerAction: %d", (unsigned int)m_propFramesPerAction));

//    LOG_SUCCESS(cString::Format("m_propLaneAnglePointLeftX: %d", (int)m_propLaneAnglePointLeftX));
//    LOG_SUCCESS(cString::Format("m_propLaneAnglePointLeftY: %d", (int)m_propLaneAnglePointLeftY));
//    LOG_SUCCESS(cString::Format("m_propLaneAnglePointRightX: %d", (int)m_propLaneAnglePointRightX));
//    LOG_SUCCESS(cString::Format("m_propLaneAnglePointRightY: %d", (int)m_propLaneAnglePointRightY));
//    LOG_SUCCESS(cString::Format("m_propLaneMiddlePointY: %d", (int)m_propLaneMiddlePointY));
//    LOG_SUCCESS(cString::Format("m_propLaneMiddlePointILCY: %d", (int)m_propLaneMiddlePointILCY));
//    LOG_SUCCESS(cString::Format("m_propLaneMiddlePointIRCY: %d", (int)m_propLaneMiddlePointIRCY));
//    LOG_SUCCESS(cString::Format("m_propLaneDistanceRight: %d", (int)m_propLaneDistanceRight));
//    LOG_SUCCESS(cString::Format("m_propLaneDistanceRightILC: %d", (int)m_propLaneDistanceRightILC));
//    LOG_SUCCESS(cString::Format("m_propLaneDistanceLeftIRC: %d", (int)m_propLaneDistanceLeftIRC));
//    LOG_SUCCESS(cString::Format("m_propLaneDistanceMiddle: %d", (int)m_propLaneDistanceMiddle));
//    LOG_SUCCESS(cString::Format("m_propLaneDistanceLimitRight: %d", (int)m_propLaneDistanceLimitRight));
//    LOG_SUCCESS(cString::Format("m_propLaneDistanceLimitLeft: %d", (int)m_propLaneDistanceLimitLeft));
//    LOG_SUCCESS(cString::Format("m_propLaneDistanceRightFar: %d", (int)m_propLaneDistanceRightFar));
//    LOG_SUCCESS(cString::Format("m_propLaneDistanceLeftFar: %d", (int)m_propLaneDistanceLeftFar));
//    LOG_SUCCESS(cString::Format("m_propSteeringWeakRightLimit: %d", (int)m_propSteeringWeakRightLimit));
//    LOG_SUCCESS(cString::Format("m_propSteeringHeavyRightLimit: %d", (int)m_propSteeringHeavyRightLimit));
//    LOG_SUCCESS(cString::Format("m_propSteeringWeakLeftLimit: %d", (int)m_propSteeringWeakLeftLimit));
//    LOG_SUCCESS(cString::Format("m_propSteeringHeavyLeftLimit: %d", (int)m_propSteeringHeavyLeftLimit));
//    LOG_SUCCESS(cString::Format("m_propCarPositionInImageX: %d", (int)m_propCarPositionInImageX));
//    LOG_SUCCESS(cString::Format("m_propCarPositionInImageY: %d", (int)m_propCarPositionInImageY));

//    LOG_SUCCESS(cString::Format("m_propSteeringOffset: %f", (float)m_propSteeringOffset));
//    LOG_SUCCESS(cString::Format("m_propSteeringWeakRight: %f", (float)m_propSteeringWeakRight));
//    LOG_SUCCESS(cString::Format("m_propSteeringHeavyRight: %f", (float)m_propSteeringHeavyRight));
//    LOG_SUCCESS(cString::Format("m_propSteeringWeakLeft: %f", (float)m_propSteeringWeakLeft));
//    LOG_SUCCESS(cString::Format("m_propSteeringHeavyLeft: %f", (float)m_propSteeringHeavyLeft));
//    LOG_SUCCESS(cString::Format("m_propSteeringLaneOffsetRight: %f", (float)m_propSteeringLaneOffsetRight));
//    LOG_SUCCESS(cString::Format("m_propSteeringLaneOffsetLeft: %f", (float)m_propSteeringLaneOffsetLeft));
//    LOG_SUCCESS(cString::Format("m_propSteeringLaneOffsetRightFar: %f", (float)m_propSteeringLaneOffsetRightFar));
//    LOG_SUCCESS(cString::Format("m_propSteeringLaneOffsetLeftFar: %f", (float)m_propSteeringLaneOffsetLeftFar));
//    LOG_SUCCESS(cString::Format("m_propSpeedStraight: %f", (float)m_propSpeedStraight));
//    LOG_SUCCESS(cString::Format("m_propSpeedWeakCurve: %f", (float)m_propSpeedWeakCurve));
//    LOG_SUCCESS(cString::Format("m_propSpeedHeavyCurve: %f", (float)m_propSpeedHeavyCurve));
//    LOG_SUCCESS(cString::Format("m_F32InCarViewXmin: %f", (float)m_F32InCarViewXmin));
//    LOG_SUCCESS(cString::Format("m_F32InCarViewXmax: %f", (float)m_F32InCarViewXmax));
//    LOG_SUCCESS(cString::Format("m_F32InCarViewYmin: %f", (float)m_F32InCarViewYmin));
//    LOG_SUCCESS(cString::Format("m_F32InCarViewYmax: %f", (float)m_F32InCarViewYmax));

//    LOG_SUCCESS(cString::Format("m_propEnableGaussianBlur: %d", (bool)m_propEnableGaussianBlur));
//    LOG_SUCCESS(cString::Format("m_propEnableCannyEdge: %d", (bool)m_propEnableCannyEdge));
//    LOG_SUCCESS(cString::Format("m_propEnableDilation: %d", (bool)m_propEnableDilation));
//    LOG_SUCCESS(cString::Format("m_propEnableHough: %d", (bool)m_propEnableHough));
//    LOG_SUCCESS(cString::Format("m_propEnableDebugging: %d", (bool)m_propEnableDebugging));
//    LOG_SUCCESS(cString::Format("m_propEnableDebuggingVideo: %d", (bool)m_propEnableDebuggingVideo));

    RETURN_NOERROR;
}

