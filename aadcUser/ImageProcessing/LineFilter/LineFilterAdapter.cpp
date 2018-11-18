/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. Team FAUtonomous. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

TODO: write a short description of your filter!!
This filter is our template filter for aadc2018.
Use this filter for creating your own filters.
Keep in mind to change "cTemplateFilter" to your filtername.

**********************************************************************/

#include "stdafx.h"
#include "LineFilterAdapter.h"
#include "LineFilter.h"

#define LF_NEAR "Field of View::near"

#define LF_FOV_H "Field of View::Horizontal"
#define LF_FOV_V "Field of View::Vertical"

#define LF_PITCH "Camera position/rotation::pitch"
#define LF_ROLL  "Camera position/rotation::roll"
#define LF_YAW   "Camera position/rotation::yaw"

#define LF_CAMERA_X_OFFSET "Camera position/rotation::X"
#define LF_CAMERA_Y_OFFSET "Camera position/rotation::Y"
#define LF_CAMERA_Z_OFFSET "Camera position/rotation::Z"

#define LF_ADAPTIVE_THRESHOLD_CONSTANT "Image filtering::adaptive threshold constant"
#define LF_ADAPTIVE_THRESHOLD_WIDTH    "Image filtering::adaptive threshold width"
#define LF_ENABLE_ADAPTIVE_THRESHOLD   "Image filtering::enable adaptive thresholding"
#define LF_ENABLE_MEDIAN_FILTER        "Image filtering::median filter size 3x3"

// This will define the filter and expose it via plugin class factory.
// Class cTemplateFilter will be embedded to the filter and called by
// data trigger of "speed" by default.
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(
    CID_LINE_FILTER_ADAPTER_FILTER,		// references to header file
    "LineFilterAdapter",              // label
    LineFilterAdapter,                // class
    adtf::filter::pin_trigger({"inputVideo"}));	// set trigger pin


// -------------------------------
// --------- CONSTRUCTOR ---------
// -------------------------------
// in the constructor, we will set the adtf streamtypes for signal values and pins
LineFilterAdapter::LineFilterAdapter()
{
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
    Register(m_WriterVideo     , "outputVideo"      , pTypeOutput);
    Register(m_WriterDebugVideo, "outputDebugVideo" , pTypeOutput);

    m_poseStruct        .registerPin(this, m_ReaderPose , "inputPose" );
    m_SignalValuePitchId.registerPin(this, m_ReaderPitch, "inputPitch");

    //register callback for type changes
    m_ReaderVideo.SetAcceptTypeCallback([this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pVideoDataType) -> tResult
    {
        return ChangeType(m_ReaderVideo, *pVideoDataType.Get());
    });

    // register properties
    RegisterPropertyVariable(LF_ADAPTIVE_THRESHOLD_CONSTANT, m_propThresholdConstant);
    RegisterPropertyVariable(LF_ADAPTIVE_THRESHOLD_WIDTH   , m_propThresholdWidth);
    RegisterPropertyVariable(LF_CAMERA_X_OFFSET            , m_propCameraOffsetX);
    RegisterPropertyVariable(LF_CAMERA_Y_OFFSET            , m_propCameraOffsetY);
    RegisterPropertyVariable(LF_CAMERA_Z_OFFSET            , m_propCameraOffsetZ);
    RegisterPropertyVariable(LF_ENABLE_ADAPTIVE_THRESHOLD  , m_propEnableAdaptiveThresholding);
    RegisterPropertyVariable(LF_ENABLE_MEDIAN_FILTER       , m_propEnableMedianFiltering);
    RegisterPropertyVariable(LF_FOV_H                      , m_propFieldOfViewHorizontal);
    RegisterPropertyVariable(LF_FOV_V                      , m_propFieldOfViewVertical);
    RegisterPropertyVariable(LF_NEAR                       , m_propNearPlane);
    RegisterPropertyVariable(LF_PITCH                      , m_propPitch);
    RegisterPropertyVariable(LF_ROLL                       , m_propRoll);
    RegisterPropertyVariable(LF_YAW                        , m_propYaw);
    RegisterPropertyVariable("ROI X"                       , m_propROIX);
    RegisterPropertyVariable("ROI Y"                       , m_propROIY);
    RegisterPropertyVariable("ROI Width"                   , m_propROIWidth);
    RegisterPropertyVariable("ROI Height"                  , m_propROIHeight);
    RegisterPropertyVariable("Threshold constant"          , m_propNewThresholdConstand);

    // set some variables
    lineFilter = new cLineFilter();
    m_f32EstimationErrorOffset = 0;
    m_tsTimeOfPrevFrame = 0;

    // init linefilter
    lineFilter->Init(m_propNearPlane, m_propFieldOfViewHorizontal, m_propFieldOfViewVertical, m_propPitch,
                     m_propCameraOffsetY, m_propCameraOffsetZ, m_propThresholdConstant, m_propThresholdWidth, 1280, 720);
    lineFilter->EnableMedianFiltering       (m_propEnableMedianFiltering);
    lineFilter->EnableAdaptiveThresholding  (m_propEnableAdaptiveThresholding);
    lineFilter->SetCameraSize               (1280, 720);
    lineFilter->SetPitch                    (m_propPitch);
    lineFilter->SetNearFar                  (m_propNearPlane);
    lineFilter->SetCameraPosition           (m_propCameraOffsetY, m_propCameraOffsetZ);
    lineFilter->SetAdaptiveThresholdConstant(m_propThresholdConstant);
    lineFilter->SetAdaptiveThresholdWidth   (m_propThresholdWidth);
    lineFilter->SetFOV                      (m_propFieldOfViewHorizontal,
                                             m_propFieldOfViewVertical);
}


// ------------------------------
// --------- CONFIGURE ----------
// ------------------------------
// implement configure() to read ALL properties
tResult LineFilterAdapter::Configure()
{
    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

    RETURN_NOERROR;
}


// ------------------------------
// ---------- PROCESS -----------
// ------------------------------
// process() is always called each time a trigger occured (new data sample arrive).
// IMPORTANT: Make sure to read ALL samples of ALL readers!
// This is how to read a sample
tResult LineFilterAdapter::Process(tTimeStamp tmTimeOfTrigger)
{
    // video input
    object_ptr<const ISample> pReadVideoSample;
    if (IS_OK(m_ReaderVideo.GetNextSample(pReadVideoSample)))
    {
        object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
        //lock read buffer
        if (IS_OK(pReadVideoSample->Lock(pReadBuffer)))
        {
            //create a opencv matrix from the media sample buffer
            Mat inputImage (cv::Size(m_sInputImageFormat.m_ui32Width, m_sInputImageFormat.m_ui32Height),
                            CV_8UC3, (uchar*)pReadBuffer->GetPtr());
            Mat outputImage(cv::Size(m_sOutputImageFormat.m_ui32Width, m_sOutputImageFormat.m_ui32Height),
                            CV_8UC3, Scalar(0, 0, 0));
            Mat roiImage(cv::Size(m_sOutputImageFormat.m_ui32Width, m_sOutputImageFormat.m_ui32Height),
                             CV_8UC3, Scalar(0, 0, 0));
            m_MatIn  =  inputImage;
            cvtColor(m_MatIn, m_MatIn, COLOR_BGR2GRAY);
            // detect edges
            Canny(m_MatIn, m_MatIn, 120, 255, 3);

            // draw houghlines
            std::vector<Vec4i> lines;
            HoughLinesP(m_MatIn, lines, 1, CV_PI/180, 50, 25, 2 );
            cvtColor(m_MatIn, m_MatIn, COLOR_GRAY2BGR);

            for(size_t i = 0; i < lines.size(); i++)
            {
                Vec4i l = lines[i];
                line(m_MatIn, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,  255,  255), 4, CV_AA);
            }

            // ---- debug Video ----------
            // make copy of input video
            Mat debugMat = outputImage.clone();
            cvtColor(m_MatIn, debugMat, COLOR_BGR2GRAY);
            cvtColor(roiImage, roiImage, COLOR_BGR2GRAY);
            cv::GaussianBlur(m_MatIn, m_MatIn, cv::Size(3, 3), 0);
            // draw rect of mask
            //rectangle(debugMat, Rect(m_propROIX, m_propROIY, m_propROIWidth, m_propROIHeight), cv::Scalar(0, 255, 0), 10);
            /** Create some points */
            Point rook_points[1][6];
            rook_points[0][0] = Point(   0, 490);
            rook_points[0][1] = Point(1280, 490);
            rook_points[0][2] = Point(1280, 716);
            rook_points[0][3] = Point( 854, 603);
            rook_points[0][4] = Point( 428, 603);
            rook_points[0][5] = Point(   0, 716);

            const Point* ppt[1] = { rook_points[0] };
            int npt[] = { 6 };

            //fillPoly( debugMat, ppt, npt, 1, Scalar( 255, 255, 255 ), 8 );
            //Canny(debugMat, debugMat, 50, 200, 3);
            // output//Write processed Image to Output Pin
            if (!debugMat.empty())
            {
                //update output format if matrix size does not fit to
                if (debugMat.total() * debugMat.elemSize() != m_sInputImageFormat.m_szMaxByteSize)
                {
                    setTypeFromMat(m_WriterDebugVideo, debugMat);
                }
                // write to pin
                writeMatToPin(m_WriterDebugVideo, debugMat, m_pClock->GetStreamTime());
            }
            // ---------------------------

            m_MatOut = outputImage;

            // set ROI
            //rectangle(roiImage, Rect(m_propROIX, m_propROIY, m_propROIWidth, m_propROIHeight), cv::Scalar(255, 255, 255), CV_FILLED);
            fillPoly( roiImage, ppt, npt, 1, Scalar( 255, 255, 255 ), 8 );

            lineFilter->SetValidPixels(roiImage, m_MatIn.size(), true);
            try
            {
                RETURN_IF_FAILED(lineFilter->GetTopDownView(m_MatIn, roiImage, m_MatOut,
                                 m_sInputImageFormat.m_strFormatName != ADTF_IMAGE_FORMAT(BGR_24)));
            }
            catch (cv::Exception &e)
            {
                LOG_ERROR(e.msg.c_str());
            }

            threshold(m_MatOut, m_MatOut, m_propNewThresholdConstand, 255, THRESH_BINARY);// Generate Binary Image

            if (carPose.size() > 0)
            {
                CarPose pose = carPose.front();
                putText(m_MatOut, cv::String(cString::Format("%.2f %.2f %.0f", pose.position.x,
                                                               pose.position.y, 180 / CV_PI * pose.yaw).GetPtr()),
                        Point2f(0, 17), CV_FONT_NORMAL, 0.4, Scalar(155));
            }

            m_tsTimeOfPrevFrame = pReadVideoSample->GetTime();

            tFloat32 pitch = lineFilter->GetPitch();
            WriteAsFloat(m_MatOut, 0, pitch);
            cv::Point2f frameCoord = lineFilter->GetTopLeftWorldCoordinates();

            WriteAsFloat(m_MatOut, 4, frameCoord.x + m_propCameraOffsetX - m_f32EstimationErrorOffset);
            WriteAsFloat(m_MatOut, 8, frameCoord.y);
            WriteAsFloat(m_MatOut, 12, m_propNearPlane);
            m_bFirstFrame = tFalse;

            pReadBuffer->Unlock();
        }

        //cv::GaussianBlur(m_MatOut, m_MatOut, cv::Size(1, 1), 0);

        //Write processed Image to Output Pin
        if (!m_MatOut.empty())
        {
            //update output format if matrix size does not fit to
            if (m_MatOut.total() * m_MatOut.elemSize() != m_sOutputImageFormat.m_szMaxByteSize)
            {
                setTypeFromMat(m_WriterVideo, m_MatOut);
            }
            // write to pin
            writeMatToPin(m_WriterVideo, m_MatOut, m_pClock->GetStreamTime());
        }
    }

    // pose input
    TPoseStruct::Data poseIn;
    if(IS_OK(m_poseStruct.readPin(m_ReaderPose, (void *) &poseIn)))
    {
        CarPose current(Point2f(poseIn.f32PosX, poseIn.f32PosY), poseIn.f32Yaw,
                        poseIn.f32Pitch, poseIn.f32Roll, tmTimeOfTrigger, false);

        carPose.push_front(current);
    }

    // pitch input
    TSignalValue::Data pitchIn;
    if(IS_OK(m_SignalValuePitchId.readPin(m_ReaderPitch, (void *) &pitchIn, 0)))
    {
        lineFilter->SetPitch(pitchIn.f32Value);
    }
    // done
    RETURN_NOERROR;
}


// ------------------------------
// ------ OTHER FUNCTIONS -------
// ------------------------------

// setTypeFromMat
void LineFilterAdapter::setTypeFromMat(adtf::streaming::cSampleWriter& writer, const cv::Mat& outputImage, tBool keepFormat)
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
void LineFilterAdapter::writeMatToPin(adtf::streaming::cSampleWriter& writer, const cv::Mat& outputImage, tTimeStamp streamTime)
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
tResult LineFilterAdapter::ChangeType(adtf::streaming::cDynamicSampleReader& inputPin,
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
        m_sOutputImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(GREYSCALE_8);
        // and set pType also to samplewriter
        m_WriterVideo << pTypeInput;
    }
    else
    {
        RETURN_ERROR(ERR_INVALID_TYPE);
    }

    RETURN_NOERROR;
}


// WriteAsFloat
tResult LineFilterAdapter::WriteAsFloat(cv::Mat& image, tUInt32 bytePosition,
        tFloat32 value) {
    if (bytePosition > image.dataend - image.datastart) {
        RETURN_AND_LOG_ERROR_STR(ERR_INVALID_INDEX,
                "LineFilter: ReadAsFloatAndClear bytePosition out of bounds");
    }

    union {
        tUInt8 bytes[4];
        tFloat32 f;
    } floatToByte;

    floatToByte.f = value;

    image.data[bytePosition] = floatToByte.bytes[0];
    image.data[bytePosition + 1] = floatToByte.bytes[1];
    image.data[bytePosition + 2] = floatToByte.bytes[2];
    image.data[bytePosition + 3] = floatToByte.bytes[3];

    RETURN_NOERROR;
}
