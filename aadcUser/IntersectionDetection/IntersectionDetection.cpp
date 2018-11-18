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
#include "IntersectionDetection.h"
#include <common_helper.h>
#include <property_structs.h>
#include "ScmCommunication.h"
#include <cmath>
#include "tinyxml2.h"

// This will define the filter and expose it via plugin class factory.
// Class cTemplateFilter will be embedded to the filter and called by
// data trigger of "speed" by default.
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(
    CID_INTERSECTION_DETECTION_FILTER,		// references to header file
    "IntersectionDetection",              // label
    IntersectionDetection,                // class
    adtf::filter::pin_trigger({"inputVideo"}));	// set trigger pin


// -------------------------------
// --------- CONSTRUCTOR ---------
// -------------------------------
// in the constructor, we will set the adtf streamtypes for signal values and pins
IntersectionDetection::IntersectionDetection()
{

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
    Register(m_WriterDebugBirdVideo, "outputDebugBirdVideo"   , pTypeOutput);

    //register callback for type changes
    m_ReaderVideo.SetAcceptTypeCallback([this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pVideoDataType) -> tResult
    {
        return ChangeType(m_ReaderVideo, *pVideoDataType.Get());
    });

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
    RegisterPropertyVariable("Imageprocessing/Threshold/enable"       , m_propEnableBinaryThreshold);
    RegisterPropertyVariable("Imageprocessing/Threshold/threshold"    , m_propBinaryThreshold);
    RegisterPropertyVariable("Imageprocessing/Harris/enable"          , m_propEnableCornerHarris);
    RegisterPropertyVariable("Imageprocessing/Harris/blocksize"       , m_propHarrisBlockSize);
    RegisterPropertyVariable("Imageprocessing/Harris/aperture"        , m_propHarrisAperture);
    RegisterPropertyVariable("Imageprocessing/Harris/threshold"       , m_propHarrisThreshold);

    RegisterPropertyVariable("Imageprocessing/LaneRoiWidth", m_propLaneROIWidth);
    RegisterPropertyVariable("Frames", m_propFrame);

    m_ui32FrameCounter = 0;
}


// ------------------------------
// --------- CONFIGURE ----------
// ------------------------------
// implement configure() to read ALL properties
tResult IntersectionDetection::Configure()
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
tResult IntersectionDetection::Process(tTimeStamp)
{

    // video input
    object_ptr<const ISample> pReadVideoSample;
    if (IS_OK(m_ReaderVideo.GetNextSample(pReadVideoSample)))
    {
        object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
        //lock read buffer
        if (IS_OK(pReadVideoSample->Lock(pReadBuffer)))
        {
            if(m_ui32FrameCounter == m_propFrame)
            {

                // create a opencv matrix from the media sample buffer
                Mat inputImage (cv::Size(m_sInputImageFormat.m_ui32Width, m_sInputImageFormat.m_ui32Height),
                                CV_8UC3, (uchar*)pReadBuffer->GetPtr());
                Mat matRoi (cv::Size(m_propTransformedImageWidth, m_propTransformedImageHeight),
                            CV_8UC3, Scalar(0, 0, 0));

                // set our global mat to input image
                m_MatInputImage       = inputImage;
                m_MatOutputDebugImage = m_MatInputImage.clone();

                setRoiOfLanes();
                transformToBirdsEyeView();

                cvtColor(m_MatTransformedImage, m_MatTransformedImage, CV_BGR2GRAY);

                if(m_propEnableGaussianBlur)
                {
                    GaussianBlur(m_MatTransformedImage, m_MatTransformedImage,
                                     cv::Size(m_propGaussianKernelSize, m_propGaussianKernelSize), 0);
                }

                if(m_propEnableBinaryThreshold)
                {
                    threshold(m_MatTransformedImage, m_MatTransformedImage,
                              m_propBinaryThreshold, 255, THRESH_BINARY);// Generate Binary Image
                }

                Mat cornerMat, cornerNorm, cornerScaled;;
                cornerMat = Mat::zeros(m_MatTransformedImage.size(), CV_32FC1);
                cornerHarris(m_MatTransformedImage, cornerMat, m_propHarrisBlockSize,
                             m_propHarrisAperture, 0.4f, BORDER_DEFAULT);

                normalize(cornerMat, cornerNorm, 0, 255, NORM_MINMAX, CV_32FC1, Mat());
                convertScaleAbs(cornerNorm, cornerScaled);

                cvtColor(m_MatTransformedImage, m_MatTransformedImage, CV_GRAY2BGR);

                for(int j = 0; j < cornerNorm.rows; ++j)
                {
                    for(int i = 0; i < cornerNorm.cols; ++i)
                    {
                        if((int) cornerNorm.at<float>(j, i) < m_propHarrisThreshold)
                        {
                            circle(m_MatTransformedImage, Point(i, j), 3, Scalar(255, 150, 70), CV_AA);
                            if(i >= m_propTransformedImageWidth - m_propLaneROIWidth)
                            {
                                m_bFreeRois[j / (m_propTransformedImageHeight/10)] = true;
                            }
                        }
                    }
                }

                for(int i = 0; i < 10; ++i)
                {
                    if(m_bFreeRois[i])
                    {
                        rectangle(matRoi, Rect(m_propTransformedImageWidth - m_propLaneROIWidth, (m_propTransformedImageHeight/10) * i,
                                               m_propLaneROIWidth, m_propTransformedImageHeight/10), Scalar(0, 255, 0), CV_AA);
                        rectangle(matRoi, Rect(m_propTransformedImageWidth - m_propLaneROIWidth, (m_propTransformedImageHeight/10) * i,
                                               m_propLaneROIWidth, m_propTransformedImageHeight/10), Scalar(0, 180, 0), CV_FILLED);
                    }
                    else
                    {
                        rectangle(matRoi, Rect(m_propTransformedImageWidth - m_propLaneROIWidth, (m_propTransformedImageHeight/10) * i,
                                               m_propLaneROIWidth, m_propTransformedImageHeight/10), Scalar(0, 0, 255), CV_AA);
                        rectangle(matRoi, Rect(m_propTransformedImageWidth - m_propLaneROIWidth, (m_propTransformedImageHeight/10) * i,
                                               m_propLaneROIWidth, m_propTransformedImageHeight/10), Scalar(0, 0, 180), CV_FILLED);
                    }

                }

                addWeighted(m_MatTransformedImage, 1, matRoi, 0.4f, 0.0, m_MatTransformedImage);
                for(int i = 0; i < 10; ++i)
                {
                    m_bFreeRois[i] = false;
                }
                m_ui32FrameCounter = 0;
            }

            m_ui32FrameCounter++;
        }

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
        if (!m_MatTransformedImage.empty())
        {
            //update output format if matrix size does not fit to
            if (m_MatTransformedImage.total() * m_MatTransformedImage.elemSize() != m_sOutputImageFormat.m_szMaxByteSize)
            {
                setTypeFromMat(m_WriterDebugBirdVideo, m_MatTransformedImage);
            }
            // write to pin
            writeMatToPin(m_WriterDebugBirdVideo, m_MatTransformedImage, m_pClock->GetStreamTime());
        }

    }
    RETURN_NOERROR;
}


// ------------------------------
// ------ OTHER FUNCTIONS -------
// ------------------------------

// ------------- Functions for VIDEO stuff
// setTypeFromMat
// we need this for video outputs
void IntersectionDetection::setTypeFromMat(adtf::streaming::cSampleWriter& writer, const cv::Mat& outputImage, tBool keepFormat)
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
void IntersectionDetection::writeMatToPin(adtf::streaming::cSampleWriter& writer, const cv::Mat& outputImage, tTimeStamp streamTime)
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
tResult IntersectionDetection::ChangeType(adtf::streaming::cDynamicSampleReader& inputPin,
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
        m_sOutputImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
        // and set pType also to samplewriter
        m_WriterDebugVideo     << pTypeInput;
        m_WriterDebugBirdVideo     << pTypeInput;
    }
    else
    {
        RETURN_ERROR(ERR_INVALID_TYPE);
    }

    RETURN_NOERROR;
}


// setRoiOfLanes
// This function will set our ROI, where the lanes are located
// make sure to set right coordinates of ROI in properties! :)
void IntersectionDetection::setRoiOfLanes()
{
    // set roi points
    m_p2fROIVertices[0] = Point(m_propROIPoint1X, m_propROIPoint1Y); // upper left
    m_p2fROIVertices[1] = Point(m_propROIPoint2X, m_propROIPoint2Y); // upper right
    m_p2fROIVertices[2] = Point(m_propROIPoint3X, m_propROIPoint3Y); // lower right
    m_p2fROIVertices[3] = Point(m_propROIPoint4X, m_propROIPoint4Y); // lower left

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

// transformToBirdsEyeView
// This function will transform our image to bird's eye view
// transformed image is stored in m_MatTransformedImage
void IntersectionDetection::transformToBirdsEyeView()
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
    m_MatTransformedImage = dst.clone();
}


//this one is for the fans
tResult IntersectionDetection::LoadProperties()
{
    intersection_detection_binh_properties properties;
    LoadXMLData("/home/aadc/AADC/utilities/propertyXMLs/intersection_detection_binh.xml", (void*) (&properties));

    // unsigned ints
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

    m_propGaussianKernelSize  = properties.m_propGaussianKernelSize;
    m_propBinaryThreshold     = properties.m_propBinaryThreshold;

    m_propHarrisBlockSize = properties.m_propHarrisBlockSize;
    m_propHarrisAperture  = properties.m_propHarrisAperture;
    m_propHarrisThreshold = properties.m_propHarrisThreshold;

    m_propLaneROIWidth = properties.m_propLaneROIWidth;
    m_propFrame = properties.m_propFrame;

    // bools
    m_propEnableGaussianBlur     = properties.m_propEnableGaussianBlur;
    m_propEnableBinaryThreshold  = properties.m_propEnableBinaryThreshold;
    m_propEnableCornerHarris     = properties.m_propEnableCornerHarris;


    LOG_SUCCESS(cString::Format("m_propROIPoint1X: %d", (unsigned int)m_propROIPoint1X));
    LOG_SUCCESS(cString::Format("m_propROIPoint1Y: %d", (unsigned int)m_propROIPoint1Y));
    LOG_SUCCESS(cString::Format("m_propROIPoint2X: %d", (unsigned int)m_propROIPoint2X));
    LOG_SUCCESS(cString::Format("m_propROIPoint2Y: %d", (unsigned int)m_propROIPoint2Y));
    LOG_SUCCESS(cString::Format("m_propROIPoint3X: %d", (unsigned int)m_propROIPoint3X));
    LOG_SUCCESS(cString::Format("m_propROIPoint3Y: %d", (unsigned int)m_propROIPoint3Y));
    LOG_SUCCESS(cString::Format("m_propROIPoint4X: %d", (unsigned int)m_propROIPoint4X));
    LOG_SUCCESS(cString::Format("m_propROIPoint4Y: %d", (unsigned int)m_propROIPoint4Y));

    LOG_SUCCESS(cString::Format("m_propTransformedImageWidth: %d" , (unsigned int)m_propTransformedImageWidth));
    LOG_SUCCESS(cString::Format("m_propTransformedImageHeight: %d", (unsigned int)m_propTransformedImageHeight));
    LOG_SUCCESS(cString::Format("m_propGaussianKernelSize: %d"    , (unsigned int)m_propGaussianKernelSize));
    LOG_SUCCESS(cString::Format("m_propBinaryThreshold: %d"       , (unsigned int)m_propBinaryThreshold));
    LOG_SUCCESS(cString::Format("m_propHarrisBlockSize: %d"       , (unsigned int)m_propHarrisBlockSize));
    LOG_SUCCESS(cString::Format("m_propHarrisAperture: %d"        , (unsigned int)m_propHarrisAperture));

    LOG_SUCCESS(cString::Format("m_propHarrisThreshold: %d"       , (unsigned int)m_propHarrisThreshold));
    LOG_SUCCESS(cString::Format("m_propLaneROIWidth: %d"          , (unsigned int)m_propLaneROIWidth));
    LOG_SUCCESS(cString::Format("m_propFrame: %d"                 , (unsigned int)m_propFrame));

    LOG_SUCCESS(cString::Format("m_propEnableGaussianBlur: %d"   , (bool)m_propEnableGaussianBlur));
    LOG_SUCCESS(cString::Format("m_propEnableBinaryThreshold: %d", (bool)m_propEnableBinaryThreshold));
    LOG_SUCCESS(cString::Format("m_propEnableCornerHarris: %d"   , (bool)m_propEnableCornerHarris));

    RETURN_NOERROR;
}

