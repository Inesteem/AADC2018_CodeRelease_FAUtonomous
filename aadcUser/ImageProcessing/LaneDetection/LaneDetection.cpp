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
#include "LaneDetection.h"

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
    //create and set inital input format type
    m_sInputImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(GREYSCALE_8);
    adtf::ucom::object_ptr<IStreamType> pVideoDataType = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pVideoDataType, m_sInputImageFormat);

    // set output type
    adtf::ucom::object_ptr<IStreamType> pTypeOutput = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pTypeOutput, m_sOutputImageFormat);

    // Register input pin
    Register(m_ReaderVideo, "inputVideo" , pVideoDataType);
    // Register output pin
    Register(m_WriterDebugVideo, "outputDebugVideo" , pTypeOutput);

    // Register other pins
    m_SteeringId.registerPin(this, m_WriterSteering, "steeringOut" );

    //register callback for type changes
    m_ReaderVideo.SetAcceptTypeCallback([this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pVideoDataType) -> tResult
    {
        return ChangeType(m_ReaderVideo, *pVideoDataType.Get());
    });

}


// ------------------------------
// --------- CONFIGURE ----------
// ------------------------------
// implement configure() to read ALL properties
tResult LaneDetection::Configure()
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
tResult LaneDetection::Process(tTimeStamp tmTimeOfTrigger)
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
                            CV_8UC1, (uchar*)pReadBuffer->GetPtr());
            Mat maskImage(cv::Size(m_sOutputImageFormat.m_ui32Width, m_sOutputImageFormat.m_ui32Height),
                            CV_8UC1, Scalar(0, 0, 0));
            Mat tmp(cv::Size(m_sOutputImageFormat.m_ui32Width, m_sOutputImageFormat.m_ui32Height),
                            CV_8UC1, Scalar(0, 0, 0));

            m_MatIn       =  inputImage;


            rectangle(maskImage, Rect(75, 170, 145, 130), cv::Scalar(255, 255, 255), CV_FILLED);

            // make copy of input video
            m_MatDebugOut = m_MatIn.clone();
            m_MatDebugOut.copyTo(tmp, maskImage);
            m_MatDebugOut = tmp.clone();

            // draw houghlines
            std::vector<Vec4i> lines;
            Vec4i leftLane, rightLane;
            HoughLinesP(m_MatDebugOut, lines, 1, CV_PI/180, 50, 25, 2 );

            for(size_t i = 0; i < lines.size(); i++)
            {
                Vec4i validLine = lines[i];
                // left line
                if(validLine[0] >= 75 && validLine[0] < 150 && validLine[1] > 170)
                {
                    if(leftLane[0] <= validLine[0])
                    {
                        leftLane = validLine;
                    }
                }
                // right line
                if(validLine[0] >= 150 && validLine[0] < 220 && validLine[1] > 170)
                {
                    if(rightLane[0] >= validLine[0] || rightLane[0] < 150)
                    {
                        rightLane[0] = validLine[2];
                        rightLane[1] = validLine[3];
                        rightLane[2] = validLine[0];
                        rightLane[3] = validLine[1];
                    }
                }

            }

            if(leftLane[0] == 0 && leftLane[1] == 0 && leftLane[2] == 0 && leftLane[3] == 0)
            {
                leftLane = m_lastLeftLine;
            }
            else
            {
                m_lastLeftLine = leftLane;
            }
            if(rightLane[0] == 0 && rightLane[1] == 0 && rightLane[2] == 0 && rightLane[3] == 0)
            {
                rightLane = m_lastRightLine;
            }
            else
            {
                m_lastRightLine = rightLane;
            }

            tFloat32 slopeLeftLine  = 0;
            tFloat32 slopeRightLine = 0;

            if((leftLane[2] - leftLane[0]) != 0 && leftLane[3] >= leftLane[1])
            {
                slopeLeftLine  = (leftLane[3] - leftLane[1])/(leftLane[2] - leftLane[0]);
            }
            if((leftLane[2] - leftLane[0]) != 0 && leftLane[3] < leftLane[1])
            {
                slopeLeftLine  = (leftLane[1] - leftLane[3])/(leftLane[0] - leftLane[2]);
            }
            if((rightLane[2] - rightLane[0]) != 0 && rightLane[3] >= rightLane[1])
            {
                slopeRightLine = (rightLane[3] - rightLane[1])/(rightLane[2] - rightLane[0]);
            }
            if((rightLane[2] - rightLane[0]) != 0 && rightLane[3] < rightLane[1])
            {
                slopeRightLine = (rightLane[1] - rightLane[3])/(rightLane[0] - rightLane[2]);
            }

            if(m_FrameCounter == 2)
            {
                if(slopeLeftLine <= 2 && slopeLeftLine >= 0 && slopeRightLine <= 2 && slopeRightLine >= 0 && !(slopeLeftLine == 0 && slopeRightLine == 0))
                {
                    m_SteeringAngle.f32Value = -43;
                    LOG_INFO(cString::Format("left: %f/%f", slopeLeftLine, slopeRightLine));

                }
                else if(slopeLeftLine > -2 && slopeLeftLine <= 0 && slopeRightLine > -2 && slopeRightLine <= 0 && !(slopeLeftLine == 0 && slopeRightLine == 0))
                {
                    m_SteeringAngle.f32Value = 45;
                    LOG_INFO(cString::Format("right: %f/%f", slopeLeftLine, slopeRightLine));

                }
                else if(slopeLeftLine == 0 && slopeRightLine == 0)
                {
                    // do nothing
                }
                else
                {
                    m_SteeringAngle.f32Value = -15;
                    LOG_INFO(cString::Format("straight: %f/%f", slopeLeftLine, slopeRightLine));
                }
                RETURN_IF_FAILED(m_SteeringId.writePin(m_WriterSteering, (void *) &m_SteeringAngle, m_pClock->GetStreamTime()));
            }
            m_FrameCounter++;
            if(m_FrameCounter == 3)
            {
                m_FrameCounter = 0;
            }

            //LOG_INFO(cString::Format("Steigungen: %f/%f", slopeLeftLine, slopeRightLine));

            cvtColor(m_MatDebugOut, m_MatDebugOut, COLOR_GRAY2BGR);
            line(m_MatDebugOut, Point( leftLane[0],  leftLane[1]), Point( leftLane[2],  leftLane[3]), Scalar(255,    0,  0), 2, CV_AA);
            line(m_MatDebugOut, Point(rightLane[0], rightLane[1]), Point(rightLane[2], rightLane[3]), Scalar(255,  255,  0), 2, CV_AA);
            circle(m_MatDebugOut, Point( leftLane[0],  leftLane[1]), 2, Scalar(0, 0, 255));
            circle(m_MatDebugOut, Point( leftLane[2],  leftLane[3]), 2, Scalar(0, 255, 0));
            circle(m_MatDebugOut, Point( rightLane[0],  rightLane[1]), 2, Scalar(0, 0, 255));
            circle(m_MatDebugOut, Point( rightLane[2],  rightLane[3]), 2, Scalar(0, 255, 0));

            // horizontal
            line(m_MatDebugOut, Point(0, 170), Point(300, 170), Scalar(0, 255, 255), 1, CV_AA);
            // vertical
            line(m_MatDebugOut, Point( 75, 170), Point( 75, 300), Scalar(0, 255, 255), 1, CV_AA);
            line(m_MatDebugOut, Point(220, 170), Point(220, 300), Scalar(0, 255, 255), 1, CV_AA);

            // car
            rectangle(m_MatDebugOut, Rect(152, 280, 5, 5), cv::Scalar(0, 0, 255), CV_FILLED);

        }
        // output//Write processed Image to Output Pin
        if (!m_MatDebugOut.empty())
        {
            //update output format if matrix size does not fit to
            if (m_MatDebugOut.total() * m_MatDebugOut.elemSize() != m_sOutputImageFormat.m_szMaxByteSize)
            {
                setTypeFromMat(m_WriterDebugVideo, m_MatDebugOut);
            }
            // write to pin
            writeMatToPin(m_WriterDebugVideo, m_MatDebugOut, m_pClock->GetStreamTime());
        }
    }

    // done
    RETURN_NOERROR;
}


// ------------------------------
// ------ OTHER FUNCTIONS -------
// ------------------------------

// setTypeFromMat
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
        m_WriterDebugVideo << pTypeInput;
    }
    else
    {
        RETURN_ERROR(ERR_INVALID_TYPE);
    }

    RETURN_NOERROR;
}
