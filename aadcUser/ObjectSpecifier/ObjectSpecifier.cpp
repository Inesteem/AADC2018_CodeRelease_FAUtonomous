/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The spelling mistake of brake was done by the employees of Audi ;)

This filter is an example filter to learn ADTF.
Filter was presented in first adtf online tutorial by Audi.
It takes 'speed' and the laser scanner as input and will signal an emergency break.
$from the frist video session enhanced by Illmer and Xiangfei 21.08.2018
Annotation: this filer is based on the filter from the video session completed by the solution of the last year team and own ideas

Annotation about the LIDAR open angle: from car left 270° to car front 360°/0° to car right 90°
**********************************************************************/
#include <mutex>
#include "stdafx.h"
#include "ObjectSpecifier.h"
#include <ADTF3_helper.h>
#include "ScmCommunication.h"


#define PI 3.1415


static int compareAngle(const void * l1, const void *l2)
{
    TPolarCoordiante::Data *pc1 = (TPolarCoordiante::Data *) l1;
    TPolarCoordiante::Data *pc2 = (TPolarCoordiante::Data *) l2;
    return pc1->f32Angle > pc2->f32Angle;

}

// This will define the filter and expose it via plugin class factory.
// Class EmergencyBreak will be embedded to the filter and called by
// data trigger of "speed" by default.
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(
        CID_OBJECT_SPECIFIER_FILTER,		// references to header file
        "cObjectSpecifier",			// label
        ObjectSpecifier,				// class

        //adtf::filter::pin_trigger({"laserscannerIn"}));
        //adtf::filter::pin_trigger({"actionInput"}));

        adtf::filter::pin_trigger({"inputVideo"}));

        //adtf::filter::timer_trigger(200000));



// -------------------------------
// --------- CONSTRUCTOR ---------
// -------------------------------
// in the constructor, we will set the adtf streamtypes for signal values and pins
ObjectSpecifier::ObjectSpecifier()
{
    // ------------------------------------------
    // create pointers for adtf streamtypes
    // coding convention:       p******DataType
    //object_ptr<IStreamType> pSpeedDataType;
    SetName("ObjectSpecifier Constructor");


    //set pins
    o_LaserScanner.registerPin(this, m_ReaderLaserScanner           , "laserscannerIn"                  );
    o_ActionStruct.registerPin(this, m_ReaderAction                 , "actionInput"                     );
    o_FeedbackStruct.registerPin(this, m_WriterFeedback             , "feedbackOutput"                  );
    // ------------------------------------------
    // --------- VIDEO PINS ----------
    //create and set inital input format type
    m_sInputImageFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    adtf::ucom::object_ptr<IStreamType> pVideoDataType = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pVideoDataType, m_sInputImageFormat);

    // set output type
    adtf::ucom::object_ptr<IStreamType> pTypeOutput = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pTypeOutput, m_sOutputImageFormat);
    // initialize variables
    // Register input pin
    Register(m_ReaderVideo, "inputVideo" , pVideoDataType);
    // Register output pin
    Register(m_WriterDebugVideo    , "outputDebugVideo"       , pTypeOutput);

    //register callback for type changes
    m_ReaderVideo.SetAcceptTypeCallback([this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pVideoDataType) -> tResult
    {
        return ChangeType(m_ReaderVideo, *pVideoDataType.Get());
    });
    // -----------------------------------------
    // set property variables
    RegisterPropertyVariable("Debug/ Lidar"      , m_propBDebugLidar);
    RegisterPropertyVariable("Lidar/ Threshold/ ObjectDetection"      , m_propF32ObjectThresholdDetection);//m_propF32ObjectThresholdDetection
    RegisterPropertyVariable("Lidar/ Threshold/ Radius"               , m_propF32ObjectThresholdRadius   );

    RegisterPropertyVariable("Lidar/ ROI normal driving/ Xmin"               , m_propF32ROIDollXmin   );
    RegisterPropertyVariable("Lidar/ ROI normal driving/ Xmax"               , m_propF32ROIDollXmax   );
    RegisterPropertyVariable("Lidar/ ROI normal driving/ Ymin"               , m_propF32ROIDollYmin   );
    RegisterPropertyVariable("Lidar/ ROI normal driving/ Ymax"               , m_propF32ROIDollYmax   );

    RegisterPropertyVariable("Lidar/ Threshold/ Adult Doll Width"            , m_propF32AdultDollWidth  );
    RegisterPropertyVariable("Lidar/ Threshold/ Doll to another object"      , m_propF32ThresholdDollAnotherObject  );//
    RegisterPropertyVariable("Lidar/ Threshold/ Distance In General relevant Object"      , m_propF32ThresholdRelevantObjectLidarProcessing  );//
    m_propF32ThresholdDollAnotherObject = 150.0;
   // LOG_INFO("OS Config finished");
}


// ------------------------------
// --------- CONFIGURE ----------
// ------------------------------
// implement configure() to read ALL properties
tResult ObjectSpecifier::Configure()
{
    //m_ui32TimestampSpeed = 0;
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    RETURN_NOERROR;
}


// ------------------------------
// ---------- PROCESS -----------
// ------------------------------
// process() is always called each time a trigger occured (new data sample arrive).
// IMPORTANT: Make sure to read ALL samples of ALL readers!
tResult ObjectSpecifier::Process(tTimeStamp tmTimeOfTrigger)
{
    if(IS_OK(ProcessActionInput()))
    {

    }

    tBool dollDetetedByLidar = false;
    tBool dollDetetedByCam = false;

    TLaserScannerData::Data inputLaserData;
    if(IS_OK(o_LaserScanner.readPin(m_ReaderLaserScanner, (void *) &inputLaserData)))
    {
        //ObjectSpecifier::LidarObstacles lidarObstacles = ObstacleDetectionWithLidar(inputLaserData);
        TLaserScannerData::Data processLidarData = ProcessLidar(inputLaserData);//neg to pos
        //dollDetetedByLidar = CheckForDoll(processLidarData);

        if(m_action.ui32Command == AC_OS_DETECT_NORMAL_DRIVE)
        {
            //LOG_INFO("Before check for doll");
            dollDetetedByLidar = CheckForDoll(processLidarData);
            //LOG_INFO("After check for doll");
        }
    }

    // video in
    object_ptr<const ISample> pReadVideoSample;
    if(IS_OK(m_ReaderVideo.GetNextSample(pReadVideoSample)))
    {
        //LOG_INFO("got frame");
        object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
        //lock read buffer
        if (IS_OK(pReadVideoSample->Lock(pReadBuffer)))
        {

                // create a opencv matrix from the media sample buffer
            Mat inputImage (cv::Size(m_sInputImageFormat.m_ui32Width, m_sInputImageFormat.m_ui32Height),
                            CV_8UC3, (uchar*)pReadBuffer->GetPtr());

            // set our global mat to input image
            m_MatInputImage  = inputImage;
            m_MatOutputImage = m_MatInputImage.clone();
            cvtColor(m_MatOutputImage, m_MatOutputImage, CV_RGB2HSV);
            Mat mask (cv::Size(m_sInputImageFormat.m_ui32Width, m_sInputImageFormat.m_ui32Height),
                            CV_8UC3, Scalar(0, 0, 0));
            inRange(m_MatOutputImage, Scalar(145, 80, 80), Scalar(160, 255, 255), mask);

            int x = 640;
            int y = 340;
            int w = 400;
            int h = 340;

            Mat barbieRoi = mask(Rect(x,y,w,h));
            int barbiePix  = countNonZero(barbieRoi);

            //LOG_INFO("%d", barbiePix);
            cvtColor(m_MatOutputImage, m_MatOutputImage, CV_HSV2RGB);
            if(barbiePix > 10)
            {
                rectangle(m_MatOutputImage, Rect(x,y,w,h), Scalar(0, 255, 0), 10, CV_AA);

                if(m_action.ui32Command == AC_OS_DETECT_NORMAL_DRIVE)
                {
                    dollDetetedByCam = true;
                }
            }
            else
            {
                rectangle(m_MatOutputImage, Rect(x,y,w,h), Scalar(255, 0, 0), 10, CV_AA);
            }
        }


        // output//Write processed Image to Output Pin
        if (!m_MatOutputImage.empty())
        {
            //update output format if matrix size does not fit to
            if (m_MatOutputImage.total() * m_MatOutputImage.elemSize() != m_sOutputImageFormat.m_szMaxByteSize)
            {
                setTypeFromMat(m_WriterDebugVideo, m_MatOutputImage);
            }
            // write to pin
            writeMatToPin(m_WriterDebugVideo, m_MatOutputImage, m_pClock->GetStreamTime());
        }

    }
    /*if(dollDetetedByCam)
    {
        LOG_INFO("Doll bz cam!");
    }
    if(dollDetetedByLidar)
    {
        LOG_INFO("Doll bz lidar!");
    }*/
    if(dollDetetedByCam && dollDetetedByLidar)
    {
        if(m_bTransmitFeedbackFlag)
        {
            TransmitFeedbackChildDollDetected();
            m_bTransmitFeedbackFlag = tFalse;
        }
       // LOG_INFO("I'm a Barbie Girl");
    //    LOG_INFO("In a barie world!");
    }
    else
    {
        m_bTransmitFeedbackFlag = tTrue;
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
void ObjectSpecifier::setTypeFromMat(adtf::streaming::cSampleWriter& writer, const cv::Mat& outputImage, tBool keepFormat)
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
void ObjectSpecifier::writeMatToPin(adtf::streaming::cSampleWriter& writer, const cv::Mat& outputImage, tTimeStamp streamTime)
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
tResult ObjectSpecifier::ChangeType(adtf::streaming::cDynamicSampleReader& inputPin,
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
    }
    else
    {
        RETURN_ERROR(ERR_INVALID_TYPE);
    }

    RETURN_NOERROR;
}
tResult ObjectSpecifier::ProcessActionInput()
{
    TActionStruct::Data inputAction;
    tResult res;
    static tTimeStamp lasttmaction = 0;//to save the last timestamp
    if(IS_FAILED(res = o_ActionStruct.readPin(m_ReaderAction, (void *) &inputAction, lasttmaction)))
    {
        RETURN_ERROR(res);
    }
    lasttmaction = inputAction.ui32ArduinoTimestamp;

    /* Filter gets ui32Command to start the timer */
    if(inputAction.bEnabled && inputAction.bStarted && inputAction.ui8FilterId == F_OBJECT_SPECIFIER)
    {
        if(inputAction.ui32Command == AC_OS_DETECT_NORMAL_DRIVE)
        {
            //LOG_ERROR("110");
            if(m_propBDebugLidar)LOG_INFO("AC_OS_DETECT_NORMAL_DRIVE");
        }
        else if(inputAction.ui32Command == AC_OS_DETECT_CROSSING_RIGHT)
        {
            if(m_propBDebugLidar)LOG_INFO("AC_OS_DETECT_CROSSING_RIGHT");

        }
        else if(inputAction.ui32Command == AC_OS_DETECT_CROSSING_LEFT)
        {
        if(m_propBDebugLidar)LOG_INFO("AC_OS_DETECT_CROSSING_LEFT");

        }
        else if(inputAction.ui32Command == AC_OS_DETECT_CROSSING_STRAIGHT_AHEAD)
        {
            if(m_propBDebugLidar)LOG_INFO("AC_OS_DETECT_CROSSING_STRAIGHT_AHEAD");
        }
        else
        {
            LOG_ERROR("OS: Unknown action command: %d", inputAction.ui32Command);
        }
    }
    m_action = inputAction;
    //LOG_INFO("got action %d", m_action.ui32Command);
    RETURN_NOERROR;
}


TLaserScannerData::Data ObjectSpecifier::ProcessLidar(TLaserScannerData::Data lasersample)
{/*Corrects angle and sorts the angles from the smallest to the greatest angle*/
    TLaserScannerData::Data processLaser = lasersample;

    for (uint32_t i=0; (i<processLaser.ui32Size); i++)
    {
        processLaser.tScanArray[i].f32Angle = AngleCompensation(processLaser.tScanArray[i].f32Angle);
    }
    //sort the input data
    qsort((void *) processLaser.tScanArray, processLaser.ui32Size, sizeof(TPolarCoordiante::Data), compareAngle);
    return processLaser;
}

tBool ObjectSpecifier::CheckForDoll(TLaserScannerData::Data lasersample)
{
    vector <LidarPoint> lidarPointPossiblePointDoll;
    for(tUInt32 i = 0; i < lasersample.ui32Size; i++)
    {

       if((lasersample.tScanArray[i].f32Radius < m_propF32ThresholdRelevantObjectLidarProcessing) && (lasersample.tScanArray[i].f32Radius != 0.0))
       {

           LidarPoint scanArray;
           scanArray.ui32Counter = i;
           scanArray.f32Radius = lasersample.tScanArray[i].f32Radius;
           scanArray.f32Angle = lasersample.tScanArray[i].f32Angle;
           scanArray.f32y = lasersample.tScanArray[i].f32Radius * sinf((90-lasersample.tScanArray[i].f32Angle)*tFloat32(PI)/180.0f);
           scanArray.f32x = lasersample.tScanArray[i].f32Radius * cosf((90-lasersample.tScanArray[i].f32Angle)*tFloat32(PI)/180.0f);
           if((scanArray.f32x < m_propF32ROIDollXmax) && (scanArray.f32x > m_propF32ROIDollXmin) && (scanArray.f32y < m_propF32ROIDollYmax) &&
                   (scanArray.f32y > m_propF32ROIDollYmin))
           {
               lidarPointPossiblePointDoll.push_back(scanArray);
           }
       }
    }
    if(lidarPointPossiblePointDoll.size() < 2)
    {
        if(m_propBDebugLidar)LOG_INFO("Not enough points for the doll: %d", lidarPointPossiblePointDoll.size() );
        return tFalse;
    }
    /*I assume the point with the smallest angle is the closest object. And the only possible doll. No other object will be considerate (as a result of lack of time :(*/
    vector <LidarPoint> coorDollFinal;
    tUInt32 ui32LastPoint = 999;
    for (tUInt32 i = 0; i < lidarPointPossiblePointDoll.size(); i++)
    {
        /*distance current point to the first detection in the roi*/
        tFloat32 f32Distance = sqrt((pow((lidarPointPossiblePointDoll[i].f32x-lidarPointPossiblePointDoll[0].f32x), 2))+
                pow((lidarPointPossiblePointDoll[i].f32y-lidarPointPossiblePointDoll[0].f32y), 2));
        if(f32Distance < m_propF32AdultDollWidth)
        {
            coorDollFinal.push_back(lidarPointPossiblePointDoll[i]);
            ui32LastPoint = i;
        }
        else
        {
            if(ui32LastPoint == 999)
            {
                LOG_WARNING("ui32LastPoint is 999 -> seg fault; something went terribly wrong!");
                return tFalse;
            }
            if(coorDollFinal.size() < 1)
            {
                if(m_propBDebugLidar)LOG_INFO("False: Not enough final points for the doll: %d", coorDollFinal.size());
                return tFalse;
            }
            /*distance of the current point to the last point*/
            if (ui32LastPoint >= lidarPointPossiblePointDoll.size())LOG_ERROR("Possible seg fault!!!");
            tFloat32 f32DistanceToLast = sqrt((pow((lidarPointPossiblePointDoll[i].f32x-lidarPointPossiblePointDoll[ui32LastPoint].f32x), 2))+
                                              pow((lidarPointPossiblePointDoll[i].f32y-lidarPointPossiblePointDoll[ui32LastPoint].f32y), 2));

            if(f32DistanceToLast > m_propF32ThresholdDollAnotherObject)
            {
                tFloat32 f32DistanceBeginToEnd = sqrt((pow((lidarPointPossiblePointDoll[0].f32x-lidarPointPossiblePointDoll[ui32LastPoint].f32x), 2))+
                        pow((lidarPointPossiblePointDoll[0].f32y-lidarPointPossiblePointDoll[ui32LastPoint].f32y), 2));
                if(f32DistanceBeginToEnd > m_propF32AdultDollWidth)
                {
                    if(m_propBDebugLidar)LOG_INFO("false, object is too big!; Distance begin to end: %f", f32DistanceBeginToEnd);
                    return tFalse;
                    //RETURN_NOERROR;
                }
                else
                {
                    if(coorDollFinal.size() <= 1)
                    {
                        /*sometime the barbie is sometimes only one point*/
                        if(m_propBDebugLidar)LOG_INFO("False because it is/are only %d point(s)", coorDollFinal.size());
                        return tFalse;
                    }

                }
            }
            else
            {
                if(m_propBDebugLidar)LOG_INFO("false, object is too big!: %f", f32DistanceToLast);
                return tFalse;
            }
        }
    }
    /*one the left hand side*/
    tFloat32 f32DistanceLastValidPointToLastxy = 0.0f;
    tFloat32 f32Lastx = lidarPointPossiblePointDoll[0].f32x;
    tFloat32 f32Lasty = lidarPointPossiblePointDoll[0].f32y;
    //tFloat32 f32DistanceLastPointToCurrentPoint = 999.9;
    ///hier stimmt was nicht durcheinander von laser vector 1 und vector2
    for(tUInt32 i = (coorDollFinal[0].ui32Counter-1); i>0; i--)
    {
        if(lasersample.tScanArray[i].f32Radius != 0.0)
        {
            tFloat32 f32yLeft = lasersample.tScanArray[i].f32Radius * sinf((90-lasersample.tScanArray[i].f32Angle)*tFloat32(PI)/180.0f);
            tFloat32 f32xLeft = lasersample.tScanArray[i].f32Radius * cosf((90-lasersample.tScanArray[i].f32Angle)*tFloat32(PI)/180.0f);
            tFloat32 f32LastPointToCurrentPoint = sqrt((pow((f32Lastx-f32xLeft), 2))+
                    pow((f32Lasty-f32yLeft), 2));
            tFloat32 f32DistanceLastPointToLastxy = sqrt((pow((lidarPointPossiblePointDoll[lidarPointPossiblePointDoll.size()-1].f32x-f32Lastx), 2))+
                    pow((lidarPointPossiblePointDoll[lidarPointPossiblePointDoll.size()-1].f32y-f32yLeft), 2));
            if(f32LastPointToCurrentPoint > m_propF32ThresholdDollAnotherObject)
            {
                //check total size -> too big false; true continue
                if(f32DistanceLastValidPointToLastxy > m_propF32AdultDollWidth)
                {
                    if(m_propBDebugLidar)LOG_INFO("false object close to the left side ; Distance: %f", f32DistanceLastPointToLastxy);
                    return false;
                }
                break;

            }
            f32DistanceLastValidPointToLastxy = f32DistanceLastPointToLastxy;
            f32Lastx = f32xLeft;
            f32Lasty = f32yLeft;
        }
    }
    if(f32DistanceLastValidPointToLastxy > m_propF32AdultDollWidth)
    {
        if(m_propBDebugLidar)LOG_INFO("false; object is too big");
        return tFalse;
    }

    /*check if the last point belongs to an object that is also outside the roi*/
    /*right hand side*/
    tFloat32 f32FirstPointToLastPoint = 0.0;
    tFloat32 f32Lastxright = lidarPointPossiblePointDoll[coorDollFinal.size()-1].f32x;
    tFloat32 f32Lastyright = lidarPointPossiblePointDoll[coorDollFinal.size()-1].f32y;
    for(tUInt32 i = coorDollFinal[coorDollFinal.size()-1].ui32Counter; i<lasersample.ui32Size; i++)
    {
        if(lasersample.tScanArray[i].f32Radius != 0.0)
        {
            tFloat32 f32yRight = lasersample.tScanArray[i].f32Radius * sinf((90-lasersample.tScanArray[i].f32Angle)*tFloat32(PI)/180.0f);
            tFloat32 f32xRight = lasersample.tScanArray[i].f32Radius * cosf((90-lasersample.tScanArray[i].f32Angle)*tFloat32(PI)/180.0f);
            tFloat32 f32DistanceLastPointToCurrentPoint = sqrt((pow((f32Lastxright-f32xRight), 2))+
                    pow((f32Lastyright-f32yRight), 2));

            if(f32DistanceLastPointToCurrentPoint > m_propF32ThresholdDollAnotherObject)
            {

                if(f32FirstPointToLastPoint > m_propF32AdultDollWidth)
                {
                    if(m_propBDebugLidar)LOG_INFO("false, too big: %f", f32FirstPointToLastPoint);
                    return tFalse;
                }
                break;
            }
            f32Lastxright = f32xRight;
            f32Lastyright = f32yRight;
            f32FirstPointToLastPoint = sqrt((pow((lidarPointPossiblePointDoll[0].f32x-f32Lastxright), 2))+
                    pow((lidarPointPossiblePointDoll[0].f32y-f32Lastyright), 2));
        }
    }
    if(f32FirstPointToLastPoint > m_propF32AdultDollWidth)
    {
        if(m_propBDebugLidar)LOG_INFO("false; too big: %f", f32FirstPointToLastPoint);
        return tFalse;
    }

    tFloat32 f32DistanceBeginToEnd = sqrt((pow((coorDollFinal[coorDollFinal.size()-1].f32x-coorDollFinal[0].f32x), 2))+
            pow((coorDollFinal[coorDollFinal.size()-1].f32y-coorDollFinal[0].f32y), 2));
    if(f32DistanceBeginToEnd < m_propF32AdultDollWidth)
    {
        if(m_propBDebugLidar)LOG_INFO("true, out of the loop! Distance: %f", f32DistanceBeginToEnd);
        return tTrue;
    }
    else
    {
        if(m_propBDebugLidar)LOG_INFO("FALSE, object is too long: %f", f32DistanceBeginToEnd);
        return tFalse;
    }

}

//to compensate the inconsistante angle range-> output car right 90° to car left -90°
tFloat32 ObjectSpecifier::AngleCompensation(tFloat32 angle)
{
    if (angle >= 0.0 && angle <= 90)
    {
        return angle;
    }
    else if(angle >= 270 && angle <= 360)
    {
        return (angle -360);
    }
    else
    {
        LOG_ERROR("Angle out of range!");
        return 100000;//Error value but without error handeling!
    }
}

//Transmit feedback
tResult ObjectSpecifier::TransmitFeedbackAdultDollDetected()
{
    //boost::lock_guard<boost::mutex> lock(criticalSection_TransmitFeedback);
    TFeedbackStruct::Data feedbackDollAdultDetetcted;
    feedbackDollAdultDetetcted.ui32FeedbackStatus = FB_OS_DETECTED_ADULT;
    feedbackDollAdultDetetcted.ui8FilterId = F_OBJECT_SPECIFIER;
    RETURN_IF_FAILED(o_FeedbackStruct.writePin(m_WriterFeedback, (void *) &feedbackDollAdultDetetcted, m_pClock->GetStreamTime()));
    if(m_propBDebugLidar)LOG_INFO("TransmitFeedbackAdultDollDetected()");
    RETURN_NOERROR;
}

tResult ObjectSpecifier::TransmitFeedbackChildDollDetected()
{
    boost::lock_guard<boost::mutex> lock(criticalSection_TransmitFeedback);
    TFeedbackStruct::Data feedbackDollChildDetetcted;
    feedbackDollChildDetetcted.ui32FeedbackStatus = FB_OS_DETECTED_CHILD;
    feedbackDollChildDetetcted.ui8FilterId = F_OBJECT_SPECIFIER;
    RETURN_IF_FAILED(o_FeedbackStruct.writePin(m_WriterFeedback, (void *) &feedbackDollChildDetetcted, m_pClock->GetStreamTime()));
    if(m_propBDebugLidar)LOG_INFO("TransmitFeedbackChildDollDetected()");
    RETURN_NOERROR;
}
