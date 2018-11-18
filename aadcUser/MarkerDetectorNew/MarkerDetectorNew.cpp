/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*********************************************
* Author: Xiangfei
* Comment: tvecs: x: right y: down z: front
**********************************************************************/

#include "stdafx.h"
#include "MarkerDetectorNew.h"
#include "ADTF3_OpenCV_helper.h"
#include "aadc_roadSign_enums.h"
#include <property_structs.h>


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_MARKERDETECTORNEW_FILTER,
                                    "MarkerDetectorNew",
                                    MarkerDetectorNew,
                                    adtf::filter::pin_trigger({ "input" }));


MarkerDetectorNew::MarkerDetectorNew()
{
    //Register Properties
    RegisterPropertyVariable("Calibration File", m_calibFile);
    RegisterPropertyVariable("Detector Parameter File", m_fileDetectorParameter);
    RegisterPropertyVariable("Marker Size [m]", m_f32MarkerSize);

    // XY: new property variables
    RegisterPropertyVariable("ROI::XOffset", m_propI32ROIOffsetX);

    RegisterPropertyVariable("ROI::YOffset", m_propI32ROIOffsetY);
    RegisterPropertyVariable("ROI::Width", m_propI32ROIWidth);
    RegisterPropertyVariable("Roi::height", m_propI32ROIHeight);
    RegisterPropertyVariable("Marker Roi::maximum distance", m_propF32MaxDist);
    RegisterPropertyVariable("Marker Roi::dropped frames", m_propI32DropFrame);



    RegisterPropertyVariable("z axis calibration offset", m_propF32CalibZOffset);
    RegisterPropertyVariable("z axis calibration slope", m_propF32CalibZSlope);


    //create and set inital input format type
    m_sInputFormat.m_strFormatName = ADTF_IMAGE_FORMAT(RGB_24);
    adtf::ucom::object_ptr<IStreamType> pType = adtf::ucom::make_object_ptr<cStreamType>(stream_meta_type_image());
    set_stream_type_image_format(*pType, m_sInputFormat);


    //register input pin
    Register(m_oReader, "input", pType);
    //register output pin
    Register(m_oImagePinWriter, "output", pType);

    //XY: register new pins
    o_TRoadSignExt.registerPin(this     , m_WriterRoadSignExt   , "road_sign_ext"   );
    o_TActionStruct.registerPin(this    , m_ReaderAction        , "actionInput"         );
    o_TFeedbackStruct.registerPin(this  , m_WriterFeedback      , "feedbackOutput"      );
    o_LaneDetectionLineStruct.registerPin(this, m_ReaderLaneDetectionLine             , "laneDetectionLine"                  );

    //register callback for type changes
    m_oReader.SetAcceptTypeCallback([this](const adtf::ucom::ant::iobject_ptr<const adtf::streaming::ant::IStreamType>& pType) -> tResult
    {
        return ChangeType(m_oReader, m_sInputFormat, *pType.Get(), m_oImagePinWriter);
    });
}

tResult MarkerDetectorNew::Configure()
{
    m_propIntFilterCount = 8;
    //init timestamps
    m_ui32TimeStampAction = 0;
    m_bSpecialCase = tFalse;

    m_bSpecialCasePed = tFalse;
    m_bCheckSpecialCase = tFalse;
    // Read Camera Calibration File
    cFilename fileCalibration = m_calibFile;
    adtf::services::ant::adtf_resolve_macros(fileCalibration);
    //check if calibration file with camera paramters exits
    if (fileCalibration.IsEmpty())
    {
        RETURN_ERROR_DESC(ERR_INVALID_FILE, "Calibration File for camera not found");
    }
    if (!(cFileSystem::Exists(fileCalibration)))
    {
        RETURN_ERROR_DESC(ERR_INVALID_FILE, "Calibration File for camera not found");
    }
    else
    {
        // read the calibration file with camera paramters exits and save to member variable
        readCameraParameters(fileCalibration.GetPtr(), m_matIntrinsics, m_matDistorsion);
        m_bCamaraParamsLoaded = tTrue;
    }


    //Aruco
    //create the detector params
    cFilename fileDetectorParameter = m_fileDetectorParameter;
    m_detectorParams = aruco::DetectorParameters::create();
    adtf::services::ant::adtf_resolve_macros(fileDetectorParameter);
    if (!(readDetectorParameters(fileDetectorParameter.GetPtr(), m_detectorParams)))
    {
        RETURN_ERROR_DESC(ERR_INVALID_FILE, "Detector Parameter file not valid");
    }

    //set marker dictionary
    m_Dictionary = aruco::getPredefinedDictionary(aruco::DICT_5X5_50);

    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    // XY: set variables
    m_iFrameCount = 0;

    LoadProperties();

    RETURN_NOERROR;
}

tResult MarkerDetectorNew::Process(tTimeStamp )
{

    // ************************************ //
    //             Video Input              //
    // ************************************ //

    object_ptr<const ISample> pReadSample;

    if (IS_OK(m_oReader.GetNextSample(pReadSample)))
    {
        RETURN_IF_FAILED(ProcessVideo(pReadSample));

    }





    // ************************************ //
    //              Lane Input              //
    // ************************************ //


    TLaneDetectionLineStruct::Data tmp_laneDetectionLine;
    static tTimeStamp lasttmldl = 0;
    if(IS_OK(o_LaneDetectionLineStruct.readPin(m_ReaderLaneDetectionLine, (void *) & tmp_laneDetectionLine, lasttmldl)))
    {
        lasttmldl = tmp_laneDetectionLine.ui32ArduinoTimestamp;
        m_dataLastLane = tmp_laneDetectionLine;
        m_i32IsCurve.push_back(m_dataLastLane.i8IsCurve);

        // remove first element in lists when count is reached
        if ((int)  m_i32IsCurve.size() > m_propIntFilterCount)
        {

            m_i32IsCurve.pop_front();
        }


    }


    // ************************************ //
    //            Action Input              //
    // ************************************ //



    TActionStruct::Data tmpAction;
    RETURN_IF_FAILED(o_TActionStruct.readPin(m_ReaderAction, (void *) & tmpAction, m_ui32TimeStampAction));
    m_ui32TimeStampAction = tmpAction.ui32ArduinoTimestamp;


    if(tmpAction.ui32Command == 72) // normal case
    {
        LoadProperties();


    }

    if(tmpAction.ui32Command == 3100) // normal case
    {
        m_dataActionIn = tmpAction;

    }
    else if(tmpAction.ui32Command == 3150) // start to check special case after left turn
    {
        m_bCheckSpecialCase = tTrue;
        m_bSpecialCase = tFalse;
        m_dataActionCheckSpecialCaseIn = tmpAction;

    }
    else if(tmpAction.ui32Command == 3152) // start to check special case after right turn
    {
        m_bCheckSpecialCase = tTrue;
        m_bSpecialCase = tFalse;
        m_dataActionCheckSpecialCaseIn = tmpAction;

    }
    else if(tmpAction.ui32Command == 3154) // end of checking special case
    {
        m_dataActionCheckSpecialCaseIn = tmpAction;

        boost::lock_guard<boost::mutex> lockFeedback(m_mutexFeedback);
        TFeedbackStruct::Data tmp_feedback;
        tmp_feedback.ui8FilterId = F_ROAD_SIGNS;
        if(m_bSpecialCase == tTrue)
        {
            tmp_feedback.ui32FeedbackStatus = FB_RS_CROSSING_SIGN_SPECIAL_CASE;
            LOG_ERROR("special case!");
        }
        else
        {
            tmp_feedback.ui32FeedbackStatus = FB_RS_CROSSING_SIGN_NORMAL_CASE;
            LOG_ERROR("normal case!");
        }
        m_bSpecialCase = tFalse;

        m_dataActionCheckSpecialCaseIn.ui32Command = AC_RS_CHECK_CROSSING_SIGN_RESET; //

        RETURN_IF_FAILED(o_TFeedbackStruct.writePin(m_WriterFeedback, (void *) &tmp_feedback, m_pClock->GetStreamTime()));
    }
    else if(tmpAction.ui32Command == 3180) // check if ped direct after left/right turn
    {
        boost::lock_guard<boost::mutex> lockFeedback(m_mutexFeedback);
            TFeedbackStruct::Data tmp_feedback;
            tmp_feedback.ui8FilterId = F_ROAD_SIGNS;
            if(m_bSpecialCasePed == tTrue)
            {
                tmp_feedback.ui32FeedbackStatus = FB_RS_PED_SIGN_SPECIAL_CASE;
                LOG_ERROR("special case Ped!");
            }
            else
            {
                tmp_feedback.ui32FeedbackStatus = FB_RS_PED_SIGN_NORMAL_CASE;
                LOG_ERROR("normal case, no Ped!");
            }
            m_bSpecialCasePed = tFalse;
            RETURN_IF_FAILED(o_TFeedbackStruct.writePin(m_WriterFeedback, (void *) &tmp_feedback, m_pClock->GetStreamTime()));

    }
    else if(tmpAction.ui32Command == 3158) // get special sign type
    {

        RETURN_IF_FAILED(GetSpecialSign());
    }
    else if(tmpAction.ui32Command == 3170) // get special sign type
    {


        // calc mean values for all sensors and replace old values in received data
        // Side Left average
        int sum = 0;
        for (std::list<int>::iterator p = m_i32IsCurve.begin(); p != m_i32IsCurve.end(); ++p)
        {
            sum += (float)*p;
        }

        if(! m_i32IsCurve.empty())
        {
             m_i32IsCurve.clear();
        }

         TFeedbackStruct::Data tmp_feedback;
        tmp_feedback.ui8FilterId = F_ROAD_SIGNS;
        LOG_INFO("in, sum %d", sum);
        if (sum == -m_propIntFilterCount) // left curve
        {
            boost::lock_guard<boost::mutex> lockFeedback(m_mutexFeedback);
            tmp_feedback.ui32FeedbackStatus = FB_RS_LEFT_CURVE_SPECIAL_CASE;
            LOG_ERROR("left curve! %d", sum);
            RETURN_IF_FAILED(o_TFeedbackStruct.writePin(m_WriterFeedback, (void *) &tmp_feedback, m_pClock->GetStreamTime()));
            LOG_INFO("fb should be sent %d", tmp_feedback.ui32FeedbackStatus);

        }
        else if(sum == m_propIntFilterCount) // right curve
        {

            boost::lock_guard<boost::mutex> lockFeedback(m_mutexFeedback);
            tmp_feedback.ui32FeedbackStatus = FB_RS_RIGHT_CURVE_SPECIAL_CASE;
            LOG_ERROR("right curve! %d", sum);
            RETURN_IF_FAILED(o_TFeedbackStruct.writePin(m_WriterFeedback, (void *) &tmp_feedback, m_pClock->GetStreamTime()));
            LOG_INFO("fb should be sent %d", tmp_feedback.ui32FeedbackStatus);
        }
        else
        {
            boost::lock_guard<boost::mutex> lockFeedback(m_mutexFeedback);
            tmp_feedback.ui32FeedbackStatus = FB_RS_RIGHT_CURVE_NORMAL_CASE;
            LOG_ERROR("no curve or nix detected! %d", sum);
            RETURN_IF_FAILED(o_TFeedbackStruct.writePin(m_WriterFeedback, (void *) &tmp_feedback, m_pClock->GetStreamTime()));
            LOG_INFO("fb should be sent %d", tmp_feedback.ui32FeedbackStatus);
        }
    }



    RETURN_NOERROR;
}

tResult MarkerDetectorNew::LoadProperties()
{


    //TEST CASE
    LOG_SUCCESS("here in load properties");
    markerdetector_properties properties;
    LoadXMLData("/home/aadc/AADC/utilities/propertyXMLs/markerdetector_properties.xml", (void*) (&properties));

    m_f32MarkerSize = properties.m_f32MarkerSize;


    m_propF32CalibZOffset = properties.m_propF32CalibZOffset;
    m_propF32CalibZSlope = properties.m_propF32CalibZSlope;
    m_propF32MaxDist = properties.m_propF32MaxDist;
    m_propI32ROIOffsetX = properties.m_propI32ROIOffsetX;
    m_propI32ROIOffsetY = properties.m_propI32ROIOffsetY;

    m_propI32ROIWidth = properties.m_propI32ROIWidth;
    m_propI32ROIWidthCurve = properties.m_propI32ROIWidthCurve;
    m_propI32ROIHeight = properties.m_propI32ROIHeight;
    m_propI32DropFrame = properties.m_propI32DropFrame;

    LOG_SUCCESS(cString::Format("m_f32MarkerSize: %f", properties.m_f32MarkerSize));


    LOG_SUCCESS(cString::Format("m_propF32CalibZOffset: %f", (float)m_propF32CalibZOffset));
    LOG_SUCCESS(cString::Format("m_propF32CalibZSlope: %f", (float)m_propF32CalibZSlope));
    LOG_SUCCESS(cString::Format("m_propF32MaxDist: %f", (float)m_propF32MaxDist));
    LOG_SUCCESS(cString::Format("m_propI32ROIOffsetX: %d", (int)m_propI32ROIOffsetX));
    LOG_SUCCESS(cString::Format("m_propI32ROIOffsetY: %d", (int)m_propI32ROIOffsetY));
    LOG_SUCCESS(cString::Format("m_propI32ROIWidth: %d", (int)m_propI32ROIWidth));
    LOG_SUCCESS(cString::Format("m_propI32ROIWidthCurve: %d", (int)m_propI32ROIWidthCurve));
    LOG_SUCCESS(cString::Format("m_propI32ROIHeight: %d", (int)m_propI32ROIHeight));
    LOG_SUCCESS(cString::Format("m_propI32DropFrame: %d", (int)m_propI32DropFrame));

    RETURN_NOERROR;


}

tResult MarkerDetectorNew::ProcessVideo(object_ptr<const ISample> pReadSample)
{

    m_iFrameCount++;

    //Only process every 10 frame

    if(m_iFrameCount < m_propI32DropFrame)
    {

        RETURN_NOERROR;
    }
    else
    {
        m_iFrameCount = 0;
    }

    Mat outputImage;


    vector< Vec3d > rvecs, tvecs;
    vector< int > ids;
    object_ptr_shared_locked<const ISampleBuffer> pReadBuffer;
    // the results from aruco detection
    vector< vector< Point2f > > corners, rejected;

    //lock read buffer
    if (IS_OK(pReadSample->Lock(pReadBuffer)))
    {
        //create a opencv matrix from the media sample buffer
        Mat m_inputImage(cv::Size(m_sInputFormat.m_ui32Width, m_sInputFormat.m_ui32Height),
                         CV_8UC3, (uchar*)pReadBuffer->GetPtr());


        aruco::detectMarkers(m_inputImage, m_Dictionary, corners, ids, m_detectorParams, rejected);


        // if we have the camera pararmeter available we calculate the pose
        if (m_bCamaraParamsLoaded && ids.size() > 0)
            aruco::estimatePoseSingleMarkers(corners, m_f32MarkerSize, m_matIntrinsics, m_matDistorsion, rvecs,
                                             tvecs);

        outputImage = m_inputImage.clone();
    }
    else
    {

        RETURN_NOERROR;
    }

    // draw detections

    aruco::drawDetectedMarkers(outputImage, corners, ids);

    if (m_bCamaraParamsLoaded  && ids.size() > 0)
    {
        for (unsigned int i = 0; i < ids.size(); i++)
        {
            aruco::drawAxis(outputImage, m_matIntrinsics, m_matDistorsion, rvecs[i], tvecs[i],
                            m_f32MarkerSize * 0.5f);
        }
    }


    //write WITH markers to output pin
    if (!outputImage.empty())
    {
        rectangle(outputImage, Rect(m_propI32ROIOffsetX, m_propI32ROIOffsetY,
                                    m_propI32ROIWidth,m_propI32ROIHeight), cv::Scalar(0, 255, 0), 10, CV_AA);
        rectangle(outputImage, Rect(m_propI32ROIOffsetX, m_propI32ROIOffsetY,
                                    m_propI32ROIWidthCurve,m_propI32ROIHeight), cv::Scalar(255, 0, 0), 10, CV_AA);

        //update output format if matrix size does not fit to
        if (outputImage.total() * outputImage.elemSize() != m_sInputFormat.m_szMaxByteSize)
        {
            setTypeFromMat(m_oImagePinWriter, outputImage);
        }
        // write to pin
        writeMatToPin(m_oImagePinWriter, outputImage, m_pClock->GetStreamTime());
    }

    //print marker info and draw the markers in image
    for (unsigned int i = 0; i < ids.size(); i++)
    {

        // z axis calibration

        tvecs[i][2] = (tvecs[i][2] + m_propF32CalibZOffset)* m_propF32CalibZSlope;

        //LOG_INFO("id %d, x %f y %f, z %f, roll %f, pitch %f, yaw %f ", ids[i],  tvecs[i][0],tvecs[i][1],tvecs[i][2], rvecs[i][0],rvecs[i][1],rvecs[i][2]  );

        // **************************************************//
        //  check if special case, check all detected signs  //
        // **************************************************//
        if ( ids[i] == roadsignIDs::MARKER_ID_UNMARKEDINTERSECTION
             || ids[i] == roadsignIDs::MARKER_ID_STOPANDGIVEWAY
             || ids[i] == roadsignIDs::MARKER_ID_GIVEWAY
             || ids[i] == roadsignIDs::MARKER_ID_HAVEWAY
             || ids[i] == roadsignIDs::MARKER_ID_PEDESTRIANCROSSING)
        {
            if (m_dataActionCheckSpecialCaseIn.ui32Command == AC_RS_CHECK_CROSSING_SIGN_RIGHT_START) // check special case after a right turn
            {
                if (tvecs[i][0] < 1.1f && tvecs[i][0] > 0.55f && tvecs[i][2] < 1.1f && tvecs[i][2] > 0.5f )
                {
                    if ( ids[i] == roadsignIDs::MARKER_ID_PEDESTRIANCROSSING)
                    {
                        m_bSpecialCasePed = tTrue;
                        m_dataSpecialSign.i16Identifier = ids[i];
                    }
                    else
                    {
                        m_bSpecialCase = tTrue;
                        m_dataSpecialSign.i16Identifier = ids[i];
                    }
                    //                for(unsigned int j = 0; j< 3;j++)
                    //                {
                    //                    m_dataSpecialSign.af32TVec[j] = tvecs[i][j];
                    //                    m_dataSpecialSign.af32RVec[j] = rvecs[i][j];
                    //                }
                    //                m_dataSpecialSign.ui32ArduinoTimestamp = m_pClock->GetStreamTime();
                }

            }
            else if (m_dataActionCheckSpecialCaseIn.ui32Command == AC_RS_CHECK_CROSSING_SIGN_LEFT_START) // check special case after a left turn
            {
                if (corners.at(i).at(0).x > m_propI32ROIOffsetX
                        && corners.at(i).at(0).x
                        < m_propI32ROIOffsetX
                        + m_propI32ROIWidth
                        && corners.at(i).at(0).y > m_propI32ROIOffsetY
                        && corners.at(i).at(0).y
                        < m_propI32ROIOffsetY
                        + m_propI32ROIHeight
                        &&  tvecs[i][0]  < 0.6f && sqrt(tvecs[i][0]*tvecs[i][0]+tvecs[i][2] * tvecs[i][2]) < m_propF32MaxDist)
                {
                    if ( ids[i] == roadsignIDs::MARKER_ID_PEDESTRIANCROSSING)
                    {
                        m_bSpecialCasePed = tTrue;
                        m_dataSpecialSign.i16Identifier = ids[i];
                    }
                    else
                    {
                        m_bSpecialCase = tTrue;
                        m_dataSpecialSign.i16Identifier = ids[i];
                    }
                }

            }
        }




        // **************************************************//
        //                    normal case                    //
        // **************************************************//

        if (corners.at(i).at(0).x > m_propI32ROIOffsetX
                && corners.at(i).at(0).x
                < m_propI32ROIOffsetX
                + m_propI32ROIWidth
                && corners.at(i).at(0).y > m_propI32ROIOffsetY
                && corners.at(i).at(0).y
                < m_propI32ROIOffsetY
                + m_propI32ROIHeight
                && sqrt(tvecs[i][0]*tvecs[i][0]+tvecs[i][2] * tvecs[i][2]) < m_propF32MaxDist)
        {



            // call the function to transmit a extended road sign sample with the detected marker if the Tvec in the marker was correctly set
            if (m_bCamaraParamsLoaded)
            {
                m_dataRoadSignExtOut.i16Identifier = ids[i];

                for(unsigned int j = 0; j< 3;j++)
                {
                    m_dataRoadSignExtOut.af32TVec[j] = tvecs[i][j];
                    m_dataRoadSignExtOut.af32RVec[j] = rvecs[i][j];
                }
                m_dataRoadSignExtOut.ui32ArduinoTimestamp = m_pClock->GetStreamTime();



                //RETURN_IF_FAILED(o_TRoadSignExt.writePin(m_WriterRoadSignExt, (void *) &m_dataRoadSignExtOut, m_pClock->GetStreamTime())); // only for testing, later delete this plz
                RETURN_IF_FAILED(o_TRoadSignExt.writePin(m_WriterRoadSignExt, (void *) &m_dataRoadSignExtOut, m_pClock->GetStreamTime()));


                if(m_dataActionIn.ui32Command == AC_RS_START)
                {


                    // replace Roundabout sign with giveway sign
                    if(ids[i] == roadsignIDs::MARKER_ID_ROUNDABOUT)
                    {
                        ids[i] = roadsignIDs::MARKER_ID_GIVEWAY;
                    }

                    boost::lock_guard<boost::mutex> lockFeedback(m_mutexFeedback);
                    switch (ids[i])
                    {

                    //update
                    case roadsignIDs::MARKER_ID_UNMARKEDINTERSECTION:
                        m_dataLastPrioritySignOut.ui8FilterId = F_ROAD_SIGNS;
                        m_dataLastPrioritySignOut.ui32FeedbackStatus = FB_RS_UNMARKEDINTERSECTION;
                        RETURN_IF_FAILED(o_TFeedbackStruct.writePin(m_WriterFeedback, (void *) &m_dataLastPrioritySignOut, m_pClock->GetStreamTime()));
                        m_dataActionIn.ui32Command = 3999;

                        break;
                    case roadsignIDs::MARKER_ID_STOPANDGIVEWAY:
                        m_dataLastPrioritySignOut.ui8FilterId = F_ROAD_SIGNS;
                        m_dataLastPrioritySignOut.ui32FeedbackStatus = FB_RS_STOPANDGIVEWAY;
                        RETURN_IF_FAILED(o_TFeedbackStruct.writePin(m_WriterFeedback, (void *) &m_dataLastPrioritySignOut, m_pClock->GetStreamTime()));

                        m_dataActionIn.ui32Command = 3999;
                        break;
                    case roadsignIDs::MARKER_ID_GIVEWAY:
                    {
                        m_dataLastPrioritySignOut.ui8FilterId = F_ROAD_SIGNS;
                        m_dataLastPrioritySignOut.ui32FeedbackStatus = FB_RS_GIVEWAY;
                        RETURN_IF_FAILED(o_TFeedbackStruct.writePin(m_WriterFeedback, (void *) &m_dataLastPrioritySignOut, m_pClock->GetStreamTime()));

                        m_dataActionIn.ui32Command = 3999;
                        break;
                    }
                    case roadsignIDs::MARKER_ID_HAVEWAY:
                    {
                        m_dataLastPrioritySignOut.ui8FilterId = F_ROAD_SIGNS;
                        m_dataLastPrioritySignOut.ui32FeedbackStatus = FB_RS_HAVEWAY;
                        RETURN_IF_FAILED(o_TFeedbackStruct.writePin(m_WriterFeedback, (void *) &m_dataLastPrioritySignOut, m_pClock->GetStreamTime()));

                        m_dataActionIn.ui32Command = 3999;
                        break;
                    }
                    case roadsignIDs::MARKER_ID_PEDESTRIANCROSSING:
                    {
                        if (tvecs[i][2] < 0.45){

                        m_dataLastPrioritySignOut.ui8FilterId = F_ROAD_SIGNS;
                        m_dataLastPrioritySignOut.ui32FeedbackStatus = FB_RS_PEDESTRIANCROSSING;
                        RETURN_IF_FAILED(o_TFeedbackStruct.writePin(m_WriterFeedback, (void *) &m_dataLastPrioritySignOut, m_pClock->GetStreamTime()));

                        m_dataActionIn.ui32Command = 3999;
                        }
                        break;
                    }
                    case roadsignIDs::MARKER_ID_PARKINGAREA:
                    {
                        m_bParkingSignReceived = tTrue; // set parking sign received
                        TFeedbackStruct::Data response;
                        response.ui8FilterId = F_ROAD_SIGNS;
                        response.ui32FeedbackStatus = 3002;

                        RETURN_IF_FAILED(o_TFeedbackStruct.writePin(m_WriterFeedback, (void *) &response, m_pClock->GetStreamTime()));

                        m_dataActionIn.ui32Command = 3999;
                        break;
                    }
                    }
                }


            }
        }
    }



    RETURN_NOERROR;

}


tResult MarkerDetectorNew::ProcessActionInput(TActionStruct::Data )
{

    RETURN_NOERROR;

}


tResult MarkerDetectorNew::GetSpecialSign(){


    boost::lock_guard<boost::mutex> lockFeedback(m_mutexFeedback);
    TFeedbackStruct::Data tmp_feedback;

    switch (m_dataSpecialSign.i16Identifier)
    {
    //update
    case roadsignIDs::MARKER_ID_UNMARKEDINTERSECTION:
        tmp_feedback.ui8FilterId = F_ROAD_SIGNS;
        tmp_feedback.ui32FeedbackStatus = FB_RS_UNMARKEDINTERSECTION;
        RETURN_IF_FAILED(o_TFeedbackStruct.writePin(m_WriterFeedback, (void *) &tmp_feedback, m_pClock->GetStreamTime()));

        break;
    case roadsignIDs::MARKER_ID_STOPANDGIVEWAY:
        tmp_feedback.ui8FilterId = F_ROAD_SIGNS;
        tmp_feedback.ui32FeedbackStatus = FB_RS_STOPANDGIVEWAY;
        RETURN_IF_FAILED(o_TFeedbackStruct.writePin(m_WriterFeedback, (void *) &tmp_feedback, m_pClock->GetStreamTime()));
        break;
    case roadsignIDs::MARKER_ID_GIVEWAY:
    {
        tmp_feedback.ui8FilterId = F_ROAD_SIGNS;
        tmp_feedback.ui32FeedbackStatus = FB_RS_GIVEWAY;
        RETURN_IF_FAILED(o_TFeedbackStruct.writePin(m_WriterFeedback, (void *) &tmp_feedback, m_pClock->GetStreamTime()));
        break;
    }
    case roadsignIDs::MARKER_ID_HAVEWAY:
    {
        tmp_feedback.ui8FilterId = F_ROAD_SIGNS;
        tmp_feedback.ui32FeedbackStatus = FB_RS_HAVEWAY;
        RETURN_IF_FAILED(o_TFeedbackStruct.writePin(m_WriterFeedback, (void *) &tmp_feedback, m_pClock->GetStreamTime()));
        break;
    }
    case roadsignIDs::MARKER_ID_PEDESTRIANCROSSING:
    {
        tmp_feedback.ui8FilterId = F_ROAD_SIGNS;
        tmp_feedback.ui32FeedbackStatus = FB_RS_PEDESTRIANCROSSING;
        RETURN_IF_FAILED(o_TFeedbackStruct.writePin(m_WriterFeedback, (void *) &tmp_feedback, m_pClock->GetStreamTime()));
        break;
    }
    default: {
        LOG_ERROR("no proper sign detected, id: %d", m_dataSpecialSign.i16Identifier);
    }
        m_dataSpecialSign.i16Identifier = -1;
    }
    RETURN_NOERROR;
}
