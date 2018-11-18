/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************/

#include <mutex>
#include "stdafx.h"
#include "IntersectionDetectorFilterNew.h"
#include "ScmCommunication.h"

#include <cmath>
#include "aadc_roadSign_enums.h"
#include "opencv2/core/types.hpp"

#include "CVMath.h"
#include <property_structs.h>

#define PI  3.14159265

#define ISD_STOPPOINT_OFFSET_X "stoppoint::offset_x"
#define ISD_STOPPOINT_OFFSET_Y "stoppoint::offset_y"
#define ISD_CAMERA_OFFSET_X "camera::offset_x"
#define ISD_LASERSCANNER_OFFSET_X "LaserScanner::offset_x"
#define ISD_CAMERA_OFFSET_Y "camera::offset_y"
#define ISD_MINIMUM_SIGN_DISTANCE "minimum_sign_distance"
#define ISD_SAMPLE_COUNT "sample_count"
#define ISD_DEBUG_COMMAND "debug_command"
#define ISD_DEBUG_ENABLE "enable_debug"

// This will define the filter and expose it via plugin class factory.
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(
        CID_INTERSECTION_DETECTOR_NEW_FILTER_FILTER,                         // references to header file
        "IntersectionDetectorFilterNew",                                   // label
        IntersectionDetectorFilterNew,                                      // class
        //        adtf::filter::pin_trigger({ "laserScannerInput" }));	// set trigger pin
        adtf::filter::timer_trigger(2));

// -------------------------------
// --------- CONSTRUCTOR ---------
// -------------------------------
// in the constructor, we will set the adtf streamtypes for signal values and pins
IntersectionDetectorFilterNew::IntersectionDetectorFilterNew()
{
    // ------------------------------------------
    SetName("IntersectionDetectorFilterNew Constructor");

    // -----------------------------------------
    // set pins
    o_TRoadSignExt.registerPin(this, m_ReaderRoadSignExt        , "RoadSignExtInput"    );
    o_TActionStruct.registerPin(this, m_ReaderAction            , "actionInput"         );
    o_TLaserScannerData.registerPin(this , m_ReaderLaserScanner , "laserScannerInput"   );

    o_TFeedbackStruct.registerPin(this, m_WriterFeedback        , "feedbackOutput"      );
    o_TPoseStruct.registerPin(this, m_WriterLocalGoal           , "localGoalOutput"     );
    // -----------------------------------------
    // set property variables
    // set property variables
    RegisterPropertyVariable("StopLineToMarkerX"        , m_propF32XOffset              );
    RegisterPropertyVariable("DistStartSignDetection"          , m_propF32DistStartSignDetection      );
    RegisterPropertyVariable("DistCameraToLS"           , m_propF32DistCameraLS         );
    RegisterPropertyVariable("param for adjustment"     , m_propF32AdjustmentFactor     );
    RegisterPropertyVariable("DistStartLaserScanner"    , m_propF32DistStartLS          );
    RegisterPropertyVariable("MinLaserScannerAngle"     , m_propF32MinAngle             );
    RegisterPropertyVariable("MaxLaserScannerAngle"     , m_propF32MaxAngle             );
    RegisterPropertyVariable("RangeForPoseDetection"    , m_propF32RadiusRange          );
    RegisterPropertyVariable("DistFactor"    , m_propF32DistFaktor          );

    RegisterPropertyVariable("SampleCountLS"            , m_propISampleCount            );
    RegisterPropertyVariable("XMAX"            , m_propF32XMax            );
    RegisterPropertyVariable("XMIN"            , m_propF32XMin            );
    RegisterPropertyVariable("YMAX"            , m_propF32YMax            );
    RegisterPropertyVariable("YMIN"            , m_propF32YMin            );

    RegisterPropertyVariable("Width of the sign"  , m_propF32WidthSign            );
    RegisterPropertyVariable("Threshold x: difference not detected as sign"  , m_propF32ThresholdXNoSign            );

}


// ------------------------------
// --------- CONFIGURE ----------
// ------------------------------
// implement configure() to read ALL properties
tResult IntersectionDetectorFilterNew::Configure()
{

    m_ui32TimestampRoadSign = 0;
    m_ui32TimestampAction = 0;
    m_ui32TimestampPose = 0;
    SetSignDetected(tFalse);
    //m_bSignDetected = tFalse;
    m_bLSStarted = tFalse;

    SetCurrentActionIn(AC_ISD_STOP);
    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

    LoadProperties();
    // done
    RETURN_NOERROR;

}


// ------------------------------
// ---------- PROCESS -----------
// ------------------------------
// process() is always called each time a trigger occured (new data sample arrive).
// IMPORTANT: Make sure to read ALL samples of ALL readers!
tResult IntersectionDetectorFilterNew::Process(tTimeStamp )
{
    // if new data is action signal
    TActionStruct::Data tmp_action;
    if(IS_OK(o_TActionStruct.readPin(m_ReaderAction, (void *) & tmp_action, m_ui32TimestampAction)))
    {

        // timestamps
        if (tmp_action.ui8FilterId == F_INTERSECTION_DETECTOR){
            SetCurrentActionIn(tmp_action.ui32Command);
            m_ui32TimestampAction = tmp_action.ui32ArduinoTimestamp;
            //m_dataCurrentActionIn = tmp_action;
        }
    }

    // if new data is road sign ext signal
    TRoadSignExt::Data inputRoadSignExt;
    if(IS_OK(o_TRoadSignExt.readPin(m_ReaderRoadSignExt, (void *) & inputRoadSignExt, m_ui32TimestampRoadSign))){
        // process data

        m_ui32TimestampRoadSign = inputRoadSignExt.ui32ArduinoTimestamp;

        // LOG_ERROR("dis x %f", inputRoadSignExt.af32TVec[2]);
        RETURN_IF_FAILED(ProcessRoadSignExt(inputRoadSignExt, m_pClock->GetTime()));

    }


    // if new signal is LS
    TLaserScannerData::Data tmp_LSData;
    if(IS_OK(o_TLaserScannerData.readPin(m_ReaderLaserScanner, (void *) & tmp_LSData))){

        for (uint32_t i=0; (i<tmp_LSData.ui32Size); i++)
        {
            tmp_LSData.tScanArray[i].f32Angle = AngleCompensation(tmp_LSData.tScanArray[i].f32Angle);
        }
        RETURN_IF_FAILED(ProcessLaserScannerData(tmp_LSData));

    }

    // done
    RETURN_NOERROR;
}
// ------------------------------
// ------ OTHER FUNCTIONS -------
// ------------------------------

tResult IntersectionDetectorFilterNew::LoadProperties(){


      //TEST CASE
        LOG_SUCCESS("here in load properties");
        isd_properties properties;
        LoadXMLData("/home/aadc/AADC/utilities/propertyXMLs/isd_properties.xml", (void*) (&properties));

        m_propF32XOffset = properties.m_propF32XOffset;
        m_propF32DistStartSignDetection = properties.m_propF32DistStartSignDetection;
        m_propF32DistCameraLS = properties.m_propF32DistCameraLS;
        m_propF32AdjustmentFactor = properties.m_propF32AdjustmentFactor;
        m_propF32DistStartLS = properties.m_propF32DistStartLS;
        m_propF32DistFaktor = properties.m_propF32DistFaktor;
        m_propF32MinAngle = properties.m_propF32MinAngle;
        m_propF32MaxAngle = properties.m_propF32MaxAngle;
        m_propF32RadiusRange = properties.m_propF32RadiusRange;
        m_propF32XMax = properties.m_propF32XMax;
        m_propF32XMin = properties.m_propF32XMin;
        m_propF32YMax = properties.m_propF32YMax;
        m_propF32YMin = properties.m_propF32YMin;

        m_propF32XMinArea = properties.m_propF32XMinArea;
        m_propF32XMaxArea = properties.m_propF32XMaxArea;
        m_propF32WidthSign = properties.m_propF32WidthSign;
        m_propF32ThresholdXNoSign = properties.m_propF32ThresholdXNoSign;
        m_propISampleCount = properties.m_propISampleCount;

        tFloat32 tmp_distStartSignDetection  = m_propF32DistStartSignDetection;
        tFloat32 tmp_distStartLS  = m_propF32DistStartLS;

        LOG_SUCCESS("dist sign detection %f, dist start ls %f", tmp_distStartSignDetection, tmp_distStartLS);

    RETURN_NOERROR;


}



tResult IntersectionDetectorFilterNew::ProcessRoadSignExt(TRoadSignExt::Data inputRoadSignExt, tTimeStamp )
{


    if (GetCurrentActionIn() == AC_ISD_START)
    {
        tFloat32 tmp_distStartSignDetection;
        tmp_distStartSignDetection = m_propF32DistStartSignDetection;

        //LOG_ERROR("camera rs distance: %f, %f, %f", inputRoadSignExt.af32TVec[2],inputRoadSignExt.af32TVec[1],inputRoadSignExt.af32TVec[0]);
        //        LOG_ERROR("threshold: %f", tmp_distStartSignDetection);
        TRoadSignExt::Data roadSignExt;
        roadSignExt = inputRoadSignExt;

        //If correct road sign near enough and on right side, transmit goal
        if (roadSignExt.af32TVec[2] < tmp_distStartSignDetection // distance in x from camera to roadsign
                && (roadSignExt.i16Identifier == roadsignIDs::MARKER_ID_UNMARKEDINTERSECTION
                    || roadSignExt.i16Identifier == roadsignIDs::MARKER_ID_STOPANDGIVEWAY
                    || roadSignExt.i16Identifier == roadsignIDs::MARKER_ID_HAVEWAY
                    || roadSignExt.i16Identifier == roadsignIDs::MARKER_ID_GIVEWAY
                    || roadSignExt.i16Identifier == roadsignIDs::MARKER_ID_PARKINGAREA))
        {
            m_dataRoadSignExtIn = inputRoadSignExt;

            SetSignDetected(tTrue);

            //m_bSignDetected = tTrue;
            // LOG_ERROR("ISD sign detected!");
        }
    }
    //af32TVec 0:x 1:y 2:z
    //Coordinate System Topdown view: x: right, y: front

    RETURN_NOERROR;
}

tResult IntersectionDetectorFilterNew::ProcessLaserScannerData(TLaserScannerData::Data inputLS)
{


    tFloat32 tmp_distStartLS;
    tmp_distStartLS = m_propF32DistStartLS;



    if (GetSignDetected() == tFalse )
    {
        RETURN_NOERROR;
    }
    else
    {
      tFloat32 tmpDist= m_dataRoadSignExtIn.af32TVec[2];
      //LOG_INFO("dist: %f grenze %f", tmpDist, tmp_distStartLS);
      if (m_dataRoadSignExtIn.af32TVec[2]  < tmp_distStartLS)

      {
          TPoseStruct::Data localGoal;
          localGoal.f32PosX = m_dataRoadSignExtIn.af32TVec[2] + m_propF32XOffset - m_propF32DistCameraLS;

          TFeedbackStruct::Data feedback;
          feedback.ui8FilterId = F_INTERSECTION_DETECTOR;
          feedback.ui32FeedbackStatus = FB_ISD_FOUND;
      //localGoal.f32PosX = m_dataRoadSignPoseOut.f32PosX;
      

          localGoal.f32PosY = 0;//
            localGoal.f32Yaw = 0;
            localGoal.f32CarSpeed = 0.6;
            localGoal.ui32ArduinoTimestamp = m_pClock->GetStreamTime();
            RETURN_IF_FAILED(o_TFeedbackStruct.writePin(m_WriterFeedback, (void *) &feedback, m_pClock->GetStreamTime()));
            RETURN_IF_FAILED(o_TPoseStruct.writePin(m_WriterLocalGoal, (void *) &localGoal, m_pClock->GetStreamTime()));
                  LOG_INFO("sent!!!!! %f", localGoal.f32PosX);
          SetCurrentActionIn(AC_ISD_STOP);
          SetSignDetected(tFalse);
      //m_dataCurrentActionIn.ui32Command = AC_ISD_STOP;
          RETURN_NOERROR;
      }
    else {
      RETURN_NOERROR;
}




















//
//
//
//
// //                LOG_ERROR("ISD sign detected! ls not started yet! grenze: %f", tmp_distStartLS);
//         tmp_distanceX = m_dataRoadSignExtIn.af32TVec[2] - tmp_distCameraLS; // distance x from laser scanner to road sign
//         tmp_distanceY = m_dataRoadSignExtIn.af32TVec[0] ;
//
//         if (tmp_distanceX > tmp_distStartLS)
//         {
//
// //                     LOG_ERROR("distStartLS %f",tmp_distStartLS);
//             m_dataRoadSignPoseOut.f32PosX = tmp_distanceX;
//             m_dataRoadSignPoseOut.f32PosY = tmp_distanceY;
//         }
//
//         else // tmp_distanceX <= 0.4, switch to laser scanner, but roadSign signal still comes for a while
//         {
// //                        LOG_INFO("1");
//             //tFloat32 f32Xmax = m_propF32XMax;
//             //tFloat32 f32Ymax=m_propF32XMin;
//             //tFloat32 f32Xmin=m_propF32YMax;
//             //tFloat32 f32Ymin=m_propF32YMin;
//
//             tFloat32 f32Xmax = m_propF32XMax;
//             tFloat32 f32Xmin = m_propF32XMin;
//             //tFloat32 f32Xmax = tmp_distanceX*1000.0+m_propF32XMaxArea;
//             //tFloat32 f32Xmin=tmp_distanceX*1000.0-m_propF32XMinArea;
//             tFloat32 f32Ymax=m_propF32YMax;
//             tFloat32 f32Ymin=m_propF32YMin;
//
//             //tFloat32 f32Xmin = m_propF32DistStartSignDetection*1000.0f+m_propF32XMinArea;
//             //tFloat32 f32Xmax = m_propF32DistStartSignDetection*1000.0f+m_propF32XMaxArea;
//
//             TLaserScannerData::Data laserInROI;
//             laserInROI.ui32Size = 0;
//
//             tFloat32 f32DistanceMax = sqrt(pow(f32Xmax, 2)+pow(f32Ymin, 2));
//             //             LOG_INFO("f32DistanceMax: %f", f32DistanceMax);
//
//             for (tUInt32 i = 0; i < inputLS.ui32Size; i++)
//             {
//                 inputLS.tScanArray[i].f32Angle = AngleCompensation(inputLS.tScanArray[i].f32Angle);
//                 if((inputLS.tScanArray[i].f32Radius != 0.0f) && (inputLS.tScanArray[i].f32Radius <= f32DistanceMax) && (inputLS.tScanArray[i].f32Angle > 0.0f))
//                 {
//                     tFloat32 f32rcos = inputLS.tScanArray[i].f32Radius*cosf(inputLS.tScanArray[i].f32Angle * tFloat32(PI) / 180.0f);//x
//                     tFloat32 f32rminussin = inputLS.tScanArray[i].f32Radius*(-sinf(inputLS.tScanArray[i].f32Angle * tFloat32(PI) / 180.0f));//y
//
//                     if((f32Xmin <= f32rcos) && (f32Xmax >= f32rcos) && (f32Ymin <= f32rminussin) &&  (f32rminussin <= f32Ymax))
//                     {
//
//                         laserInROI.tScanArray[laserInROI.ui32Size].f32Radius = f32rcos;//x
//                         laserInROI.tScanArray[laserInROI.ui32Size].f32Angle = f32rminussin;//y
//                         laserInROI.ui32Size++;
//
//                     }
//                 }
//             }
//
//
//
//
//             int samplecount = m_propISampleCount;
//             if ( laserInROI.ui32Size <= samplecount)
//             {
// //                                 LOG_WARNING("Selected points: %d smaller as %d. ", laserInROI.ui32Size, samplecount);
//                 RETURN_NOERROR;
//             }
//             else
//             {
//                 if (GetCurrentActionIn() == AC_ISD_START)
//                 {
//                     //
//                     LOG_INFO("Punkt erkannt!");
//                     TPoseStruct::Data localGoal;
//                     //localGoal.f32PosX = laserInROI.tScanArray[((int)(laserInROI.ui32Size / 2))].f32Radius;
//                     tFloat32 f32Xmean = 0.0f;
//                     for (tUInt32 i = 0; i < laserInROI.ui32Size; i++)
//                     {
//                         f32Xmean += laserInROI.tScanArray[i].f32Radius;
//                     }
//                     f32Xmean /= laserInROI.ui32Size;
//                     //find laserPoint with the closest x Value
//                     tFloat32 f32ClosestYValue = -9999.9f;
//                     tFloat32 f32XValueOfClosestYValue = 0.0f;
//
//                     for (tUInt32 i = 0; i < laserInROI.ui32Size; i++)
//                     {
//                         if(f32ClosestYValue < laserInROI.tScanArray[i].f32Angle)
//                         {
//                             f32ClosestYValue = laserInROI.tScanArray[i].f32Angle;
//                             f32XValueOfClosestYValue = laserInROI.tScanArray[i].f32Radius;
//                         }
//                     }
//                     TLaserScannerData::Data possibleSign;
//                     possibleSign.ui32Size = 0;
//                     //tUInt32 ui32CounterForY = 0;
//                     //DOTO: ask judith (pattern recognition)
//                     tFloat32 f32Xmean2 = 0.0f;
//                     LOG_INFO("Closest Y Value: %f", f32ClosestYValue);
//                     LOG_INFO("X threshold: %f", (f32XValueOfClosestYValue+m_propF32ThresholdXNoSign));
//                     for (tUInt32 i = 0; i < laserInROI.ui32Size; i++)
//                     {
//                         LOG_INFO("y: %f", laserInROI.tScanArray[i].f32Angle);
//                         LOG_INFO("x. %f", laserInROI.tScanArray[i].f32Radius);
//
//                         if((f32ClosestYValue > laserInROI.tScanArray[i].f32Angle) && (laserInROI.tScanArray[i].f32Radius < (f32XValueOfClosestYValue+m_propF32ThresholdXNoSign)) && ((f32XValueOfClosestYValue-m_propF32ThresholdXNoSign)< laserInROI.tScanArray[i].f32Radius))
//                         {
//                             possibleSign.tScanArray[possibleSign.ui32Size].f32Angle = laserInROI.tScanArray[i].f32Angle;//y
//                             possibleSign.tScanArray[possibleSign.ui32Size].f32Radius = laserInROI.tScanArray[i].f32Radius;//x
//                             possibleSign.ui32Size++;
//                             f32Xmean2 = f32Xmean2 + laserInROI.tScanArray[i].f32Radius;
//                         }
//                     }
//                     LOG_INFO("possibleSign.ui32Size: %d", possibleSign.ui32Size);
//                     if(possibleSign.ui32Size != 0)
//                     {
//                         f32Xmean2 /= possibleSign.ui32Size;
//                         localGoal.f32PosX = f32Xmean2/1000;
//                         LOG_INFO("possible sign:");
//                     }
//                     //LOG_ERROR("f32Xmean: %f", f32Xmean);
//                     else
//                     {
//                     localGoal.f32PosX = f32Xmean/1000 * m_propF32DistFaktor;
//                     }
//                     //                LOG_INFO("localGoal.f32PosX %f", localGoal.f32PosX);
//                     LOG_ERROR("ls detected X: %f", localGoal.f32PosX);
//                     TFeedbackStruct::Data feedback;
//                     feedback.ui8FilterId = F_INTERSECTION_DETECTOR;
//                     feedback.ui32FeedbackStatus = FB_ISD_FOUND;
//                     //localGoal.f32PosX = m_dataRoadSignPoseOut.f32PosX;
//                     localGoal.f32PosY = 0;//
//                     localGoal.f32Yaw = 0;
//                     localGoal.f32CarSpeed = 0.6;
//                     localGoal.ui32ArduinoTimestamp = m_pClock->GetStreamTime();
//                     RETURN_IF_FAILED(o_TFeedbackStruct.writePin(m_WriterFeedback, (void *) &feedback, m_pClock->GetStreamTime()));
//                     RETURN_IF_FAILED(o_TPoseStruct.writePin(m_WriterLocalGoal, (void *) &localGoal, m_pClock->GetStreamTime()));
//                     //            LOG_INFO("sent!!!!!");
//                     SetCurrentActionIn(AC_ISD_STOP);
//                     //m_dataCurrentActionIn.ui32Command = AC_ISD_STOP;
//                     RETURN_NOERROR;
//                 }
//             }
//
//         }


    }


    RETURN_NOERROR;
}

tResult IntersectionDetectorFilterNew::SetCurrentActionIn(tUInt32 currentAction)
{
    boost::lock_guard<boost::mutex> lock(criticalSectionActionCommand);
    m_dataCurrentActionIn.ui32Command = currentAction;
    RETURN_NOERROR;
}

tUInt32 IntersectionDetectorFilterNew::GetCurrentActionIn()
{
    boost::lock_guard<boost::mutex> lock(criticalSectionActionCommand);
    return m_dataCurrentActionIn.ui32Command;
}

tResult IntersectionDetectorFilterNew::SetSignDetected(tBool signDetected)
{
    boost::lock_guard<boost::mutex> lock(criticalSectionSignDetected);
    m_bSignDetected = signDetected;
    RETURN_NOERROR;
}

tBool IntersectionDetectorFilterNew::GetSignDetected()
{
    boost::lock_guard<boost::mutex> lock(criticalSectionSignDetected);
    return m_bSignDetected;
}


tFloat32 IntersectionDetectorFilterNew::AngleCompensation(tFloat32 angle)
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
        //LOG_ERROR("Angle out of range!");
        return 100000;//Error value but without error handeling!
    }
}
