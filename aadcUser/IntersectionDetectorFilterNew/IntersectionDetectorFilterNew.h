/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**********************************************************************
* This filter finds the position of the stop line at a crossing

* $Adapted by:: Xiangfei#  $Date:: 2018-08-30 12:44:00#
**********************************************************************/

#pragma once

#define CID_INTERSECTION_DETECTOR_NEW_FILTER_FILTER "intersection_detector_new_filter.filter.user.aadc.cid"

// namespace
using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;

#include <adtf_platform_inc.h>
#include <adtf_systemsdk.h>
#include <boost/thread.hpp>
#include "PoseCache.h"
#include "tinyxml2.h"

// IntersectionDetectorFilterNew
class IntersectionDetectorFilterNew : public cTriggerFunction
{
private:
    /*------------ STRUCTS -------------*/
    // create structs to hold information we get from SignalValues
    // coding convention: 	o_******Id

    TFeedbackStruct o_TFeedbackStruct;
    TActionStruct o_TActionStruct;
    TRoadSignExt o_TRoadSignExt;
    //TTrafficSign m_TTrafficSign;
    TSignalValue o_TSignalValue;
    TPoseStruct o_TPoseStruct;
    TLaserScannerData o_TLaserScannerData;

    /*-------------- PINS --------------*/
    // create pins for input and output
    // coding convention:	m_Reader******	for input  signals
    //			m_Writer******	for output signals

    // input pins
    cPinReader m_ReaderRoadSignExt;
    cPinReader m_ReaderAction;
    cPinReader m_ReaderLaserScanner;

    // output pins
    cPinWriter m_WriterFeedback;
    cPinWriter m_WriterLocalGoal;
//    cPinWriter m_WriterAction;
    /*------------ PROPERTIES ----------*/
    // set property variables
    // coding convention:	m_prop******
    adtf::base::property_variable<tFloat32> m_propF32XOffset          = tFloat(0.0f); // distance between marker and stop line in x direction
    adtf::base::property_variable<tFloat32> m_propF32DistStartSignDetection  = tFloat(0.9f); // minimum distance from camera to road sign, smaller -> sign detected
    adtf::base::property_variable<tFloat32> m_propF32DistCameraLS     = tFloat(0.2f); // distance between camera and laser scanner
    adtf::base::property_variable<tFloat32> m_propF32AdjustmentFactor = tFloat(-0.1f); // distance for small final adjustment
    adtf::base::property_variable<tFloat32> m_propF32DistStartLS      = tFloat(0.3f); //distance from laser scanner to road sign, smaller -> send pose to moveToPoint
    adtf::base::property_variable<tFloat32> m_propF32DistFaktor         = tFloat(1.0f); // min angle for ls pose detection
    adtf::base::property_variable<tFloat32> m_propF32MinAngle         = tFloat(0.0f); // min angle for ls pose detection
    adtf::base::property_variable<tFloat32> m_propF32MaxAngle         = tFloat(90.0f); // max angle for ls pose detection
    adtf::base::property_variable<tFloat32> m_propF32RadiusRange         = tFloat(0.1f); // range for pose detection
    adtf::base::property_variable<int>    m_propISampleCount        = int(2); // sample for pose detection by laser scanner

    adtf::base::property_variable<tFloat32>    m_propF32XMax        = tFloat(500.0f); // sample for pose detection by laser scanner
    adtf::base::property_variable<tFloat32>    m_propF32XMin        = tFloat(200.0f); // sample for pose detection by laser scanner
    adtf::base::property_variable<tFloat32>    m_propF32YMax        = tFloat(-200.0f); // sample for pose detection by laser scanner
    adtf::base::property_variable<tFloat32>    m_propF32YMin        = tFloat(-900.0); // sample for pose detection by laser scanner

    adtf::base::property_variable<tFloat32>    m_propF32XMinArea     = tFloat(-100.0); // sample for pose detection by laser scanner
    adtf::base::property_variable<tFloat32>    m_propF32XMaxArea     = tFloat(200.0); // sample for pose detection by laser scanner

    adtf::base::property_variable<tFloat32>    m_propF32WidthSign     = tFloat(150.0); // [mm]
    adtf::base::property_variable<tFloat32>    m_propF32ThresholdXNoSign     = tFloat(80.0);//[mm]



    /*------------ VARIABLES -----------*/
    // coding convention:	m_******
    object_ptr<adtf::services::IReferenceClock> m_pClock; // clock for timestamps

    TPoseStruct::Data m_dataRoadSignPoseOut;
    TRoadSignExt::Data m_dataRoadSignExtIn;
    TActionStruct::Data m_dataCurrentActionIn;
    TLaserScannerData::Data m_dataLaserScannerIn;
    tBool m_bSignDetected;
    tBool m_bLSStarted;
    // timestamps

    tUInt32 m_ui32TimestampRoadSign;
    tUInt32 m_ui32TimestampAction;
    tUInt32 m_ui32TimestampPose;

public:
    /*------------ FUNCTIONS -----------*/
    // constructor
    IntersectionDetectorFilterNew();

    // destructor
    ~IntersectionDetectorFilterNew(){}

    // Configure
    tResult Configure() override;

    // Process
    tResult Process(tTimeStamp tmTimeOfTrigger) override;

    boost::mutex  criticalSectionActionCommand, criticalSectionSignDetected;

    // Process RoadSignExt
    tResult ProcessRoadSignExt(TRoadSignExt::Data inputRoadSignExt, tTimeStamp);

    // Process Action
    tResult ProcessAction(TActionStruct::Data inputAction);

    tResult SetCurrentActionIn(tUInt32 currentAction);
    tUInt32 GetCurrentActionIn();
    tResult SetSignDetected(tBool signDetected);
    tBool GetSignDetected();

    // Process Pose
//    tResult ProcessPose(TPoseStruct::Data inputPose, tTimeStamp);

    tResult ProcessLaserScannerData(TLaserScannerData::Data);

tFloat32 AngleCompensation(tFloat32 angle);

tResult LoadProperties();
}; // IntersectionDetectorFilterNew
