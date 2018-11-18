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

#pragma once

//*************************************************************************************************
#define CID_SELECTSPEED_DATA_TRIGGERED_FILTER "select_speed.filter.user.aadc.cid"

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;

#include <boost/thread.hpp>

class SelectSpeed : public cTriggerFunction
{
private:
    
    object_ptr<adtf::services::IReferenceClock> m_pClock;

    TSignalValue    o_LDSpeedSignalId;
    TSignalValue    o_MTPSpeedSignalId;
    TSignalValue    o_SpeedSignalId;
    TActionStruct   o_ActionStructId;
    TFeedbackStruct o_FeedbackStructId;
    TSignalValue    o_LDSteeringAngleId;
    TSignalValue    o_MTPSteeringAngleId;
    TSignalValue    o_SteeringAngleId;

    /*-------------- PINS --------------*/
    // create pins for input and output
    // coding convention:	m_Reader******	for input  signals
    //			m_Writer******	for output signals

    // input pins
    //cPinReader m_ReaderActionStopSpeed;//gleich auf 0 setzen?
    cPinReader m_ReaderMoveToPointSpeed;
    cPinReader m_ReaderLaneDetectionSpeed;
    cPinReader m_ReaderAction;
    cPinReader m_ReaderMoveToPointSteeringAngle;
    cPinReader m_ReaderLaneDetectionSteeringAngle;

    // output pins
    cPinWriter m_WriterOutputSpeed;
    cPinWriter m_WriterFeedback;
    cPinWriter m_WriterSteeringAngle;

    adtf::base::property_variable<tBool> m_propEnableDebugging = tBool(true);

    tBool m_bLaneDetection = tFalse;
    tBool m_bMoveToPoint = tFalse;

public:

    /*! Default constructor. */
    SelectSpeed();

    /*! Destructor. */
    virtual ~SelectSpeed() = default;

    /**
    * Overwrites the Configure
    * This is to Read Properties prepare your Trigger Function
    */
    tResult virtual Configure() override;
    /**
    * Overwrites the Process
    * You need to implement the Reading and Writing of Samples within this function
    * MIND: Do Reading until the Readers queues are empty or use the IPinReader::GetLastSample()
    * This FUnction will be called if the Run() of the TriggerFunction was called.
    */
    tResult virtual Process(tTimeStamp tmTimeOfTrigger) override;

    tResult TransmitSpeed(TSignalValue::Data transmittedSpeed);
    tResult TransmitFeedback(TFeedbackStruct::Data transmittedFeedback);
    tResult TransmitSteering(TSignalValue::Data transmittedSteering);

    //variables
    TSignalValue::Data m_LaneDetectionSpeed;
    TSignalValue::Data m_LaneDetectionSteering;
    TSignalValue::Data m_MoveToPointSpeed;
    TSignalValue::Data m_MoveToPointSteering;
    TActionStruct::Data m_Action;
    TFeedbackStruct::Data m_Feedback;

    boost::mutex m_mutexAction, m_mutexSpeed, m_mutexSteering, m_mutexFeedback, m_mutexLD, m_mutexMTP;
};


//*************************************************************************************************
