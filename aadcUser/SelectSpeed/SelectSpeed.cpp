/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Coded by Illmer: status: doing
Filter selcet which speed should be transmitted to ACC -> selects it according to the action command of the state machine

**********************************************************************/

#include "stdafx.h"
#include "SelectSpeed.h"
#include "ScmCommunication.h"
#include <stdlib.h>

#include <boost/thread.hpp>


ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_SELECTSPEED_DATA_TRIGGERED_FILTER,
                                    "SelectSpeed",
                                    SelectSpeed,
//                                    adtf::filter::pin_trigger({"ActionStructInput"}));
adtf::filter::timer_trigger(2));
// -------------------------------
// --------- CONSTRUCTOR ---------
// -------------------------------
// in the constructor, we will set the adtf streamtypes for signal values and pins
SelectSpeed::SelectSpeed()
{
    ////LOG_INFO("SelectSpeed: constructor start");
    o_LDSpeedSignalId   .registerPin(this, m_ReaderLaneDetectionSpeed        , "LaneDetectionSpeed");
    o_MTPSpeedSignalId  .registerPin(this, m_ReaderMoveToPointSpeed          , "MoveToPointSpeed"  );
    o_ActionStructId    .registerPin(this, m_ReaderAction                    ,  "ActionStructInput");
    o_SpeedSignalId     .registerPin(this, m_WriterOutputSpeed               , "OutputSpeed");
    o_FeedbackStructId  .registerPin(this, m_WriterFeedback                  , "OutputFeedback");
    o_LDSteeringAngleId .registerPin(this, m_ReaderLaneDetectionSteeringAngle, "LaneDetectionSteering");
    o_MTPSteeringAngleId.registerPin(this, m_ReaderMoveToPointSteeringAngle  , "MoveToPointSteering");
    o_SteeringAngleId   .registerPin(this, m_WriterSteeringAngle             , "OutputSteeringAngle");
    ////LOG_INFO("SelectSpeed: registered all pins");


    RegisterPropertyVariable("enableDebug" , m_propEnableDebugging);
    ////LOG_INFO("SelectSpeed: constructor end");

}
//implement the Configure function to read ALL Properties
tResult SelectSpeed::Configure()
{
    ////LOG_INFO("SelectSpeed: configure start");
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    ////LOG_INFO("SelectSpeed: configure end");
    RETURN_NOERROR;
}

tResult SelectSpeed::Process(tTimeStamp)
{
    ////LOG_INFO("SelectSpeed: process start");
    ////LOG_INFO("SelectSpeed: start read action");
    // action input
    TActionStruct::Data actionIn;
    static tTimeStamp actionTimestamp = 0;
    if(IS_OK(o_ActionStructId.readPin(m_ReaderAction, (void *) &actionIn, actionTimestamp)))
    {
        boost::lock_guard<boost::mutex> lock(m_mutexAction);
        actionTimestamp = actionIn.ui32ArduinoTimestamp;
        if(actionIn.ui8FilterId == F_SELECT_SPEED)
        {
            m_Action = actionIn;
            if(m_propEnableDebugging)
            {
                //LOG_INFO("SelectSpeed: rcv action (%d)", m_Action.ui32Command);
            }

            {
                boost::lock_guard<boost::mutex> lockLd(m_mutexLD);
                if(m_Action.ui32Command == AC_SES_LANEDETECTION)
                {
                    m_bLaneDetection = tTrue;
                    TFeedbackStruct::Data tmp_feedback;
                    tmp_feedback.ui8FilterId = F_SELECT_SPEED;
                    tmp_feedback.ui32FeedbackStatus = FB_SES_LANE_DETECTION;
                    TransmitFeedback(tmp_feedback);
                }
                else
                {
                    m_bLaneDetection = tFalse;
                }
                boost::lock_guard<boost::mutex> lockmtp(m_mutexMTP);
                if(m_Action.ui32Command == AC_SES_MOVETOPOINT)
                {
                    m_bMoveToPoint = tTrue;
                    TFeedbackStruct::Data tmp_feedback;
                    tmp_feedback.ui8FilterId = F_SELECT_SPEED;
                    tmp_feedback.ui32FeedbackStatus = FB_SES_MOVETOPOINT;
                    TransmitFeedback(tmp_feedback);
                }
                else
                {
                    m_bMoveToPoint = tFalse;
                }
            }
        }
    }

    ////LOG_INFO("SelectSpeed: start read ldspeed");
    // ld speed
    TSignalValue::Data ldSpeed;
    static tTimeStamp ldSpeedtts = 0;
    if(IS_OK(o_LDSpeedSignalId.readPin(m_ReaderLaneDetectionSpeed, (void *) &ldSpeed, ldSpeedtts)))
    {
        boost::lock_guard<boost::mutex> lock(m_mutexAction);
        boost::lock_guard<boost::mutex> lockld(m_mutexLD);
        if(m_propEnableDebugging)
        {
            //LOG_INFO("SelectSpeed: rcv ld speed (%f)", ldSpeed.f32Value);
        }
        if(m_bLaneDetection)
        {
            TransmitSpeed(ldSpeed);
            TFeedbackStruct::Data tmp_feedback;
            tmp_feedback.ui8FilterId = F_SELECT_SPEED;
            tmp_feedback.ui32FeedbackStatus = FB_SES_LANE_DETECTION;
            TransmitFeedback(tmp_feedback);
        }
    }

    ////LOG_INFO("SelectSpeed: start read ldsteering");
    // ld Steering
    TSignalValue::Data ldSteering;
    static tTimeStamp ldSteeringtts = 0;
    if(IS_OK(o_LDSteeringAngleId.readPin(m_ReaderLaneDetectionSteeringAngle, (void *) &ldSteering, ldSteeringtts)))
    {
        boost::lock_guard<boost::mutex> lock(m_mutexAction);
        boost::lock_guard<boost::mutex> lockld(m_mutexLD);
        if(m_propEnableDebugging)
        {
            //LOG_INFO("SelectSpeed: rcv ld steering (%f)", ldSteering.f32Value);
        }
        if(m_bLaneDetection)
        {
            TransmitSteering(ldSteering);
            TFeedbackStruct::Data tmp_feedback;
            tmp_feedback.ui8FilterId = F_SELECT_SPEED;
            tmp_feedback.ui32FeedbackStatus = FB_SES_LANE_DETECTION;
            TransmitFeedback(tmp_feedback);
        }
    }

    //LOG_INFO("SelectSpeed: start read mtpspeed");
    // MTP speed
    TSignalValue::Data MTPSpeed;
    static tTimeStamp mtpSpeedtts = 0;
    if(IS_OK(o_MTPSpeedSignalId.readPin(m_ReaderMoveToPointSpeed, (void *) &MTPSpeed, mtpSpeedtts)))
    {
        boost::lock_guard<boost::mutex> lock(m_mutexAction);
        boost::lock_guard<boost::mutex> lockmtp(m_mutexMTP);
        if(m_propEnableDebugging)
        {
            //LOG_INFO("SelectSpeed: rcv mtp speed (%f)", MTPSpeed.f32Value);
        }
        if(m_bMoveToPoint)
        {
            TransmitSpeed(MTPSpeed);
            TFeedbackStruct::Data tmp_feedback;
            tmp_feedback.ui8FilterId = F_SELECT_SPEED;
            tmp_feedback.ui32FeedbackStatus = FB_SES_MOVETOPOINT;
            TransmitFeedback(tmp_feedback);
        }
    }

    //LOG_INFO("SelectSpeed: start read mtpsteering");
    // MTP Steering
    TSignalValue::Data MTPSteering;
    static tTimeStamp mtpSteeringtts = 0;
    if(IS_OK(o_MTPSteeringAngleId.readPin(m_ReaderMoveToPointSteeringAngle, (void *) &MTPSteering, mtpSteeringtts)))
    {
        boost::lock_guard<boost::mutex> lock(m_mutexAction);
        boost::lock_guard<boost::mutex> lockmtp(m_mutexMTP);
        if(m_propEnableDebugging)
        {
            //LOG_INFO("SelectSpeed: rcv mtp steering (%f)", MTPSteering.f32Value);
        }
        if(m_bMoveToPoint)
        {
            TransmitSteering(MTPSteering);
            TFeedbackStruct::Data tmp_feedback;
            tmp_feedback.ui8FilterId = F_SELECT_SPEED;
            tmp_feedback.ui32FeedbackStatus = FB_SES_MOVETOPOINT;
            TransmitFeedback(tmp_feedback);
        }
    }

    {
        boost::lock_guard<boost::mutex> lock(m_mutexAction);
        if(m_Action.ui32Command == AC_SES_ACTION_STOP)
        {
            TSignalValue::Data acSpeed;
            acSpeed.f32Value = 0.0f;
            TransmitSpeed(acSpeed);
            TFeedbackStruct::Data tmp_feedback;
            tmp_feedback.ui8FilterId = F_SELECT_SPEED;
            tmp_feedback.ui32FeedbackStatus = FB_SES_ACTION_STOP;
            TransmitFeedback(tmp_feedback);
        }
    }

    //LOG_INFO("SelectSpeed: process end");
    RETURN_NOERROR;
}

tResult SelectSpeed::TransmitSpeed(TSignalValue::Data transmittedSpeed)
{
    if(m_propEnableDebugging)
    {
        //LOG_INFO("SelectSpeed: sending speed (%f)...", transmittedSpeed.f32Value);
    }
    boost::lock_guard<boost::mutex> lock(m_mutexSpeed);
    RETURN_IF_FAILED(o_SpeedSignalId.writePin(m_WriterOutputSpeed, (void *) &transmittedSpeed, m_pClock->GetStreamTime()));
    if(m_propEnableDebugging)
    {
        LOG_INFO("SelectSpeed: done sending speed (%f)", transmittedSpeed.f32Value);
    }
    RETURN_NOERROR;
}

tResult SelectSpeed::TransmitSteering(TSignalValue::Data transmittedSteering)
{
    if(m_propEnableDebugging)
    {
        //LOG_INFO("SelectSpeed: sending steering (%f)...", transmittedSteering.f32Value);
    }
    boost::lock_guard<boost::mutex> lock(m_mutexSteering);
    RETURN_IF_FAILED(o_SteeringAngleId.writePin(m_WriterSteeringAngle , (void *) &transmittedSteering, m_pClock->GetStreamTime()));
    if(m_propEnableDebugging)
    {
        LOG_INFO("SelectSpeed: done sending steering (%f)", transmittedSteering.f32Value);
    }
    RETURN_NOERROR;
}

tResult SelectSpeed::TransmitFeedback(TFeedbackStruct::Data transmittedFeedback)
{
    if(m_propEnableDebugging)
    {
        //LOG_INFO("SelectSpeed: sending feedback (%d)...", transmittedFeedback.ui32FeedbackStatus);
    }
    boost::lock_guard<boost::mutex> lock(m_mutexFeedback);
    RETURN_IF_FAILED(o_FeedbackStructId.writePin(m_WriterFeedback, (void *) &transmittedFeedback, m_pClock->GetStreamTime()));
    if(m_propEnableDebugging)
    {
        LOG_INFO("SelectSpeed: done sending feedback (%d)", transmittedFeedback.ui32FeedbackStatus);
    }
    RETURN_NOERROR;
}
