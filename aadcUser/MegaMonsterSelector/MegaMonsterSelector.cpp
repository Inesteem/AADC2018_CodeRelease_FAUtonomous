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
#include "MegaMonsterSelector.h"
#include "ScmCommunication.h"
#include <stdlib.h>

#include <boost/thread.hpp>

//  MEGA MONSTER SELECTOR 5000x Lite Booster Pack XXL Crazy STUFF

ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_MEGAMONSTERSELECTOR_DATA_TRIGGERED_FILTER,
                                    "MegaMonsterSelector",
                                    MegaMonsterSelector,
//                                    adtf::filter::pin_trigger({"ActionStructInput"}));
adtf::filter::timer_trigger(2));
// -------------------------------
// --------- CONSTRUCTOR ---------
// -------------------------------
// in the constructor, we will set the adtf streamtypes for signal values and pins
MegaMonsterSelector::MegaMonsterSelector()
{
    //  MEGA MONSTER SELECTOR 5000x Lite Booster Pack XXL Crazy STUFF
    // first create some monster inputs1!! and outputs lol crazyyy
    o_BinhSpeedSignalId     .registerPin(this, m_ReaderBinhSpeed           , "BinhSpeed");
    o_BinhSteeringAngleId   .registerPin(this, m_ReaderBinhSteeringAngle   , "BinhSteering");
    o_AndreasSpeedSignalId  .registerPin(this, m_ReaderNicoSpeed           , "NicoSpeed");
    o_AndreasSteeringAngleId.registerPin(this, m_ReaderNicoSteeringAngle   , "NicoSteering");
    o_NicoSpeedSignalId     .registerPin(this, m_ReaderAndreasSpeed        , "AndreasSpeed");
    o_NicoSteeringAngleId   .registerPin(this, m_ReaderAndreasSteeringAngle, "AndreasSteering");
    o_JudithSpeedSignalId   .registerPin(this, m_ReaderJudithSpeed         , "JudithSpeed");
    o_JudithSteeringAngleId .registerPin(this, m_ReaderJudithSteeringAngle , "JudithSteering");

    o_ActionStructId    .registerPin(this, m_ReaderAction                    , "ActionStructInput");
    o_SpeedSignalId     .registerPin(this, m_WriterOutputSpeed               , "OutputSpeed");
    o_FeedbackStructId  .registerPin(this, m_WriterFeedback                  , "OutputFeedback");
    o_SteeringAngleId   .registerPin(this, m_WriterSteeringAngle             , "OutputSteeringAngle");

    RegisterPropertyVariable("enableDebug" , m_propEnableDebugging);

}
//implement the Configure function to read ALL Properties
tResult MegaMonsterSelector::Configure()
{
    //  MEGA MONSTER SELECTOR 5000x Lite Booster Pack XXL Crazy STUFF
    // this is where we create time like a God
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    RETURN_NOERROR;
}

tResult MegaMonsterSelector::Process(tTimeStamp)
{
    //  MEGA MONSTER SELECTOR 5000x Lite Booster Pack XXL Crazy STUFF
    // action input
    // GIVE ME SOME ACTION
    TActionStruct::Data actionIn;
    static tTimeStamp actionTimestamp = 0;
    if(IS_OK(o_ActionStructId.readPin(m_ReaderAction, (void *) &actionIn, actionTimestamp)))
    {
        boost::lock_guard<boost::mutex> lock(m_mutexAction);
        actionTimestamp = actionIn.ui32ArduinoTimestamp;
        if(actionIn.ui8FilterId == F_MEGA_MONSTER_SELECTOR)
        {
            m_Action = actionIn;
            if(m_propEnableDebugging)
            {
                LOG_INFO("SelectSpeed: rcv action (%d)", m_Action.ui32Command);
            }

            {
                // i am hungryyyyy o.o
                boost::lock_guard<boost::mutex> lockBinh(m_mutexBinh);
                if(m_Action.ui32Command == AC_MMS_BINH)
                {
                    m_bBinh = tTrue;
                    TFeedbackStruct::Data tmp_feedback;
                    tmp_feedback.ui8FilterId = F_MEGA_MONSTER_SELECTOR;
                    tmp_feedback.ui32FeedbackStatus = FB_MMS_BINH;
                    TransmitFeedback(tmp_feedback);
                }
                else
                {
                    m_bBinh = tFalse;
                }
                boost::lock_guard<boost::mutex> lockNico(m_mutexNico);
                if(m_Action.ui32Command == AC_MMS_NICO)
                {
                    m_bNico = tTrue;
                    TFeedbackStruct::Data tmp_feedback;
                    tmp_feedback.ui8FilterId = F_MEGA_MONSTER_SELECTOR;
                    tmp_feedback.ui32FeedbackStatus = FB_MMS_NICO;
                    TransmitFeedback(tmp_feedback);
                }
                else
                {
                    m_bNico = tFalse;
                }
                boost::lock_guard<boost::mutex> lockAndreas(m_mutexAndreas);
                if(m_Action.ui32Command == AC_MMS_ANDREAS)
                {
                    m_bAndreas = tTrue;
                    TFeedbackStruct::Data tmp_feedback;
                    tmp_feedback.ui8FilterId = F_MEGA_MONSTER_SELECTOR;
                    tmp_feedback.ui32FeedbackStatus = FB_MMS_ANDREAS;
                    TransmitFeedback(tmp_feedback);
                }
                else
                {
                    m_bAndreas = tFalse;
                }
                boost::lock_guard<boost::mutex> lockJudith(m_mutexJudith);
                if(m_Action.ui32Command == AC_MMS_JUDITH)
                {
                    m_bJudith = tTrue;
                    TFeedbackStruct::Data tmp_feedback;
                    tmp_feedback.ui8FilterId = F_MEGA_MONSTER_SELECTOR;
                    tmp_feedback.ui32FeedbackStatus = FB_MMS_JUDITH;
                    TransmitFeedback(tmp_feedback);
                }
                else
                {
                    m_bJudith = tFalse;
                }
            }
        }
    }

    /*
    *
    *
    *  XIANGFEI IS THE BEST OF THE BEST, HE IS OUR BRAIN
    *
    *
    */



    // ---------------- BINH IS THE BEST ----------------------
    // binh speed
    TSignalValue::Data binhSpeed;
    static tTimeStamp binhSpeedtts = 0;
    if(IS_OK(o_BinhSpeedSignalId.readPin(m_ReaderBinhSpeed, (void *) &binhSpeed, binhSpeedtts)))
    {
        boost::lock_guard<boost::mutex> lock(m_mutexAction);
        boost::lock_guard<boost::mutex> lockld(m_mutexBinh);
        if(m_propEnableDebugging)
        {
            //LOG_INFO("SelectSpeed: rcv binh speed (%f)", binhSpeed.f32Value);
        }
        if(m_bBinh)
        {
            TransmitSpeed(binhSpeed);
            TFeedbackStruct::Data tmp_feedback;
            tmp_feedback.ui8FilterId = F_MEGA_MONSTER_SELECTOR;
            tmp_feedback.ui32FeedbackStatus = FB_MMS_BINH;
            TransmitFeedback(tmp_feedback);
        }
    }

    // binh Steering
    TSignalValue::Data binhSteering;
    static tTimeStamp binhSteeringtts = 0;
    if(IS_OK(o_BinhSteeringAngleId.readPin(m_ReaderBinhSteeringAngle, (void *) &binhSteering, binhSteeringtts)))
    {
        boost::lock_guard<boost::mutex> lock(m_mutexAction);
        boost::lock_guard<boost::mutex> lockld(m_mutexBinh);
        if(m_propEnableDebugging)
        {
            //LOG_INFO("SelectSpeed: rcv binh steering (%f)", binhSteering.f32Value);
        }
        if(m_bBinh)
        {
            TransmitSteering(binhSteering);
            TFeedbackStruct::Data tmp_feedback;
            tmp_feedback.ui8FilterId = F_MEGA_MONSTER_SELECTOR;
            tmp_feedback.ui32FeedbackStatus = FB_MMS_BINH;
            TransmitFeedback(tmp_feedback);
        }
    }


    // ---------------- ANDREAS IS THE BEST ----------------------
    // Andreas speed
    TSignalValue::Data AndreasSpeed;
    static tTimeStamp AndreasSpeedtts = 0;
    if(IS_OK(o_AndreasSpeedSignalId.readPin(m_ReaderAndreasSpeed, (void *) &AndreasSpeed, AndreasSpeedtts)))
    {
        boost::lock_guard<boost::mutex> lock(m_mutexAction);
        boost::lock_guard<boost::mutex> lockld(m_mutexAndreas);
        if(m_propEnableDebugging)
        {
            //LOG_INFO("SelectSpeed: rcv andreas speed (%f)", AndreasSpeed.f32Value);
        }
        if(m_bAndreas)
        {
            TransmitSpeed(AndreasSpeed);
            TFeedbackStruct::Data tmp_feedback;
            tmp_feedback.ui8FilterId = F_MEGA_MONSTER_SELECTOR;
            tmp_feedback.ui32FeedbackStatus = FB_MMS_ANDREAS;
            TransmitFeedback(tmp_feedback);
        }
    }

    // Andreas Steering
    TSignalValue::Data AndreasSteering;
    static tTimeStamp AndreasSteeringtts = 0;
    if(IS_OK(o_AndreasSteeringAngleId.readPin(m_ReaderAndreasSteeringAngle, (void *) &AndreasSteering, AndreasSteeringtts)))
    {
        boost::lock_guard<boost::mutex> lock(m_mutexAction);
        boost::lock_guard<boost::mutex> lockld(m_mutexAndreas);
        if(m_propEnableDebugging)
        {
            //LOG_INFO("SelectSpeed: rcv andreas steering (%f)", AndreasSteering.f32Value);
        }
        if(m_bAndreas)
        {
            TransmitSteering(AndreasSteering);
            TFeedbackStruct::Data tmp_feedback;
            tmp_feedback.ui8FilterId = F_MEGA_MONSTER_SELECTOR;
            tmp_feedback.ui32FeedbackStatus = FB_MMS_ANDREAS;
            TransmitFeedback(tmp_feedback);
        }
    }
    // ---------------- NICO IS THE BEST ----------------------
    // Nico speed
    TSignalValue::Data NicoSpeed;
    static tTimeStamp NicoSpeedtts = 0;
    if(IS_OK(o_NicoSpeedSignalId.readPin(m_ReaderNicoSpeed, (void *) &NicoSpeed, NicoSpeedtts)))
    {
        boost::lock_guard<boost::mutex> lock(m_mutexAction);
        boost::lock_guard<boost::mutex> lockld(m_mutexNico);
        if(m_propEnableDebugging)
        {
            //LOG_INFO("SelectSpeed: rcv Nico speed (%f)", NicoSpeed.f32Value);
        }
        if(m_bNico)
        {
            TransmitSpeed(NicoSpeed);
            TFeedbackStruct::Data tmp_feedback;
            tmp_feedback.ui8FilterId = F_MEGA_MONSTER_SELECTOR;
            tmp_feedback.ui32FeedbackStatus = FB_MMS_NICO;
            TransmitFeedback(tmp_feedback);
        }
    }

    // Nico Steering
    TSignalValue::Data NicoSteering;
    static tTimeStamp NicoSteeringtts = 0;
    if(IS_OK(o_NicoSteeringAngleId.readPin(m_ReaderNicoSteeringAngle, (void *) &NicoSteering, NicoSteeringtts)))
    {
        boost::lock_guard<boost::mutex> lock(m_mutexAction);
        boost::lock_guard<boost::mutex> lockld(m_mutexNico);
        if(m_propEnableDebugging)
        {
            //LOG_INFO("SelectSpeed: rcv Nico steering (%f)", NicoSteering.f32Value);
        }
        if(m_bNico)
        {
            TransmitSteering(NicoSteering);
            TFeedbackStruct::Data tmp_feedback;
            tmp_feedback.ui8FilterId = F_MEGA_MONSTER_SELECTOR;
            tmp_feedback.ui32FeedbackStatus = FB_MMS_NICO;
            TransmitFeedback(tmp_feedback);
        }
    }
    // ---------------- Judith IS THE BEST ----------------------
    // Judith speed
    TSignalValue::Data JudithSpeed;
    static tTimeStamp JudithSpeedtts = 0;
    if(IS_OK(o_JudithSpeedSignalId.readPin(m_ReaderJudithSpeed, (void *) &JudithSpeed, JudithSpeedtts)))
    {
        boost::lock_guard<boost::mutex> lock(m_mutexAction);
        boost::lock_guard<boost::mutex> lockld(m_mutexJudith);
        if(m_propEnableDebugging)
        {
            //LOG_INFO("SelectSpeed: rcv Judith speed (%f)", JudithSpeed.f32Value);
        }
        if(m_bJudith)
        {
            TransmitSpeed(JudithSpeed);
            TFeedbackStruct::Data tmp_feedback;
            tmp_feedback.ui8FilterId = F_MEGA_MONSTER_SELECTOR;
            tmp_feedback.ui32FeedbackStatus = FB_MMS_JUDITH;
            TransmitFeedback(tmp_feedback);
        }
    }

    // Judith Steering
    TSignalValue::Data JudithSteering;
    static tTimeStamp JudithSteeringtts = 0;
    if(IS_OK(o_JudithSteeringAngleId.readPin(m_ReaderJudithSteeringAngle, (void *) &JudithSteering, JudithSteeringtts)))
    {
        boost::lock_guard<boost::mutex> lock(m_mutexAction);
        boost::lock_guard<boost::mutex> lockld(m_mutexJudith);
        if(m_propEnableDebugging)
        {
            //LOG_INFO("SelectSpeed: rcv Judith steering (%f)", JudithSteering.f32Value);
        }
        if(m_bJudith)
        {
            TransmitSteering(JudithSteering);
            TFeedbackStruct::Data tmp_feedback;
            tmp_feedback.ui8FilterId = F_MEGA_MONSTER_SELECTOR;
            tmp_feedback.ui32FeedbackStatus = FB_MMS_JUDITH;
            TransmitFeedback(tmp_feedback);
        }
    }

    {
        // pls staaahp
        boost::lock_guard<boost::mutex> lock(m_mutexAction);
        if(m_Action.ui32Command == AC_MMS_CRAZY_SUPER_FAST_STOP)
        {
            TSignalValue::Data acSpeed;
            acSpeed.f32Value = 0.0f;
            TransmitSpeed(acSpeed);
            TSignalValue::Data acSteering;
            acSteering.f32Value = 0.0f;
            TransmitSteering(acSteering);
            TFeedbackStruct::Data tmp_feedback;
            tmp_feedback.ui8FilterId = F_MEGA_MONSTER_SELECTOR;
            tmp_feedback.ui32FeedbackStatus = FB_MMS_CRAZY_SUPER_FAST_STOP;
            TransmitFeedback(tmp_feedback);
        }
    }

    RETURN_NOERROR;
}

tResult MegaMonsterSelector::TransmitSpeed(TSignalValue::Data transmittedSpeed)
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

tResult MegaMonsterSelector::TransmitSteering(TSignalValue::Data transmittedSteering)
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

tResult MegaMonsterSelector::TransmitFeedback(TFeedbackStruct::Data transmittedFeedback)
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
