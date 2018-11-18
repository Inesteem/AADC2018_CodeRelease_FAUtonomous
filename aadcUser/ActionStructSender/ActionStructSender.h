/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. Team FAUtonomous. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

**********************************************************************/

#pragma once
#define CID_ACTION_STRUCT_SENDER_FILTER "actionstruct_sender.filter.user.aadc.cid"

// namespaces
using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;

#include "stdafx.h"
// ActionStructSender
class ActionStructSender : public cTriggerFunction
{
private:
    /*------------ STRUCTS -------------*/
    // create structs to hold information we get from SignalValues
    // coding convention: 	o_******Id

//    iSignalValue  o_SpeedSignalId;
//    iActionStruct o_ActionId;
    TSignalValue m_TSignalValue;
    TActionStruct m_TActionStruct;
    TPoseStruct m_TPoseStruct;


    /*-------------- PINS --------------*/
    // create pins for input and output
    // coding convention:	m_Reader******	for input  signals
    //			            m_Writer******	for output signals

    // input pins
    cPinReader m_ReaderSpeed;

    // output pins
    cPinWriter m_WriterAction1;
    cPinWriter m_WriterAction2;
    cPinWriter m_WriterAction3;

    cPinWriter m_WriterCarPose1;


    object_ptr<adtf::services::IReferenceClock> m_pClock; // clock for timestamps
//    adtf::base::property_variable<tUInt>   m_propFilterId = tUInt(15);
//    adtf::base::property_variable<tBool>   m_propEnabled  = tBool(tTrue);
//    adtf::base::property_variable<tBool>   m_propStarted  = tBool(tTrue);
//    adtf::base::property_variable<tUInt32> m_propCommand  = tUInt32(15010);


    TActionStruct::Data m_action1;
    TActionStruct::Data m_action2;
    TActionStruct::Data m_action3;

    TPoseStruct::Data m_carPose1;

    tUInt32 m_ui32TimeGapAction;
    tUInt32 m_ui32TimeLast;
    tUInt32 m_ui32TimeNow;

public:
    /*------------ FUNCTIONS -----------*/
    /*------------ FROM AUDI -----------*/
    // constructor
    ActionStructSender();

    // destructor
    ~ActionStructSender() = default;

    // Configure
    virtual tResult Configure() override;

    // Process
    virtual tResult Process(tTimeStamp tmTimeOfTrigger) override;

    // ---------- FROM USER ------------*/
    // TransmitSpeed
//    tResult TransmitAction(tFloat32 inputSpeed);

}; // ActionStructSender
