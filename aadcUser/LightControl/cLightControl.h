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
#define CID_LIGHT_CONTORL_DATA_TRIGGERED_FILTER "light_control.filter.user.aadc.cid"

using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;

#include "ADTF3_helper.h"
#include "stdafx.h"
#include <boost/thread.hpp>

class LightControl : public cTriggerFunction
{
private:
//neue schreibweise (rest in Klassen gekapselt):
    TSignalValue m_SignalValue;
    TBoolSignalValue m_BoolValue;
    TActionStruct m_ActionStruct;





    /*! Reader of an InPin. */
    cPinReader		actionInput;
    cPinReader		carspeedInput;
    cPinReader		setspeedInput;

    cPinWriter		headLightOutput;
    cPinWriter		reverseLightOutput;
    cPinWriter		brakeLightOutput;
    cPinWriter		turnRightLightOutput;
    cPinWriter		turnLeftLightOutput;
    cPinWriter		hazardLightOutput;

    // filter properties
    property_variable<tFloat32> factorEnableBrake;
    property_variable<tFloat32> factorDisableBrake;
    property_variable<tUInt32> averageSampleCount;
    property_variable<tFloat32> diffSpeedBrakeEnable;

    // save car speed values
    std::list<tFloat32> last_carspeeds;
    tFloat acc_speed;

    // save brake light status
    tBool brakelightEnabled;

    // save reverse light status
    tBool reverselightEnabled;

public:


    tBool HeadLight;
    tBool ReverseLight;
    tBool BrakeLight;
    tBool TurnRightLight;
    tBool TurnLeftLight;
    tBool HazardLight;
    int ManuallySetBrakeLight;

    /*! The reference clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;

    boost::mutex  cs_inputSpeed, cs_inputSetSpeed, cs_inputAction;

    /*! Default constructor. */
    LightControl();

    /*! Destructor. */
    virtual ~LightControl() = default;

    /**
    * Overwrites the Configure
    * This is to Read Properties prepare your Trigger Function
    */
    tResult Configure() override;
    /**
    * Overwrites the Process
    * You need to implement the Reading and Writing of Samples within this function
    * MIND: Do Reading until the Readers queues are empty or use the IPinReader::GetLastSample()
    * This FUnction will be called if the Run() of the TriggerFunction was called.
    */
    tResult Process(tTimeStamp tmTimeOfTrigger) override;

    tResult ProcessSpeedInput(tTimeStamp &tmTimeOfTrigger);

    tResult ProcessSetSpeedInput(tTimeStamp &tmTimeOfTrigger);

    tResult ProcessAction(tTimeStamp &tmTimeOfTrigger);

};


//*************************************************************************************************
