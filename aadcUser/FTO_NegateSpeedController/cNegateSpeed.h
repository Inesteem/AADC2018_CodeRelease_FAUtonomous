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
* $Author:: spiesra $  $Date:: 2017-05-12 09:34:53#$ $Rev:: 63109   $
* $changedBy:: NicoBenndorf $  $Date:: 2018-07-25 12:41:00#$ Ported to ADTF3  $
**********************************************************************/


#pragma once

#define CID_FTO_NEGATE_SPEED_CONTROLLER_FILTER "fto_negate_speed_controller.filter.user.aadc.cid"
#include "stdafx.h"

// namespace
using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;

// FTO_NegateSpeedController
class cNegateSpeed : public cTriggerFunction
{
private:
        /*------------ STRUCTS -------------*/
        // create structs to hold information we get from SignalValues
        // coding convention: 	o_******Id

         struct speedSignalValueStruct
            {
                    tSize ui32Timestamp;
                    tSize f32Value;
            } o_SpeedSignalId;


        /*------- SAMPLE FACTORIES ---------*/
        // a factory contains all data samples, which we can load into a tSignalValue
        // plus, it is responsible for (de)code the samples
        // coding convention:	m_******SampleFactory

        adtf::mediadescription::cSampleCodecFactory m_SpeedSampleFactory;



        /*-------------- PINS --------------*/
        // create pins for input and output
        // coding convention:	m_Reader******	for input  signals
        //			m_Writer******	for output signals

        // input pins
        cPinReader m_ReaderSpeedController;


        // output pins
        cPinWriter m_WriterCarSpeed;



        /*------------ PROPERTIES ----------*/
        // set property variables
        // coding convention:	m_prop******

        adtf::base::property_variable<tFloat32> m_propWheelBase;// = tFloat( 60.0f);
        adtf::base::property_variable<tFloat32> m_propTrackWidth;
        adtf::base::property_variable<tFloat32> m_propWheelCircumferences;
        adtf::base::property_variable<tFloat32> m_propSteeringCorrection;
        adtf::base::property_variable<tFloat32> m_propOdometerTicks;
        adtf::base::property_variable<tInt32>   m_propInitCounter;
        adtf::base::property_variable<tBool>    m_propResetPose;
        adtf::base::property_variable<tInt32>   m_propMaxSpeedControllerOut;

        /*------------ VARIABLES -----------*/
        // coding convention:	m_******
        //object_ptr<adtf::services::IReferenceClock> m_pClock; // clock for timestamps
        tSignalValue speedOut;
        tSignalValue speedController;
        TSignalValue carSpeedOut;
        tFloat32 limitSpeedControllerOutput;
        std::mutex m_mutexCommandActivatedFlag; // mutex for read and writes in TransmitSpeed
        object_ptr<adtf::services::IReferenceClock> m_pClock; // clock for timestamps
public:
        /*------------ FUNCTIONS -----------*/
        // constructor
        cNegateSpeed();

        // destructor
        ~cNegateSpeed() = default;

        // Configure
        virtual tResult Configure() override;

        // Process
        virtual tResult Process(tTimeStamp tmTimeOfTrigger) override;

        //virtual tResult ProcessCarSpeedInput(ISample);
        // TransmitSpeed
        tResult TransmitSpeed();
        tResult ProcessCarSpeedInput(tSignalValue pReadSpeedSample);

}; // cNegateSpeed
