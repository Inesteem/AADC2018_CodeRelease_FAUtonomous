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
* This filter is used to calculate the average of last three ultrasonic signals.

* $Adapted by:: Xiangfei#  $Date:: 2018-08-01 12:44:00# status: adapted, not tested
**********************************************************************/

#pragma once

#define CID_USTEST_FILTER "ustest.filter.user.aadc.cid"
#include "stdafx.h"
// namespace
using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;

#include "ADTF3_helper.h"
#include "stdafx.h"
#include "aadc_create_mediadescription.h"
#include "aadc_transmit_samples.h"
#include "aadc_read_samples.h"

// ustest
class ustest : public cTriggerFunction
{
private:
        /*------------ STRUCTS -------------*/
        // create structs to hold information we get from SignalValues
        // coding convention: 	o_******Id

        iUltrasonicStruct o_USId;
        iSignalValue o_TSId;


        /*------- SAMPLE FACTORIES ---------*/
        // a factory contains all data samples, which we can load into a tSignalValue
        // plus, it is responsible for (de)code the samples
        // coding convention:	m_******SampleFactory

        adtf::mediadescription::cSampleCodecFactory m_USSampleFactory;
        adtf::mediadescription::cSampleCodecFactory m_TSSampleFactory;

        /*-------------- PINS --------------*/
        // create pins for input and output
        // coding convention:	m_Reader******	for input  signals
        //			m_Writer******	for output signals

        // input pins
        cPinReader m_ReaderUSInput;


        // output pins
        cPinWriter m_WriterUSOutput;


        /*------------ PROPERTIES ----------*/
        // set property variables
        // coding convention:	m_prop******

        //adtf::base::property_variable<tInt>    m_propFilterCount        = tInt(3);  // initial value for number of measurements used for moving average filter
        //adtf::base::property_variable<tBool>    m_propFilterEnable        = tBool(tTrue);  // check if filter is enabled
        /*------------ VARIABLES -----------*/
        // coding convention:	m_******

        object_ptr<IStreamType> pUSDataType;
        object_ptr<IStreamType> pTSDataType;

        tBool m_bFilterEnable;
        object_ptr<adtf::services::IReferenceClock> m_pClock; // clock for timestamps
        tInt nSize_USStruct;// memory sizes of media samples to create


        //tempUltrasonicStruct temp_US_struct;
        //Average Lists
//        std::list<float>  US_Mean_SideLeft;
//        std::list<float>  US_Mean_SideRight;
//        std::list<float>  US_Mean_RearLeft;
//        std::list<float>  US_Mean_RearCenter;
//        std::list<float>  US_Mean_RearRight;
public:
        /*------------ FUNCTIONS -----------*/
        // constructor
        ustest();

        // destructor
        ~ustest() = default;

        // configure
        virtual tResult Configure() override;

        // process
        virtual tResult Process(tTimeStamp tmTimeOfTrigger) override;

        // transmit ultrasonic struct
        tResult transmitUSStruct(tUltrasonicStruct inputSignal);

//        //
//        tUltrasonicStruct CalcUSMeanValue(tUltrasonicStruct receivedUSData);

}; // ustest
