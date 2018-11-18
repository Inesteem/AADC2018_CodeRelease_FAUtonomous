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
* This filter calculates and sends out the average of the last three ultrasonic signals.

* $Adapted by:: Xiangfei#  $Date:: 2018-08-30 12:44:00#
**********************************************************************/

#pragma once

#define CID_ULTRASONIC_MEAN_FILTER_FILTER "ultrasonic_mean_filter.filter.user.aadc.cid"

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
#include <boost/thread.hpp>

// UltrasonicMeanFilter
class UltrasonicMeanFilter : public cTriggerFunction
{
private:
        /*------------ STRUCTS -------------*/
        // create structs to hold information we get from SignalValues
        // coding convention: 	o_******Id

        TUltrasonicStruct o_UltrasonicStruct;


        /*-------------- PINS --------------*/
        // create pins for input and output
        // coding convention:	m_Reader******	for input  signals
        //			m_Writer******	for output signals

        // input pins
        cPinReader m_ReaderUltrasonic;

        // output pins
        cPinWriter m_WriterUltrasonic;

        /*------------ PROPERTIES ----------*/
        // set property variables
        // coding convention:	m_prop******

        adtf::base::property_variable<tInt>    m_propIntFilterCount        = tInt(3);  // initial value for number of measurements used for moving average filter
        adtf::base::property_variable<tBool>    m_propBoolFilterEnable        = tBool(tTrue);  // check if filter is enabled
        /*------------ VARIABLES -----------*/
        // coding convention:	m_******

        tBool m_bFilterEnable;
        object_ptr<adtf::services::IReferenceClock> m_pClock; // clock for timestamps


        //tempUltrasonicStruct temp_US_struct;
        //Average Lists
        std::list<float>  m_listMeanSideLeft;
        std::list<float>  m_listMeanSideRight;
        std::list<float>  m_listMeanRearLeft;
        std::list<float>  m_listMeanRearCenter;
        std::list<float>  m_listMeanRearRight;

        //init timestamps
        tUInt32 m_ui32TimeStampUS;


// critical sections
     boost::mutex m_oProcessUltrasonicInput;
public:
        /*------------ FUNCTIONS -----------*/
        // constructor
        UltrasonicMeanFilter();

        // destructor
        ~UltrasonicMeanFilter() = default;

        // configure
        virtual tResult Configure() override;

        // process
        virtual tResult Process(tTimeStamp tmTimeOfTrigger) override;

        // calculate average value of ultrasonic signals
        TUltrasonicStruct::Data CalcUSMeanValue(TUltrasonicStruct::Data receivedUSData);

}; // UltrasonicMeanFilter
