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
* This filter works as a PID controller.

* $Adapted by:: Xiangfei#  $Date:: 2018-08-01 12:44:00#
**********************************************************************/

#pragma once

#define CID_PID_CONTROLLER_FILTER "new_pid_controller.filter.user.aadc.cid"

#define POS_MAX_SPEED 21
#define NEG_MAX_SPEED 19
#define POS_MIN_SPEED 7
#define NEG_MIN_SPEED 5
#define STD_DEV_SPEED 0.1


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

//#define DEBUG


enum WINDUP { SCALE };
enum PID_CTRL { INT_SUM_SCALE_1, INT_SUM_SCALE_2, INT_SUM_LIMIT_NUM, INT_SUM_BOUNDED };

// cPIDController
class cPIDController : public cTriggerFunction
{
private:

    TSignalValue signal_value;
    TBoolSignalValue bool_value;

    /*-------------- PINS --------------*/
    // create pins for input and output
    // coding convention:	m_Reader******	for input  signals
    //			m_Writer******	for output signals

    // input pins
    cPinReader m_ReaderMeasuredSpeed;
    cPinReader m_ReaderSetPoint;
    cPinReader m_ReaderReset;

    // output pins
    cPinWriter m_WriterManipulatedSpeed;

    /*------------ PROPERTIES ----------*/
    // set property variables
    // coding convention:	m_prop******
    // proportional factor for PID Controller
    adtf::base::property_variable<tFloat32>     m_propF32PIDKP;
    // integral factor for PID Controller
    adtf::base::property_variable<tFloat32>     m_propF32PIDKI;
    // differential factor for PID Controller
    adtf::base::property_variable<tFloat32>     m_propF32PIDKD;

    // differential factor for PID Controller
    adtf::base::property_variable<tFloat32>     m_propF32MaxErr;

    // the sampletime for the pid controller
    // the maximum output value for the controller
    adtf::base::property_variable<tInt32>     m_propI32PIDMaxOutput;
    // the minimum output value for the controller
    adtf::base::property_variable<tInt32>     m_propI32PIDMinOutput;

    adtf::base::property_variable<tFloat32>      m_f64hungryTerm;

    //  When Setpoint is zero, Output is set to zeros
    adtf::base::property_variable<tBool>        m_propBPIDSetZero;

    /*------------ VARIABLES -----------*/
    // coding convention:	m_******
    object_ptr<adtf::services::IReferenceClock> m_pClock; // clock for timestamps

    tFloat32 m_f32CarStoppedThreshold;                    // speed in m/s that is seen as 'car stopped'
    std::mutex m_mutexCommandActivatedFlag;               // mutex for read and writes in m_propCommandActivated

    TSignalValue::Data m_dataManipulatedSpeedOut;

    /*! holds the accumulatedVariable for the controller */

    tUInt32 m_negSpeedInt;
    tUInt32 m_posSpeedInt;

    // True: Use car speed direction info based on set speed input, False: use measured input for direction info
    tBool	m_bUseSetSpeedDirInfo;
    tBool	need_reset = false;

    // the set point is multiplied with this factor, otherwise the set point is not reached by the controller.
    tFloat64 m_f64PT1CorrectionFactor;



    // defines car direction based on set speed
    tBool m_BDirectionForwards;

    //for read tsignalvalue
    tTimeStamp m_ui32TimeStampSpeed;
    tTimeStamp m_ui32TimeStampSetPoint;


    // holds the errors over the past n samples
    std::list<tFloat64> errors;
    tFloat64 m_f64LastError = 0;
    // holds the accumulated error (Integral Part)
    tFloat64 m_f64AccError = 0;
    // holds the accumulated error (Dev Part) // TODO: may not be needed as member var
    tFloat64 m_f64DevError = 0;
    // holds the last measured speed value
    tFloat64 m_f64MeasuredSpeed;
    // holds the last set speed value
    tFloat64 m_f64SetSpeed;
    //max_intregral_value
    tFloat64 m_f64MaxIntegralValue;

// critical sections
        boost::mutex cs_setSpeed, cs_measSpeed, cs_transmit, cs_manipulate;

    cFilename debugfile;

public:
    /*------------ FUNCTIONS -----------*/
    // constructor
    cPIDController();

    // destructor
    ~cPIDController() = default;

    // Configure
    virtual tResult Configure() override;

    // Process
    virtual tResult Process(tTimeStamp tmTimeOfTrigger) override;
    tResult ProcessSetSpeed();
    tResult ProcessMeasuredSpeed();
    tResult ProcessReset();

    //Getter and Setter
    tFloat64 GetSetSpeed();
    tFloat64 GetMeasuredSpeed();

    //Stuff
    tResult TransmitManipulatedSpeed(tFloat32 manipulated_speed);
    tResult ManipulateSpeed();
    tResult CalculateSquaredErrorMean(tFloat64 set, tFloat64 real);

}; // cPIDController
