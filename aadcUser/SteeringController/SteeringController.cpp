/**********************************************************************
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: �This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS �AS IS� AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: hart#$  $Date:: 2017-05-19 08:12:10#$ $Rev:: 63515   $
**********************************************************************/
#include <adtf3.h>
#include <stdlib.h>
#include "SteeringController.h"
#include "ADTF3_helper.h"
#include "ScmCommunication.h"
#include <property_structs.h>

/// This defines a data triggered filter and exposes it via a plugin class factory.
/// The Triggerfunction cSimpleDataStatistics will be embedded to the Filter
/// and called repeatedly (last parameter of this macro)!
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_STEERINGCONTROLLER_FILTER,
                                    "SteeringController",
                                    SteeringController,
                                    adtf::filter::pin_trigger({"measured_vehicle_steering"}));



//CTOR of the TriggerFuntion
//This is to initialize the Trigger
SteeringController::SteeringController()
{
    //Register Properties
        //RegisterPropertyVariable("wheel circumference in meter", m_f32wheelCircumference);
    RegisterPropertyVariable("proportional factor for PID Controller ", m_f64PIDKp               );
    RegisterPropertyVariable("integral factor for PID Controller", m_f64PIDKi                    );
    RegisterPropertyVariable("differential factor for PID Controller", m_f64PIDKd                );
    RegisterPropertyVariable("sampletime for the pid controller [ms]", m_f64PIDSampleTime             );
    RegisterPropertyVariable("the minimum output value for the controller [%]", m_f64PIDMinimumOutput);
    RegisterPropertyVariable("the maximum output value for the controller [%]", m_f64PIDMaximumOutput);
    RegisterPropertyVariable("show debug output", m_bShowDebug                                   );
    RegisterPropertyVariable("input factor for PT1", m_f64PT1OutputFactor                        );
    RegisterPropertyVariable("time constant for pt1 controller", m_f64PT1TimeConstant            );
    RegisterPropertyVariable("set point is multiplied with this factor", m_f64PT1CorrectionFactor);
    RegisterPropertyVariable("gain factor for PT1 controller", m_f64PT1Gain                      );
    RegisterPropertyVariable("controller type", m_i32ControllerMode                              );

    

    o_TSignalValue.registerPin(this, m_oInputMeasWheelSteering    , "measured_vehicle_steering"     );
        o_TSignalValue.registerPin(this, m_oInputSetWheelSteering    , "desired_vehicle_steering"     );

        o_TSignalValue.registerPin(this, m_oOutputActuator   , "actuator_output");

}


//implement the Configure function to read ALL Properties
tResult SteeringController::Configure()
{
    m_f64LastOutput = 0;
    m_f64LastMeasuredError = 0;
    m_f64SetPoint = 0;
    m_lastSampleTime = 0;
    m_f64LastSpeedValue = 0;
    m_f64accumulatedVariable = 0;
    direction_forwards = tTrue;
    m_ui32ArduinoTimestampSetSteering = 0;
    m_ui32ArduinoTimestampMeasuredSteering = 0;


    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

    //LoadProperties();

    RETURN_NOERROR;
}

///this funtion will be executed each time a trigger occured 
///Due to the change of the data receive events it can be possible that more than one sample was pushed to the 
/// Readers queue. So make sure the execution of this funtion will read ALL Samples of ALL Readers until the queues are empty.
tResult SteeringController::Process(tTimeStamp tmTimeOfTrigger)
{
    TSignalValue::Data tmp_setSteering;
    if(IS_OK(o_TSignalValue.readPin(m_oInputSetWheelSteering, (void *) & tmp_setSteering, m_ui32ArduinoTimestampSetSteering)))
        {
            m_ui32ArduinoTimestampSetSteering = tmp_setSteering.ui32ArduinoTimestamp;
            m_dataSetSteering = tmp_setSteering;
            // process data
            RETURN_IF_FAILED(ProcessSetSteeringInput());
        }

    TSignalValue::Data tmp_measuredSteering;
    if(IS_OK(o_TSignalValue.readPin(m_oInputMeasWheelSteering, (void *) & tmp_measuredSteering, m_ui32ArduinoTimestampMeasuredSteering)))
        {
            m_ui32ArduinoTimestampMeasuredSteering = tmp_measuredSteering.ui32ArduinoTimestamp;
            m_dataMeasuredSteering = tmp_measuredSteering;
            // process data
            RETURN_IF_FAILED(ProcessMeasuredSteeringInput());
        }
        RETURN_NOERROR;
}

tResult SteeringController::LoadProperties(){


      //TEST CASE
        LOG_SUCCESS("here in load properties");
        SteeringController_properties properties;
        LoadXMLData("/home/aadc/AADC/utilities/propertyXMLs/steeringcontroller_properties.xml", (void*) (&properties));

        m_f64PIDKp = properties.m_f64PIDKp;
        m_f64PIDKi = properties.m_f64PIDKi;
        m_f64PIDKd = properties.m_f64PIDKd;
        m_f64PIDSampleTime = properties.m_f64PIDSampleTime;
        m_f64PIDMinimumOutput = properties.m_f64PIDMinimumOutput;
        m_f64PIDMaximumOutput = properties.m_f64PIDMaximumOutput;
        m_f64PT1OutputFactor = properties.m_f64PT1OutputFactor;
        m_f64PT1TimeConstant = properties.m_f64PT1TimeConstant;
        m_f64PT1CorrectionFactor = properties.m_f64PT1CorrectionFactor;
        m_f64PT1Gain = properties.m_f64PT1Gain;
        m_i32ControllerMode = properties.m_i32ControllerMode;
        m_i32AntiWindupMethod = properties.m_i32AntiWindupMethod;
        m_bShowDebug = properties.m_bShowDebug;

    RETURN_NOERROR;


}










tResult SteeringController::ProcessMeasuredSteeringInput()
{
    // write to member variable
    m_f64MeasuredVariable = static_cast<tFloat64>(m_dataMeasuredSteering.f32Value);

//    if(m_bUseSetSpeedDirInfo) {
//            if(direction_forwards) {
//                    m_f64MeasuredVariable = fabsf(m_f64MeasuredVariable);
//            } else {
//                    m_f64MeasuredVariable = fabsf(m_f64MeasuredVariable) * -1.0;
//            }
//    }

    //calculation
    // if speed = 0 is requested output is immediately set to zero
    if (fabsf(m_f64SetPoint) < 0.02)
    {
        m_f64LastOutput = 0;
        m_f64accumulatedVariable = 0;
        m_f64LastMeasuredError = 0;
    }
    else
    {
        m_f64LastOutput = getControllerValue(m_f64MeasuredVariable)*m_f64PT1OutputFactor;
    }
    TSignalValue::Data tmp_outputSteering;
    tmp_outputSteering.f32Value = static_cast<tFloat32>(m_f64LastOutput);
    tmp_outputSteering.ui32ArduinoTimestamp =  m_pClock->GetStreamTime();
    RETURN_IF_FAILED(o_TSignalValue.writePin(m_oOutputActuator, (void *) &tmp_outputSteering, m_pClock->GetStreamTime()));
    RETURN_NOERROR;
}

tResult SteeringController::ProcessSetSteeringInput()
{

    // write to member variable
    m_f64SetPoint = static_cast<tFloat64>(m_dataSetSteering.f32Value);
    if(m_f64SetPoint > 0) {
            direction_forwards = tTrue;
    } else if(m_f64SetPoint < 0) {
            direction_forwards = tFalse;
    }
    if (m_i32ControllerMode == 4)
        m_f64SetPoint = m_f64SetPoint * m_f64PT1CorrectionFactor;

    RETURN_NOERROR;
}


tFloat64 SteeringController::getControllerValue(tFloat64 i_f64MeasuredValue)
{

    //i_f64MeasuredValue = (i_f64MeasuredValue +  m_f64LastSpeedValue) /2.0;

    //m_f64LastSpeedValue = i_f64MeasuredValue;

    tFloat f64Result = 0;

    //the three controller algorithms
    if (m_i32ControllerMode == 1)
    {
        //m_lastSampleTime = GetTime();

        //algorithm:
        //y = Kp * e
        //error:
        tFloat64 f64Error = (m_f64SetPoint - i_f64MeasuredValue);

        f64Result = m_f64PIDKp * f64Error;
    }
    else if (m_i32ControllerMode == 2) //PI- Regler
    {
        //m_lastSampleTime = GetTime();

        //algorithm:
        //esum = esum + e
        //y = Kp * e + Ki * Ta * esum
        //error:
        tFloat64 f64Error = (m_f64SetPoint - i_f64MeasuredValue);
        // accumulated error:

        m_f64accumulatedVariable += (f64Error*m_f64PIDSampleTime);
	

        tFloat64 tmp_result = m_f64PIDKp*f64Error
                +(m_f64PIDKi*m_f64accumulatedVariable);

LOG_INFO("error %f", f64Error);
LOG_INFO("accu %f", m_f64accumulatedVariable);
	LOG_INFO("output %f", tmp_result);
        // Anti Windup
        if(m_i32AntiWindupMethod == 2) {
                        if (tmp_result >= m_f64PIDMaximumOutput) {
                                m_f64accumulatedVariable = 0;
                                if (m_bShowDebug) {
                                        LOG_WARNING(cString::Format("PIDController: Anti windup, Reset mode (max), resetted integral term, tmp_res: %f, max: %f", tmp_result, m_f64PIDMaximumOutput));
                                }
                        } else if (tmp_result <= m_f64PIDMinimumOutput) {
                                m_f64accumulatedVariable = 0;
                                if (m_bShowDebug) {
                                        LOG_WARNING(cString::Format("PIDController: Anti windup, Reset mode (min), resetted integral term, tmp_res: %f, min: %f", tmp_result, m_f64PIDMinimumOutput));
                                }
                    }
        } else if(m_i32AntiWindupMethod == 3) {
                        if (tmp_result >= m_f64PIDMaximumOutput) {
                                m_f64accumulatedVariable += m_f64PIDMaximumOutput - tmp_result;
                                if(m_f64accumulatedVariable < 0) m_f64accumulatedVariable = 0;
                                if (m_bShowDebug) {
                                        LOG_WARNING(cString::Format("PIDController: Anti windup, Stabil mode (max), stabilized integral term, tmp_res: %f, new integral term: %f, max: %f", tmp_result, m_f64accumulatedVariable, m_f64PIDMaximumOutput));
                                }
                        } else if (tmp_result <= m_f64PIDMinimumOutput) {
                                m_f64accumulatedVariable += m_f64PIDMinimumOutput - tmp_result;
                                if(m_f64accumulatedVariable > 0) m_f64accumulatedVariable = 0;
                                if (m_bShowDebug) {
                                        LOG_WARNING(cString::Format("PIDController: Anti windup, Stabil mode (min), stabilized integral term, tmp_res: %f, new integral term: %f, min: %f", tmp_result, m_f64accumulatedVariable, m_f64PIDMinimumOutput));
                                }
                    }
        }

        f64Result = m_f64PIDKp * f64Error
            + (m_f64PIDKi*m_f64accumulatedVariable);


    }
    else if (m_i32ControllerMode == 3)
    {
        m_lastSampleTime = GetTime();
        tFloat64 f64SampleTime = m_f64PIDSampleTime;

        //algorithm:
        //esum = esum + e
        //y = Kp * e + Ki * Ta * esum + Kd * (e � ealt)/Ta
        //ealt = e

        //error:
        tFloat64 f64Error = (m_f64SetPoint - i_f64MeasuredValue);
        // accumulated error:
        m_f64accumulatedVariable += f64Error * m_f64PIDSampleTime;
tFloat64 tmp_result = m_f64PIDKp * f64Error
            + (m_f64PIDKi*m_f64accumulatedVariable)
            + m_f64PIDKd * (f64Error - m_f64LastMeasuredError) / f64SampleTime;


LOG_INFO("error %f", f64Error);
LOG_INFO("accu %f", m_f64accumulatedVariable);
	LOG_INFO("output %f", tmp_result);
	if(m_i32AntiWindupMethod == 2) {
                        if (tmp_result >= m_f64PIDMaximumOutput) {
                                m_f64accumulatedVariable = 0;
                                if (m_bShowDebug) {
                                        LOG_WARNING(cString::Format("PIDController: Anti windup, Reset mode (max), resetted integral term, tmp_res: %f, max: %f", tmp_result, m_f64PIDMaximumOutput));
                                }
                        } else if (tmp_result <= m_f64PIDMinimumOutput) {
                                m_f64accumulatedVariable = 0;
                                if (m_bShowDebug) {
                                        LOG_WARNING(cString::Format("PIDController: Anti windup, Reset mode (min), resetted integral term, tmp_res: %f, min: %f", tmp_result, m_f64PIDMinimumOutput));
                                }
                    }
        } else if(m_i32AntiWindupMethod == 3) {
                        if (tmp_result >= m_f64PIDMaximumOutput) {
                                m_f64accumulatedVariable += m_f64PIDMaximumOutput - tmp_result;
                                if(m_f64accumulatedVariable < 0) m_f64accumulatedVariable = 0;
                                if (m_bShowDebug) {
                                        LOG_WARNING(cString::Format("PIDController: Anti windup, Stabil mode (max), stabilized integral term, tmp_res: %f, new integral term: %f, max: %f", tmp_result, m_f64accumulatedVariable, m_f64PIDMaximumOutput));
                                }
                        } else if (tmp_result <= m_f64PIDMinimumOutput) {
                                m_f64accumulatedVariable += m_f64PIDMinimumOutput - tmp_result;
                                if(m_f64accumulatedVariable > 0) m_f64accumulatedVariable = 0;
                                if (m_bShowDebug) {
                                        LOG_WARNING(cString::Format("PIDController: Anti windup, Stabil mode (min), stabilized integral term, tmp_res: %f, new integral term: %f, min: %f", tmp_result, m_f64accumulatedVariable, m_f64PIDMinimumOutput));
                                }
                    }
        }

        f64Result =  m_f64PIDKp * f64Error
            + (m_f64PIDKi*m_f64accumulatedVariable)
            + m_f64PIDKd * (f64Error - m_f64LastMeasuredError) / f64SampleTime;
	
        m_f64LastMeasuredError = f64Error;
    }
    else if (m_i32ControllerMode == 4)
    {
        /*********************************
        * PT 1 discrete algorithm
        *
        *               Tau
        *       In +    ---  * LastOut
        *             Tsample
        * Out = ---------------------
        *               Tau
        *       1 +     ---
        *             Tsample
        *
        *                           Tau
        * here with     PT1Gain =   ---
        *                         Tsample
        *
        *                  T
        * y(k) = y(k-1) + --- ( v * e(k)  - y(k-1))
        *                  T1
        *
        * e(k):  input
        * y(k-1) : last output
        * v : gain
        * T/T1: time constant
        ********************************************/
        f64Result = m_f64LastOutput + m_f64PT1TimeConstant * (m_f64PT1Gain *
            (m_f64SetPoint - i_f64MeasuredValue) - m_f64LastOutput);

    }
    // checking for minimum and maximum limits
    if (f64Result > m_f64PIDMaximumOutput)
    {
        f64Result = m_f64PIDMaximumOutput;
    }
    else if (f64Result < m_f64PIDMinimumOutput)
    {
        f64Result = m_f64PIDMinimumOutput;
    }

    m_f64LastOutput = f64Result;

    return f64Result;
}

tTimeStamp SteeringController::GetTime()
{
    return adtf_util::cHighResTimer::GetTime();
}

