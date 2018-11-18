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
#include "SpeedController.h"
#include "ADTF3_helper.h"
#include "ScmCommunication.h"
#include <property_structs.h>

/// This defines a data triggered filter and exposes it via a plugin class factory.
/// The Triggerfunction cSimpleDataStatistics will be embedded to the Filter
/// and called repeatedly (last parameter of this macro)!
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_SPEEDCONTROLLER_FILTER,
                                    "SpeedController",
                                    SpeedController,
                                    adtf::filter::pin_trigger({"measured_vehicle_speed"}));

                                    //adtf::filter::thread_trigger(tTrue));

//CTOR of the TriggerFuntion
//This is to initialize the Trigger
SpeedController::SpeedController()
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



    o_TSignalValue.registerPin(this, m_oInputMeasWheelSpeed    , "measured_vehicle_speed"     );
        o_TSignalValue.registerPin(this, m_oInputSetWheelSpeed    , "desired_vehicle_speed"     );
        o_TActionStruct.registerPin(this, m_ReaderAction    , "action_input"     );

        o_TSignalValue.registerPin(this, m_oOutputActuator   , "actuator_output");
    o_TSignalValue.registerPin(this, m_desiredAcc   , "desired_acc");
}


//implement the Configure function to read ALL Properties
tResult SpeedController::Configure()
{
    m_f64LastOutput = 0;
    m_f64LastMeasuredError = 0;
    m_f64SetPoint = 0;
    m_lastSampleTime = 0;
    m_f64LastSpeedValue = 0;
    m_f64accumulatedVariable = 0;
    direction_forwards = tTrue;
    m_ui32ArduinoTimestampSetSpeed = 0;
    m_ui32ArduinoTimestampMeasuredSpeed = 0;


    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

   // LoadProperties();

    RETURN_NOERROR;
}

///this funtion will be executed each time a trigger occured
///Due to the change of the data receive events it can be possible that more than one sample was pushed to the
/// Readers queue. So make sure the execution of this funtion will read ALL Samples of ALL Readers until the queues are empty.
tResult SpeedController::Process(tTimeStamp tmTimeOfTrigger)
{

    TSignalValue::Data tmp_setSpeed;
    if(IS_OK(o_TSignalValue.readPin(m_oInputSetWheelSpeed, (void *) & tmp_setSpeed, m_ui32ArduinoTimestampSetSpeed)))
        {
            m_ui32ArduinoTimestampSetSpeed = tmp_setSpeed.ui32ArduinoTimestamp;
            m_dataSetSpeed = tmp_setSpeed;
            // process data
            RETURN_IF_FAILED(ProcessSetSpeedInput());
        }

    TSignalValue::Data tmp_measuredSpeed;
    if(IS_OK(o_TSignalValue.readPin(m_oInputMeasWheelSpeed, (void *) & tmp_measuredSpeed, m_ui32ArduinoTimestampMeasuredSpeed)))
        {
            m_dataLastSpeed = m_dataCurrentSpeed;
            m_ui32ArduinoTimestampMeasuredSpeed = tmp_measuredSpeed.ui32ArduinoTimestamp;
            m_dataMeasuredSpeed = tmp_measuredSpeed;
            // process data
            RETURN_IF_FAILED(ProcessMeasuredSpeedInput());
            // TransmitAcc();
        }
        TActionStruct::Data tmp_action;
        if(IS_OK(o_TActionStruct.readPin(m_ReaderAction, (void *) & tmp_action, m_ui32ArduinoTimestampAction)))
            {
                m_ui32ArduinoTimestampAction = tmp_action.ui32ArduinoTimestamp;
                m_dataAction = tmp_action;
                // process data
            }
        RETURN_NOERROR;
}




// tResult SpeedController::TransmitAcc()
// {
//     TSignalValue::Data tmp_acc;
//     tFloat32 time_step = (m_dataCurrentSpeed.ui32ArduinoTimestamp - m_dataLastSpeed.ui32ArduinoTimestamp) / 1000000.0;
//     tFloat32 speed_step = m_dataCurrentSpeed.f32Value - m_dataLastSpeed.f32Value;
//     if (time_step > 0.2)
//     {
//         tmp_acc.f32Value = 0;
//         tmp_acc.ui32ArduinoTimestamp = m_pClock->GetStreamTime();
//     }
//
//     else
//     {
//        tmp_acc.f32Value = speed_step / time_step;
//         tmp_acc.ui32ArduinoTimestamp = m_pClock->GetStreamTime();
//     }
//     o_TSignalValue.writePin(m_desiredAcc, (void *) &tmp_acc, m_pClock->GetStreamTime());
//     RETURN_NOERROR;
// }





tResult SpeedController::LoadProperties(){


      //TEST CASE
        LOG_SUCCESS("here in load properties");
        speedcontroller_properties properties;
        LoadXMLData("/home/aadc/AADC/utilities/propertyXMLs/speedcontroller_properties.xml", (void*) (&properties));

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
        LOG_SUCCESS(cString::Format("m_f64PIDKp: %f", (float)m_f64PIDKd));
        LOG_SUCCESS(cString::Format("m_f64PIDKi: %f", (float)m_f64PIDKi));
        LOG_SUCCESS(cString::Format("m_f64PIDKd: %f", (float)m_f64PIDKd));

    RETURN_NOERROR;


}










tResult SpeedController::ProcessMeasuredSpeedInput()
{
    // write to member variable
    m_f64MeasuredVariable = static_cast<tFloat64>(m_dataMeasuredSpeed.f32Value);

//    if(m_bUseSetSpeedDirInfo) {
//            if(direction_forwards) {
//                    m_f64MeasuredVariable = fabsf(m_f64MeasuredVariable);
//            } else {
//                    m_f64MeasuredVariable = fabsf(m_f64MeasuredVariable) * -1.0;
//            }
//    }

    //calculation
    // if speed = 0 is requested output is immediately set to zero
    if(m_dataAction.ui32Command == AC_SPC_SPEED_RESET)
    {
      m_f64LastOutput = 0;
      m_f64accumulatedVariable = 0;
      m_f64LastMeasuredError = 0;

      m_dataAction.ui32Command = AC_SPC_SPEED_NORMAL;
    }



    if (fabsf(m_f64SetPoint) < 0.02 )
    {
        m_f64LastOutput = 0;
        m_f64accumulatedVariable = 0;
        m_f64LastMeasuredError = 0;
    }
    else
    {
        m_f64LastOutput = getControllerValue(m_f64MeasuredVariable)*m_f64PT1OutputFactor;
    }
    TSignalValue::Data tmp_outputSpeed;
    tmp_outputSpeed.f32Value = static_cast<tFloat32>(m_f64LastOutput);
    tmp_outputSpeed.ui32ArduinoTimestamp =  m_pClock->GetStreamTime();


    m_dataCurrentSpeed = tmp_outputSpeed;



    RETURN_IF_FAILED(o_TSignalValue.writePin(m_oOutputActuator, (void *) &tmp_outputSpeed, m_pClock->GetStreamTime()));
    RETURN_NOERROR;
}

tResult SpeedController::ProcessSetSpeedInput()
{

    // write to member variable
    m_f64SetPoint = static_cast<tFloat64>(m_dataSetSpeed.f32Value);
    if(m_f64SetPoint > 0) {
            direction_forwards = tTrue;
    } else if(m_f64SetPoint < 0) {
            direction_forwards = tFalse;
    }
    if (m_i32ControllerMode == 4)
        m_f64SetPoint = m_f64SetPoint * m_f64PT1CorrectionFactor;

    RETURN_NOERROR;
}


tFloat64 SpeedController::getControllerValue(tFloat64 i_f64MeasuredValue)
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

//LOG_INFO("error %f", f64Error);
//LOG_INFO("accu %f", m_f64accumulatedVariable);
        //LOG_INFO("output %f", tmp_result);
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


//LOG_INFO("error %f", f64Error);
//LOG_INFO("accu %f", m_f64accumulatedVariable);
        //LOG_INFO("output %f", tmp_result);
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

tTimeStamp SpeedController::GetTime()
{
    return adtf_util::cHighResTimer::GetTime();
}
