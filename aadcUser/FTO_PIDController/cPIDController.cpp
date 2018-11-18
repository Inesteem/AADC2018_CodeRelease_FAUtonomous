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

#include <mutex>
#include "stdafx.h"
#include "cPIDController.h"
#include "ScmCommunication.h"
#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <algorithm>
#include <common_helper.h>



/* The following values are measured in a
 * empirical way; just check the output of
 * carPose when speed is set manually to
 * -x to 0 (neg_speed_to_mps) or
 * 0 to x (pos_speed_to_mps)
 * here only a realistic range from -18 to
 * 20 is considered, since the car is
 * to fast to handle it secure above or below
*/
tFloat32 pos_speed_to_mps[POS_MAX_SPEED] =
      {0, 0, 0, 0, 0, 0, 0.55, // 0 -> 6 car doesnt move
       0.60, 0.89, 1.12, 1.41,
       1.42, 1.71, 1.97, 2.21,
       2.47, 2.70, 2.90, 3.10,
       3.31, 3.48};

tFloat32 neg_speed_to_mps[NEG_MAX_SPEED] =
      {0, 0, 0, 0, 0.7, // 0 -> -4 car doesnt move
       0.77, 1.07, 1.42, 1.71,
       2.02, 2.03, 2.32, 2.56,
       2.79, 3.05, 3.22, 3.39,
       3.56, 3.69};

//abschätzung nach oben für I-Term
//I_t+1 = hungry * I_t + max_err    <- convergiert wenn t->oo
//linear for max_error!, so this is exemplary calculated for
// max_err = 1, for err = x einfach malnehmen
tFloat32 min_hungry = 0.7;
tFloat32 step_size = 0.01;
tFloat32 max_integral_val [30] = {
           3.33333, 3.44828, 3.57143, 3.7037, 3.84615,
           4, 4.16667, 4.34783, 4.54545, 4.7619,
           5, 5.26316, 5.55556, 5.88235, 6.25,
           6.66667, 7.14286, 7.69231, 8.33333, 9.09091,
           10, 11.1111, 12.5, 14.2857, 16.6667,
           20, 25, 33.3333, 50, 100};



tFloat32 get_max(tFloat32 hungry,tFloat32 max_error){
    if(hungry < min_hungry ) return 0;
    if(hungry >= 1.0 - step_size) return max_integral_val[29]; // should not happen
    tFloat32 idx = ((hungry - min_hungry)) / step_size;
    tFloat32 diff = (idx - (tUInt32)idx);
    tFloat32 diff2 = max_integral_val[(tUInt32)idx + 1] - max_integral_val[(tUInt32)idx];

    return max_error*(max_integral_val[(tUInt32) idx] + diff * diff2);
}


#define ABS_MIN_SPEED  (-(neg_speed_to_mps[NEG_MAX_SPEED-1]+STD_DEV_SPEED))
#define ABS_MAX_SPEED  (pos_speed_to_mps[POS_MAX_SPEED-1]+STD_DEV_SPEED)

tFloat32 get_pos_mps_speed(tFloat32 speed){ //speed in [0, 10]
    if(speed >= pos_speed_to_mps[POS_MAX_SPEED-1]){
            return POS_MAX_SPEED-1;
    }
    if(speed < pos_speed_to_mps[POS_MIN_SPEED-1])
        return 0;
    int pos = POS_MIN_SPEED-1;

    while(speed >= pos_speed_to_mps[pos])
        ++pos;
    tFloat32 speed_diff = pos_speed_to_mps[pos] - pos_speed_to_mps[pos-1];
    tFloat32 ratio = (speed - pos_speed_to_mps[pos-1])/speed_diff;
    return (tFloat32)(pos-1) + ratio;
}

tFloat32 get_neg_mps_speed(tFloat32 speed){ //speed in [0, -10]
    if(speed <= -neg_speed_to_mps[NEG_MAX_SPEED-1]){
            return 1-NEG_MAX_SPEED;
    }
    if(speed > -neg_speed_to_mps[NEG_MIN_SPEED-1])
        return 0;

    int pos = NEG_MIN_SPEED-1;
        speed = -speed;

    while(speed >= neg_speed_to_mps[pos])
        ++pos;
    tFloat32 speed_diff = neg_speed_to_mps[pos] - neg_speed_to_mps[pos-1];
    tFloat32 ratio = (speed - neg_speed_to_mps[pos-1])/speed_diff;
    return -((tFloat32)(pos-1) + ratio);


}




// This will define the filter and expose it via plugin class factory.
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(
        CID_PID_CONTROLLER_FILTER,                         // references to header file
        "cPIDController",                                   // label
        cPIDController,                                      // class
        adtf::filter::pin_trigger({"measuredSpeed"}));	// set trigger pin


// -------------------------------
// --------- CONSTRUCTOR ---------
// -------------------------------
// in the constructor, we will set the adtf streamtypes for signal values and pins
//#define DEBUGTOFILE

cPIDController::cPIDController()
{

    // ------------------------------------------
    SetName("cPIDController Constructor");

    // -----------------------------------------
    // INPUT BINHs
    signal_value.registerPin(this, m_ReaderMeasuredSpeed      , "measuredSpeed"   );
    signal_value.registerPin(this, m_ReaderSetPoint           , "setPoint"        );
    bool_value.registerPin(this, m_ReaderReset           , "reset"        );

    //OUTPUT BINHs
    signal_value.registerPin(this, m_WriterManipulatedSpeed   , "manipulatedSpeed");



    debugfile = add_date_to_filename("/home/aadc/AADC/src/aadcUser/FTO_PIDController/debug/pidc","txt").c_str();
    // -----------------------------------------
    // set property variables
    // a good heuristic is (static measurements with min squared error):
    // - p ~ 0.5, better smaller
    // - i ~ 1/2p
    // - d ~ 1/(2.5)
    m_propF32PIDKP                 = tFloat32(0.095f);
    m_propF32PIDKI                 = tFloat32(3.f);
    m_propF32PIDKD                 = tFloat32(0.5f);
    m_propF32MaxErr                = tFloat32(1.5f);

    m_propI32PIDMaxOutput          = 16;
    m_propI32PIDMinOutput          = -13;

    m_propBPIDSetZero            = tBool(tTrue);


    // set threshold
    m_f32CarStoppedThreshold = tFloat(0.5f);
    m_f64MeasuredSpeed = 0;
    m_BDirectionForwards = tTrue;
    m_bUseSetSpeedDirInfo = tTrue;
    m_f64SetSpeed = 0;
    m_ui32TimeStampSpeed = 0;
    m_ui32TimeStampSetPoint = 0;
    m_f64hungryTerm = 0.98;
    RegisterPropertyVariable("PID::Kp_value"                , m_propF32PIDKP               );
    RegisterPropertyVariable("PID::Ki_value"                , m_propF32PIDKI               );
    RegisterPropertyVariable("PID::Kd_value"                , m_propF32PIDKD               );
    RegisterPropertyVariable("default_error (guessed)"      , m_propF32MaxErr      );
    RegisterPropertyVariable("PID::Maxiumum output"         , m_propI32PIDMaxOutput        );
    RegisterPropertyVariable("PID::Minimum output"          , m_propI32PIDMinOutput        );
    RegisterPropertyVariable("PID::Setpoint Zero - Output Zero"  , m_propBPIDSetZero            );
    RegisterPropertyVariable("kill old acc errors"       , m_f64hungryTerm      );


}



// ------------------------------
// --------- CONFIGURE ----------
// ------------------------------
// implement configure() to read ALL properties
tResult cPIDController::Configure() {

    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    //m_propI32PIDMinOutput
    if(std::abs(m_propI32PIDMinOutput) >= NEG_MAX_SPEED){
        LOG_WARNING(cString::Format("min neg Output out of range %d/%d; resetting", m_propI32PIDMinOutput, NEG_MAX_SPEED));
        m_propI32PIDMinOutput = 1 - NEG_MAX_SPEED;
    }
    if(m_propI32PIDMaxOutput >= POS_MAX_SPEED){
        LOG_WARNING(cString::Format("max pos Output out of range %d/%d; resetting", m_propI32PIDMaxOutput, POS_MAX_SPEED));
        m_propI32PIDMaxOutput = POS_MAX_SPEED - 1;
    }
    if(m_propI32PIDMinOutput > 0){
        LOG_WARNING(cString::Format("min neg Output not negative, negating!"));
        m_propI32PIDMinOutput = -m_propI32PIDMinOutput;
    }
    if(m_propI32PIDMaxOutput < 0){
        LOG_WARNING(cString::Format("max pos Output negative, negating!"));
        m_propI32PIDMaxOutput = -m_propI32PIDMaxOutput;
    }

    m_negSpeedInt = (-m_propI32PIDMinOutput) - NEG_MIN_SPEED;
    m_posSpeedInt = m_propI32PIDMaxOutput - POS_MIN_SPEED;


    m_f64hungryTerm = abs(m_f64hungryTerm);
    if(m_f64hungryTerm >= 1){
        m_f64hungryTerm = 0.99;
        LOG_WARNING(cString::Format("m_f64hungryTerm not in range, setting to 0.99"));
    }

    m_f64MaxIntegralValue = get_max(m_f64hungryTerm, m_propF32MaxErr);



#ifdef DEBUGTOFILE
    LOG_INFO(cString::Format("max interval: %f" ,m_f64MaxIntegralValue));
    LOG_INFO(cString::Format("pos vals prop: %d - %d = %d", m_propI32PIDMaxOutput , POS_MIN_SPEED, m_posSpeedInt));
    LOG_INFO(cString::Format("neg vals prop: %d - %d = %d", NEG_MIN_SPEED , m_propI32PIDMinOutput, m_negSpeedInt));
    std::fstream file;
    file.open(debugfile, ios::out | ios::app);
    file << "max interval: " << m_f64MaxIntegralValue << "  of hungryTerm  " << m_f64hungryTerm << "\n";
    file << "neg vals prop: " << NEG_MIN_SPEED << " - " << m_propI32PIDMinOutput << " = " << m_negSpeedInt << "\n";
    file << "pos vals prop: " << m_propI32PIDMaxOutput << " - " << POS_MIN_SPEED << " = " << m_posSpeedInt << "\n";
   // file << "(Squared Mean Error -- #Errors -- PIDKP -- PIDKI -- PIDKD -- Controller Type -- Windup\n";
    file.close();
#endif

    RETURN_NOERROR;
}


// ------------------------------
// ---------- PROCESS -----------
// ------------------------------

tResult cPIDController::Process(tTimeStamp)
{
    if (IS_OK(ProcessMeasuredSpeed()))
    {
        ManipulateSpeed();
        ProcessSetSpeed();
        ProcessReset();
    }
    //} else {
     //   LOG_WARNING("some strange black magic is happening here 0_0");
   // }

    //else if (IS_OK(ProcessSetSpeed()))
   // {
   // }

      RETURN_NOERROR;
}



tResult cPIDController::ProcessMeasuredSpeed()
{
    boost::lock_guard<boost::mutex> lock(cs_measSpeed);

    tResult res;
    TSignalValue::Data inputSpeed;

    if(IS_FAILED(res = signal_value.readPin(m_ReaderMeasuredSpeed, (void *) &inputSpeed, m_ui32TimeStampSpeed))){
        RETURN_ERROR(res);
    }
#ifdef DEBUG
    //if(inputSpeed.f32Value > 0.001 &&  m_f64DevError)
     //   LOG_SUCCESS(cString::Format("PID M IN: %f",inputSpeed.f32Value));
#endif

    m_ui32TimeStampSpeed = inputSpeed.ui32ArduinoTimestamp;
    m_f64MeasuredSpeed = static_cast<tFloat64>(inputSpeed.f32Value);

    RETURN_NOERROR;


}

tResult cPIDController::ProcessReset()
{
    boost::lock_guard<boost::mutex> lock(cs_measSpeed);

    tResult res;
    TBoolSignalValue::Data reset;
    static tTimeStamp ts = 0;
    if(IS_FAILED(res = bool_value.readPin(m_ReaderReset, (void *) &reset, ts))){
        RETURN_ERROR(res);
    }
    ts = reset.ui32ArduinoTimestamp;
    if(reset.bValue){

        m_f64AccError = 0;
        m_f64LastError = 0;

    }

    RETURN_NOERROR;


}
tResult cPIDController::ProcessSetSpeed(){
    boost::lock_guard<boost::mutex> lock(cs_setSpeed);
    tResult res;
    TSignalValue::Data inputSpeed;

    if(IS_FAILED(res = signal_value.readPin(m_ReaderSetPoint, (void *) &inputSpeed, m_ui32TimeStampSetPoint))){
        RETURN_ERROR(res);
    }
#ifdef DEBUG
    //if(inputSpeed.f32Value > 0.001 &&  m_f64DevError)
     //   LOG_SUCCESS(cString::Format("PID S IN: %f",inputSpeed.f32Value));
#endif
    m_ui32TimeStampSetPoint = inputSpeed.ui32ArduinoTimestamp;
    m_f64SetSpeed = static_cast<tFloat64>(inputSpeed.f32Value);


    RETURN_NOERROR;
}



// ------------------------------
// ------ OTHER FUNCTIONS -------
// ------------------------------

tResult cPIDController::CalculateSquaredErrorMean(tFloat64 set, tFloat64 real){

    static tFloat32 p_init = 0.3;
    static tFloat32 d_init = p_init/2.5;
    static tFloat32 i_init = p_init/2.;

    static tFloat32 p_max = 0.5;
    static tFloat32 d_max = p_max/2.5;
    static tFloat32 i_max = p_max/2;
    static bool started = false;
    static bool finished = false;
    if(!started && set != 0){
        m_propF32PIDKP =p_init;
        m_propF32PIDKD =d_init;
        m_propF32PIDKI =i_init;
        started = true;
    }
    if(!started) RETURN_NOERROR;

    if(finished) RETURN_NOERROR;

    static tFloat32 p_step = 0.02;
    static tFloat32 i_step = 0.02;
    static tFloat32 d_step = 0.02;
    static int p_max_cnt = (p_max/p_step) - (p_init/p_step) + 1;
    static int i_max_cnt = (i_max/i_step) - (i_init/i_step) + 1;
    static int d_max_cnt = (d_max/d_step) - (d_init/d_step) + 1;
    static int max_errors = 50;
    static int p_cnt = 0;
    static int i_cnt = 0;
    static int d_cnt = 0;
    static int errors_now = 0;

    static tFloat64 squared_mean = 0;
    auto error = set - real;
    squared_mean += error*error;

    ++errors_now;
    if(errors_now >= max_errors){
        squared_mean /= static_cast<tFloat64>(max_errors);

        std::fstream file;
        file.open(debugfile, ios::out | ios::app);
       // file << (double) squared_mean << "  " << (int) max_errors << (double) m_propF32PIDKP << "  " << (double) m_propF32PIDKI << "  " << (double) m_propF32PIDKD << "  " <<m_propF32ControllerType << "  " << m_propF32PIDWindup <<"\n";
         file << (double) squared_mean << "  " << (int) max_errors << ":      " << (double) m_propF32PIDKP << "  " << (double) m_propF32PIDKI << "  " << (double) m_propF32PIDKD << "  "  <<"\n";
        file.close();

        squared_mean = 0;
        errors_now = 0;
        ++d_cnt;
        if(d_cnt >= d_max_cnt || m_propF32PIDKD > (m_propF32PIDKP/1.5)){
            m_propF32PIDKD = d_init - d_step;
            ++i_cnt;
            if(i_cnt >= i_max_cnt || m_propF32PIDKI > (m_propF32PIDKP/1.5)){
                m_propF32PIDKI = (p_init/2.) - i_step;
                ++p_cnt;
                if(p_cnt >= p_max_cnt ){
                    m_propF32PIDKP = p_init - p_step;
                    p_cnt = 0;
                    finished = true;
                }
                m_propF32PIDKP = m_propF32PIDKP +p_step;
                i_cnt = 0;
            }
            m_propF32PIDKI = m_propF32PIDKI + i_step;
            d_cnt = 0;
        }
        m_propF32PIDKD = m_propF32PIDKD + d_step;
    }
        /*
        ++p_cnt;
        if(p_cnt >= p_max_cnt){
            m_propF32PIDKP = p_init - p_step;
            ++i_cnt;
            if(i_cnt >= i_max_cnt || m_propF32PIDKI > (m_propF32PIDKP/1.5)){
                m_propF32PIDKI = i_init - i_step;
                ++d_cnt;
                if(d_cnt >= d_max_cnt || m_propF32PIDKD > (m_propF32PIDKP/2.)){
                    m_propF32PIDKD = d_init - d_step;
                    d_cnt = 0;
                    finished = true;
                }
                m_propF32PIDKD = m_propF32PIDKD +d_step;
                i_cnt = 0;
            }
            m_propF32PIDKI = m_propF32PIDKI + i_step;
            p_cnt = 0;
        }
        m_propF32PIDKP = m_propF32PIDKP + p_step;
    }
*/

    /*
    static tBool calc_mean = tFalse;
    static tFloat64 squared_mean = 0;
    static tUInt32 num_errors = 0;

    auto error = set - real;

    if(set && !calc_mean){ //start of calculation
        calc_mean = tTrue;
    } else if(!set && calc_mean){
        squared_mean /= static_cast<tFloat64>(num_errors);

        std::fstream file;
        file.open(debugfile, ios::out | ios::app);
        file << (double) squared_mean << "  " << (int) num_errors << (double) m_propF32PIDKP << "  " << (double) m_propF32PIDKI << "  " << (double) m_propF32PIDKD << "  " <<m_propF32ControllerType << "  " << m_propF32PIDWindup <<"\n";
        file.close();

        calc_mean = false;
        squared_mean = 0.0;
        num_errors = 0;
    }
    if(calc_mean){
        squared_mean += error*error;
        ++num_errors;
    }*/
    RETURN_NOERROR;

}

tResult cPIDController::ManipulateSpeed(){
    boost::lock_guard<boost::mutex> lock(cs_manipulate);
    auto lastSetSpeed = GetSetSpeed();
    auto lastMeasSpeed = GetMeasuredSpeed();

    if(!lastSetSpeed && !lastMeasSpeed){
        m_f64AccError = 0;
        TransmitManipulatedSpeed(0.0);
        RETURN_NOERROR;
    }

    auto lastError = lastSetSpeed - lastMeasSpeed;



    m_f64AccError =  m_f64AccError * m_f64hungryTerm + lastError;
    m_f64DevError = lastError - m_f64LastError;
    m_f64LastError = lastError;

    tFloat64 manipulatedSpeed,P,I,D;
    P = m_propF32PIDKP * lastError;
    I = m_propF32PIDKI * (m_f64AccError/m_f64MaxIntegralValue);
    D = m_propF32PIDKD * m_f64DevError;

    if(lastSetSpeed == 0){ //breaking
     //   P *= 1.5;
       // I *= 0.5;
     //   D *= 1.5;
        manipulatedSpeed = P + I + D;
        if((IsInBounds(manipulatedSpeed, (tFloat64)-neg_speed_to_mps[NEG_MIN_SPEED], (tFloat64)pos_speed_to_mps[POS_MIN_SPEED]) )&& m_propBPIDSetZero ) // practically no speed
        {
            manipulatedSpeed = 0;
            m_f64AccError = 0;
            m_f64LastError = 0;
#ifdef DEBUG
            LOG_WARNING("braking");
#endif
        }
    } else {
       manipulatedSpeed = P + I + D;
    }
#ifdef DEBUG
    //if(m_f64DevError)
        LOG_INFO(cString::Format("Set: %f  Real: %f  Err: %f  Speed: %f", (lastSetSpeed), (lastMeasSpeed), (lastError), manipulatedSpeed));
    //if(m_f64DevError)
        LOG_INFO(cString::Format("P  : %f  *  %f  =  %f", ( lastError),( m_propF32PIDKP),P));
        LOG_INFO(cString::Format("Acc: %f  /  %f  =  %f  *  %f  =  %f", m_f64AccError,m_f64MaxIntegralValue,(m_f64AccError/m_f64MaxIntegralValue), m_propF32PIDKI, I));
        LOG_INFO(cString::Format("Dev: %f  *  %f  =  %f", ( m_f64DevError),( m_propF32PIDKD),D));
#endif

#ifdef DEBUGTOFILE
        std::fstream file2;
        file2.open(debugfile, ios::out | ios::app);
        file2 << m_pClock->GetStreamTime() << ": \n";
        file2 <<"Set: " << lastSetSpeed  <<   " Real:" << lastMeasSpeed <<" Err:  " << lastError << "  Speed: " << manipulatedSpeed << "\n";
        file2 <<"P:  " << lastError  <<   "  * " << m_propF32PIDKP <<" =  " <<  P << "\n";
        file2 <<"Acc: " << m_f64AccError  <<   " / " << (m_f64AccError/m_f64MaxIntegralValue) <<" * " << m_propF32PIDKI << "  = " << I << "\n";
        file2 <<"P:  " << m_f64DevError  <<   "  * " << m_propF32PIDKD <<" =  " <<  D << "\n";

        file2.close();
#endif

    TransmitManipulatedSpeed(manipulatedSpeed);


#ifdef DEBUGTOFILE
    std::fstream file;
    //file.open(debugfile, ios::out | ios::trunc);
    file.open(debugfile, ios::out | ios::app);
    file << "Set:  " << lastSetSpeed << "  Real: " << lastMeasSpeed << "  Err: "<< lastError  << "Speed: " << manipulatedSpeed << "\n";
    file.close();
#endif


    RETURN_NOERROR;
}

// ------------------------------
// ------ Setter/Getter FUNCTIONS -------
// ------------------------------

tResult cPIDController::TransmitManipulatedSpeed(tFloat32 manipulated_speed){
    boost::lock_guard<boost::mutex> lock(cs_transmit);
    TSignalValue::Data speed;

    if(manipulated_speed > 0){
        speed.f32Value = get_pos_mps_speed(manipulated_speed);
    } else if(manipulated_speed < 0){
        speed.f32Value = get_neg_mps_speed(manipulated_speed);
    } else //0
        speed.f32Value = manipulated_speed;

    //FINAL CHECK if out of range
    if(clamp(speed.f32Value, (float) m_propI32PIDMinOutput, (float) m_propI32PIDMaxOutput)){
#ifdef DEBUG
        LOG_WARNING(cString::Format("SPEED WAS NOT IN BOUNDARY mps(%f) --> %f",manipulated_speed, speed.f32Value));
#endif
#ifdef DEBUGTOFILE
    std::fstream file;
    file.open(debugfile, ios::out | ios::app);
    file << "SPEED WAS NOT IN BOUNDARY; before (in mps): " << manipulated_speed << "   now: "  << speed.f32Value << "\n";
    file.close();
    } else {

        std::fstream file;
        file.open(debugfile, ios::out | ios::app);
        file << "transmit speed (in mps): " << manipulated_speed << "   absolut: "  << speed.f32Value << "\n";
        file.close();

#endif
    }

#ifdef DEBUG
     if(speed.f32Value) LOG_INFO(cString::Format("transmit speed: %f (man: %f)", speed.f32Value, manipulated_speed));
#endif

    return signal_value.writePin(m_WriterManipulatedSpeed, (void *) &speed, m_pClock->GetStreamTime());
  //  RETURN_NOERROR;
}

tFloat64 cPIDController::GetSetSpeed(){
    boost::lock_guard<boost::mutex> lock(cs_setSpeed);
    auto speed = m_f64SetSpeed;
    if(clamp(speed, (tFloat64) ABS_MIN_SPEED, (tFloat64) ABS_MAX_SPEED)){
#ifdef DEBUG
        LOG_WARNING(cString::Format("set speed not in bounds: %f,  clamped to %f", m_f64SetSpeed, speed));
#endif
    }
    return speed;

}

tFloat64 cPIDController::GetMeasuredSpeed(){
    boost::lock_guard<boost::mutex> lock(cs_measSpeed);
    return m_f64MeasuredSpeed;
}

