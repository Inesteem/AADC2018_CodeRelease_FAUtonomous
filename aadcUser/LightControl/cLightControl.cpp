/*********************************************************************
typedef struct
{
    tUInt32 ui32ArduinoTimestamp;
    tBool bValue;
} tBoolSignalValue;

**********************************************************************/

#include <mutex>
#include "stdafx.h"
#include "cLightControl.h"
#include <ADTF3_helper.h>
#include "ScmCommunication.h"

ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(CID_LIGHT_CONTORL_DATA_TRIGGERED_FILTER,
                                    "cLightControl",
                                    LightControl,
                   //                  adtf::filter::pin_trigger({"carspeedInput","actionInput","setspeedInput"}));
                                    adtf::filter::pin_trigger({"carspeedInput"}));

LightControl::LightControl()
{
    SetName("LightControl Constructor");

    //INPUT BINHs
    m_SignalValue.registerPin(this, carspeedInput, "carspeedInput");
    m_SignalValue.registerPin(this, setspeedInput, "setspeedInput");

    m_ActionStruct.registerPin(this, actionInput,        "actionInput"         );

    //OUTPUT BINHs
    m_BoolValue.registerPin(this, headLightOutput,       "headLightOutput"      );
    m_BoolValue.registerPin(this, reverseLightOutput,    "reverseLightOutput"   );
    m_BoolValue.registerPin(this, brakeLightOutput,      "brakeLightOutput"     );
    m_BoolValue.registerPin(this, turnRightLightOutput,  "turnRightLightOutput" );
    m_BoolValue.registerPin(this, turnLeftLightOutput,   "turnLeftLightOutput"  );
    m_BoolValue.registerPin(this, hazardLightOutput,     "hazardLightOutput"    );


    RegisterPropertyVariable("Factor for enabling brake lights", factorEnableBrake);
    RegisterPropertyVariable("Diff for brake Light", diffSpeedBrakeEnable);
    RegisterPropertyVariable("Factor for disabling brake lights", factorDisableBrake);
    RegisterPropertyVariable("Speed Average Count", averageSampleCount);


    factorEnableBrake = 0.95; // should be less than 1
    factorDisableBrake = 1.04; // should be greater than 1
    averageSampleCount = 10; // number of samples used to calculate speed average
    diffSpeedBrakeEnable = -0.015;
    //default values
    HeadLight       = tFalse;
    ReverseLight    = tFalse;
    BrakeLight      = tFalse;
    TurnRightLight  = tFalse;
    TurnLeftLight   = tFalse;
    HazardLight     = tFalse;
    acc_speed = 0;
        ManuallySetBrakeLight = 0;

    LOG_SUCCESS(cString::Format("Registration finished!"));

}


//implement the Configure function to read ALL Properties
tResult LightControl::Configure()
{

    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));

    LOG_INFO(cString::Format("Configuration finished!"));


    RETURN_NOERROR;
}



tResult LightControl::ProcessSpeedInput(tTimeStamp &)
{
    TSignalValue::Data rec_carspeed;
    static unsigned int last_timestamp = 0;
    //ProcessSpeedInput
    //RETURN_IF_FAILED(getLastSampleTSignalValue(carspeedInput,o_SpeedIndex,m_SignalValueSampleFactory,rec_carspeed));
    RETURN_IF_FAILED(m_SignalValue.readPin(carspeedInput, (void *) & rec_carspeed, last_timestamp));

    last_timestamp = rec_carspeed.ui32ArduinoTimestamp;

    static int last_dir = 0;
    int mom_dir = 1 ? (rec_carspeed.f32Value > 0) : (0 ? (!rec_carspeed.f32Value) : -1 ); 
    if(last_dir != mom_dir){
        last_carspeeds.clear();
        acc_speed = 0;
        last_dir = mom_dir;
    } else  if (last_carspeeds.size() > averageSampleCount) {
        acc_speed -= last_carspeeds.front();
        last_carspeeds.pop_front();
    }

    
        // save received car speed value
    last_carspeeds.push_back(fabsf(rec_carspeed.f32Value));
    acc_speed += rec_carspeed.f32Value;

    // average calculation
    /*
    tFloat32 average_speed = 0;
    for (std::list<tFloat32>::iterator p = last_carspeeds.begin(); p != last_carspeeds.end(); ++p) {
        average_speed += (tFloat32)*p;
    }

    tFloat32 speed_average = average_speed / last_carspeeds.size();
    */
    tFloat32 speed_average = acc_speed / last_carspeeds.size();

    // enable brake lights
    if(!brakelightEnabled && (fabs(rec_carspeed.f32Value) - ( speed_average * factorEnableBrake)) < diffSpeedBrakeEnable){
        TBoolSignalValue::Data brake_light_data;
        brake_light_data.ui32ArduinoTimestamp = 0;
        brake_light_data.bValue = tTrue;


        RETURN_IF_FAILED(m_BoolValue.writePin(brakeLightOutput, (void *) &brake_light_data, m_pClock->GetStreamTime()));
        brakelightEnabled = tTrue;
    }
    // disable brake lights
    if ( (fabsf(rec_carspeed.f32Value) > ( speed_average * factorDisableBrake ) || !rec_carspeed.f32Value) && brakelightEnabled ) {
        if(ManuallySetBrakeLight == 0) {
            TBoolSignalValue::Data brake_light_data;
            brake_light_data.ui32ArduinoTimestamp = 0;
            brake_light_data.bValue = tFalse;

            RETURN_IF_FAILED(m_BoolValue.writePin(brakeLightOutput, (void *) &brake_light_data, m_pClock->GetStreamTime()));
            brakelightEnabled = tFalse;
        }
    }

    if(ManuallySetBrakeLight != 0) --ManuallySetBrakeLight;
    RETURN_NOERROR;
}


tResult LightControl::ProcessSetSpeedInput(tTimeStamp &)
{
    boost::lock_guard<boost::mutex> lock(cs_inputSetSpeed);
    // get set car speed
    //ProcessSpeedInput
    TSignalValue::Data rec_setcarspeed;
    static unsigned int last_timestamp = 0;
    RETURN_IF_FAILED(m_SignalValue.readPin(setspeedInput, (void *) & rec_setcarspeed, last_timestamp));
    last_timestamp = rec_setcarspeed.ui32ArduinoTimestamp;

    // enable reverse lights
    if (rec_setcarspeed.f32Value < 0 && !reverselightEnabled ) {
        TBoolSignalValue::Data reverse_light_data;
        reverse_light_data.ui32ArduinoTimestamp = 0;
        reverse_light_data.bValue = tTrue;

        RETURN_IF_FAILED(m_BoolValue.writePin(reverseLightOutput, (void *) &reverse_light_data, m_pClock->GetStreamTime()));
        reverselightEnabled = tTrue;
    }
    // disable reverse lights
    else if ( rec_setcarspeed.f32Value >= 0 && reverselightEnabled ) {
        TBoolSignalValue::Data reverse_light_data;
        reverse_light_data.ui32ArduinoTimestamp = 0;
        reverse_light_data.bValue = tFalse;

        RETURN_IF_FAILED(m_BoolValue.writePin(reverseLightOutput, (void *) &reverse_light_data, m_pClock->GetStreamTime()));

        reverselightEnabled = tFalse;
    }

    RETURN_NOERROR;
}


tResult LightControl::ProcessAction(tTimeStamp &)
{

    boost::lock_guard<boost::mutex> lock(cs_inputAction);
    TActionStruct::Data action;
    static unsigned int last_timestamp = 0;
    RETURN_IF_FAILED(m_ActionStruct.readPin(actionInput, (void *) &action, last_timestamp));
    last_timestamp = action.ui32ArduinoTimestamp;

    // action = tActionStruct.Read_Action(mediaSample, F_LIGHT_FILTER);


    if(action.bEnabled == tTrue && action.bStarted == tTrue) {
        // Light Command: 7000 - 7999
        // 7 a b c
        // a	Head Light 0: no change, 1: disable, 2: enable
        // b	Reverse Light: 0: no change, 1: disable, 2: enable
        // c	Turn Lights: 0: no change, 1: disable, 2: left turn, 3: right turn, 4: hazard

        // check command input (range 7000 to 7999)
    if(action.ui32Command == AC_LC_BRAKE_LIGHT_ON){
        BrakeLight = tTrue;
        ManuallySetBrakeLight = 100;
        TBoolSignalValue::Data brakeLightData;
        brakeLightData.bValue = tTrue;
        RETURN_IF_FAILED(m_BoolValue.writePin(brakeLightOutput, (void *) &brakeLightData, m_pClock->GetStreamTime()));
    } else if (action.ui32Command == AC_LC_BRAKE_LIGHT_OFF) {
        BrakeLight = tFalse;
        ManuallySetBrakeLight = 0;
        TBoolSignalValue::Data brakeLightData;
        brakeLightData.bValue = tFalse;
        RETURN_IF_FAILED(m_BoolValue.writePin(brakeLightOutput, (void *) &brakeLightData, m_pClock->GetStreamTime()));
                                                                    } else if(action.ui32Command > F_LIGHT_FILTER * 1000 && action.ui32Command < (F_LIGHT_FILTER + 1) * 1000 ) {

            tUInt32 a_headLight = ( action.ui32Command % 1000 ) / 100;
            tUInt32 b_reverseLight = ( action.ui32Command % 100 ) / 10;
            tUInt32 c_turnLight = action.ui32Command % 10;

            // output clock for mediasamples

            //////////////////////
            // process head light
            /////////////////////
            TBoolSignalValue::Data headLightData;
            headLightData.ui32ArduinoTimestamp = 0;

            if ( a_headLight == 0 ) {
                // do nothing
            } else if ( a_headLight < 3 ) {
                if ( a_headLight == 1 )
                    headLightData.bValue = tFalse;
                else
                    headLightData.bValue = tTrue;
                RETURN_IF_FAILED(m_BoolValue.writePin(headLightOutput, (void *) &headLightData, m_pClock->GetStreamTime()));

            } else {
                LOG_ERROR(cString::Format("LightControl: Head lights, invalid command %d", a_headLight));
                RETURN_ERROR(ERR_INVALID_STATE);
            }
            //////////////////////
            // process reverse light
            /////////////////////
            tBoolSignalValue reverseLightData;
            reverseLightData.ui32ArduinoTimestamp = 0;

            if ( b_reverseLight == 0 ) {
                // do nothing
            } else if ( b_reverseLight == 1 ) {
                reverseLightData.bValue = tFalse;
                RETURN_IF_FAILED(m_BoolValue.writePin(reverseLightOutput, (void *) &reverseLightData, m_pClock->GetStreamTime()));

            } else if (b_reverseLight == 2) {
                reverseLightData.bValue = tTrue;
                RETURN_IF_FAILED(m_BoolValue.writePin(reverseLightOutput, (void *) &reverseLightData, m_pClock->GetStreamTime()));
            } else {
                LOG_ERROR(cString::Format("LightControl: Reverse lights, invalid command %d", b_reverseLight));
                RETURN_ERROR(ERR_INVALID_STATE);
            }

            //////////////////////
            // process turn lights
            /////////////////////
            TBoolSignalValue::Data TurnRightLightData;
            TBoolSignalValue::Data TurnLeftLightData;
            TBoolSignalValue::Data HazardLightData;

            TurnLeftLightData.ui32ArduinoTimestamp = 0;
            TurnRightLightData.ui32ArduinoTimestamp = 0;
            HazardLightData.ui32ArduinoTimestamp = 0;

            if ( c_turnLight == 0 ) {
                // do nothing
            } else if (c_turnLight < 5) {

                // disable all (case 1)
                TurnLeftLightData.bValue = tFalse;
                TurnRightLightData.bValue = tFalse;
                HazardLightData.bValue = tFalse;

                if (c_turnLight == 2) {
                    // left turn
                    TurnLeftLightData.bValue = tTrue;

                    RETURN_IF_FAILED(m_BoolValue.writePin(turnLeftLightOutput, (void *) &TurnLeftLightData, m_pClock->GetStreamTime()));
                } else if (c_turnLight == 3) {
                    // right turn
                    TurnRightLightData.bValue = tTrue;

                    RETURN_IF_FAILED(m_BoolValue.writePin(turnRightLightOutput, (void *) &TurnRightLightData, m_pClock->GetStreamTime()));
                } else { // case 1 OR 4 (disable of hazard light should also disable turn lights
                    if (c_turnLight == 4) {
                        // hazard light
                        HazardLightData.bValue = tTrue;
                    }

                    RETURN_IF_FAILED(m_BoolValue.writePin(hazardLightOutput, (void *) &HazardLightData, m_pClock->GetStreamTime()));
                }

            } else {
                LOG_ERROR(cString::Format("LightControl: turn lights, invalid command %d", c_turnLight));
                RETURN_ERROR(ERR_INVALID_STATE);
            }


        } else {
            LOG_ERROR(cString::Format("LightControl: Received command %d, expected command in range %d to %d", action.ui32Command,  F_LIGHT_FILTER * 1000, (F_LIGHT_FILTER + 1) * 1000 ));
        }
    }


    RETURN_NOERROR;

}


tResult LightControl::Process(tTimeStamp tmTimeOfTrigger)
{

    if(IS_OK(ProcessSpeedInput(tmTimeOfTrigger))){

        //success!
    }

    ProcessSetSpeedInput(tmTimeOfTrigger);
        ProcessAction(tmTimeOfTrigger);

    RETURN_NOERROR;
}


