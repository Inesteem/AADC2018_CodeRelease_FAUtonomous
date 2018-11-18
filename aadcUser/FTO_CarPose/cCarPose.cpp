/**
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: �This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS �AS IS� AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2017-05-12 09:34:53#$ $Rev:: 63109   $
**********************************************************************/

#include <math.h>
#include "stdafx.h"
#include "cCarPose.h"

#define DEBUG

ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(
    CID_CAR_POSE_FILTER,		// references to header file
    "cCarPose",              // label
    cCarPose,                // class
    adtf::filter::pin_trigger({"left_wheel_data"}));	// "speed_controller", TODO


cCarPose::cCarPose()
{
        initializedYaw = false, initializedDistance = false, firstCallWheelLeft = true, firstCallWheelRight = true;
        overallDistanceRight.f32Value = 0, overallDistanceLeft.f32Value = 0;
        vectorSize = 10;
        leftTireSpeed = new tFloat32 [vectorSize];
        rightTireSpeed = new tFloat32 [vectorSize];
        for(unsigned int i = 0; i < vectorSize; ++i)
            leftTireSpeed[i] = rightTireSpeed[i] = 0.0;
        resetPose = false;
        overallDistanceLeft.f32Value = 0;
        overallDistanceRight.f32Value = 0;
        last_carPose.f32PosX = 0;
        last_carPose.f32PosY = 0;
        last_carPose.f32Yaw = 0;
        displayPose = false;


        //Set former property variables
        f32_PropertyWheelCircumference = 0.34;
        f32_PropertyWheelBase = 0.36;
        f32_PropertyOdometerTicks = 60; //amount of ticks for one wheel turn
        f32_PropertyTrackWidth = 0.30; //this has to be measured and corrected //WIRD NIRGENDWO VERWENDET; FUCK THIS SHIT!

        RegisterPropertyVariable("WheelBase", f32_PropertyWheelBase);
        RegisterPropertyVariable("WheelCircumference", f32_PropertyWheelCircumference);
        RegisterPropertyVariable("OdometerTicks", f32_PropertyOdometerTicks);
        RegisterPropertyVariable("TrackWidth", f32_PropertyTrackWidth);
        RegisterPropertyVariable("SteeringCorrection", f32_PropertySteeringCorrection);

        RegisterPropertyVariable("initCounter", initCounter);
        RegisterPropertyVariable("resetPose", resetPose);
        RegisterPropertyVariable("displayPose", displayPose);

        //OUTPUT BINHs
        pose_struct.registerPin(this, car_pose_output, "car_pose");
        signal_value.registerPin(this, car_speed_output, "car_speed");
        signal_value.registerPin(this, overall_distance_output, "overall_distance");

        //INPUT BINHs
        iner_measure_data.registerPin(this, iner_measure_input, "iner_measure_data");
        signal_value.registerPin(this, speed_controller_input, "speed_controller");
        wheel_data.registerPin(this, left_wheel_input, "left_wheel_data");
        wheel_data.registerPin(this, right_wheel_input, "right_wheel_data");
        signal_value.registerPin(this, reset_relative_pose, "reset_relative_pose");
        LOG_INFO("CONSTRUCTOR DONE!");

}
tResult cCarPose::Configure()
{
   //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));


    LOG_INFO("CONFIG DONE!");

    RETURN_NOERROR;
}

tResult cCarPose::Process(tTimeStamp tmTimeOfTrigger)
{
    //TODO
//    if (IS_OK(ProcessCarSpeedInput())) //TODO
//    {
        //LOG_SUCCESS("ProcessCarSpeedInput");
  //  }

    ProcessRightWheelInput();
    if (IS_OK(ProcessLeftWheelInput()))
    {
       // LOG_SUCCESS("ProcessRightWheelInput");
        ProcessCarSpeedInput();
        TransmitOverallDistance(tmTimeOfTrigger);
        TransmitSpeed(tmTimeOfTrigger);
        TransmitPose(tmTimeOfTrigger);
    }
    ProcessInerMeasureInput();

    RETURN_NOERROR;


}



tResult cCarPose::ProcessCarSpeedInput(){
    //Never lock cs_LatestSpeedInputAccess here!
    TSignalValue::Data speed;
    tResult res;
  //  LOG_INFO("Process Car Speed Input");
    if(IS_FAILED(res = signal_value.readPin(speed_controller_input, (void *) &speed, GetSpeedInput().ui32ArduinoTimestamp))){
        RETURN_ERROR(res);
    }
  //  LOG_INFO(cString::Format("received speed input %f", speed.f32Value));

    //speed.f32Value = -speed.f32Value; //?
    SetSpeedInput(speed);
    RETURN_NOERROR;
}

tResult cCarPose::ProcessLeftWheelInput(){
    boost::lock_guard<boost::mutex> lock(criticalSectionWheelTach);

    TWheelData::Data tmpLeft;
    tResult res;
    if(IS_FAILED(res = wheel_data.readPin(left_wheel_input, (void *) &tmpLeft, lastLeftWheelData.ui32ArduinoTimestamp)))
        RETURN_ERROR(res);
   // LOG_INFO(cString::Format("left_wheel_data input: %d %d %d",tmpLeft.ui32ArduinoTimestamp,tmpLeft.ui32WheelTach,tmpLeft.i8WheelDir));

    //first call of function, reset of pose, last input was over 0,5s ago
    if(firstCallWheelLeft || resetPose){
        lastLeftWheelData = tmpLeft;
        firstCallWheelLeft = false;
        leftTireSpeedCounter = 0;
        RETURN_NOERROR;
    }

    /*! calculate distance */
    tFloat32 tmp1 = (float)(tmpLeft.ui32WheelTach - lastLeftWheelData.ui32WheelTach);
    tmp1 = tmp1 *  (f32_PropertyWheelCircumference/f32_PropertyOdometerTicks) * 0.5;

    if(GetSpeedInput().f32Value < 0){
        tmp1 = -tmp1;
    }


    /*! calculate / update pose*/
    auto carPose = GetCarPose();
    carPose.f32PosX += tmp1 * cos(lastCarOrientation.f32Yaw);
    carPose.f32PosY += tmp1 * sin(lastCarOrientation.f32Yaw);
    carPose.f32Yaw =           lastCarOrientation.f32Yaw;
    SetCarPose(carPose);


    /*! calculate speed */

    tFloat tmp2 = DeriveSpeedFromOdometer(&tmpLeft, &lastLeftWheelData, tmp1);
    leftTireSpeed[leftTireSpeedCounter] = tmp2;

    leftTireSpeedCounter++;
    if(leftTireSpeedCounter >= vectorSize){
       leftTireSpeedCounter = 0;
     }
    overallDistanceLeft.f32Value += tmp1;
    lastLeftWheelData = tmpLeft;

    RETURN_NOERROR;
}

tResult cCarPose::ProcessRightWheelInput(){
   // boost::lock_guard<boost::mutex> lock(criticalSectionWheelTach);//TODO: sinnvoll ? war im vorjahr hier nicht

    TWheelData::Data tmpRight;

    tResult res;
    if(IS_FAILED(res = wheel_data.readPin(right_wheel_input, (void *) &tmpRight, lastRightWheelData.ui32ArduinoTimestamp)))
        RETURN_ERROR(res);
   // LOG_INFO(cString::Format("right_wheel_data input: %d %d %d",tmpRight.ui32ArduinoTimestamp,tmpRight.ui32WheelTach,tmpRight.i8WheelDir));
    auto carPose = GetCarPose();
    if(firstCallWheelRight || resetPose){
        lastRightWheelData = tmpRight;
        overallDistanceRight.f32Value = 0;
        carPose.f32PosX = 0;
        carPose.f32PosY = 0;
        carPose.f32Yaw = 0;
        SetCarPose(carPose);
        firstCallWheelRight = false;
        rightTireSpeedCounter = 0;
        LOG_SUCCESS(cString::Format("init last_carPose, etc: %d %d", lastRightWheelData.ui32WheelTach, lastRightWheelData.i8WheelDir));
        RETURN_NOERROR;
    }

    /*! calculate distance */
    tFloat32 tmp1 = (float)(tmpRight.ui32WheelTach - lastRightWheelData.ui32WheelTach);
    tmp1 = tmp1 *  (f32_PropertyWheelCircumference/f32_PropertyOdometerTicks) * 0.5;

    if(GetSpeedInput().f32Value < 0){ //Rückwärtsfahren
        tmp1 = -tmp1;
    }


    /*! calculate / update pose*/
    carPose.f32PosX += tmp1 * cos(lastCarOrientation.f32Yaw);
    carPose.f32PosY += tmp1 * sin(lastCarOrientation.f32Yaw);
    carPose.f32Yaw =           lastCarOrientation.f32Yaw;
    SetCarPose(carPose);


    /*! calculate speed */
     tFloat tmp2 = DeriveSpeedFromOdometer(&tmpRight, &lastRightWheelData, tmp1);

    rightTireSpeed[rightTireSpeedCounter] = tmp2;
    rightTireSpeedCounter++;
      if(rightTireSpeedCounter >= vectorSize){
      rightTireSpeedCounter = 0;
    }
    overallDistanceRight.f32Value += tmp1;
    lastRightWheelData = tmpRight;

    RETURN_NOERROR;
}

tFloat32 cCarPose::DeriveSpeedFromOdometer(TWheelData::Data * newData, TWheelData::Data * oldData, tFloat32 distanceTraveled){


    tFloat32 speed = 0;
    speed = (tFloat32) (newData->ui32ArduinoTimestamp - oldData->ui32ArduinoTimestamp);
    if(speed == 0){
       LOG_WARNING("cCarPose:: timedifference is 0 -> divison with 0");
       return 0;
    }
    speed = distanceTraveled * 1000000/ speed;

    return speed;
}


tResult cCarPose::ProcessInerMeasureInput(){
    boost::lock_guard<boost::mutex> lock(criticalSectionWheelTach);

    TInerMeasureData::Data tmpData;
    tResult res;
    if(IS_FAILED(res = iner_measure_data.readPin(iner_measure_input, (void *) &tmpData, lastInerMeasureData.ui32ArduinoTimestamp)))
        RETURN_ERROR(res);

    if(!initializedInerMeasure || resetPose){
        lastCarOrientation.f32Yaw = 0;
        lastInerMeasureData = tmpData;
        initializedInerMeasure = true;
    }

    //if car ist not moving dont add gyro values to limit drift
    if(latestSpeed != 0){
       tFloat32 timeDiff = (tFloat32) (tmpData.ui32ArduinoTimestamp - lastInerMeasureData.ui32ArduinoTimestamp);
       lastCarOrientation.f32Yaw += ((timeDiff/100000) * (tmpData.f32GPosZ * 3250/32000))*M_PI/180;
    }

    lastInerMeasureData = tmpData;

    RETURN_NOERROR;
}

tResult cCarPose::ProcessResetRelativePose(){
    static tUInt32 lastTimeStamp = 0;
    TSignalValue::Data tmpData;
    tResult res;
    if(IS_FAILED(res = signal_value.readPin(reset_relative_pose, (void *) &tmpData, lastTimeStamp)))
        RETURN_ERROR(res);

    TPoseStruct::Data carPose;
    carPose.f32PosX = 0;
    carPose.f32PosY = 0;
    carPose.f32Yaw = 0;
    carPose.f32CarSpeed = 0;
    SetCarPose(carPose);
    lastCarOrientation.f32Yaw = 0;
    lastTimeStamp = tmpData.ui32ArduinoTimestamp;
    RETURN_NOERROR;

}



tResult cCarPose::TransmitSpeed(tTimeStamp inputTime){
    boost::lock_guard<boost::mutex> lock(criticalSectionTransmit);

    TSignalValue::Data tmp_speed;
    for (tUInt i = 0; i < vectorSize; i++){
        tmp_speed.f32Value += leftTireSpeed[i] + rightTireSpeed[i];
    }
    tmp_speed.f32Value = tmp_speed.f32Value / vectorSize;

    if(fabs(tmp_speed.f32Value) < 10){
        auto carPose = GetCarPose();
        carPose.f32CarSpeed = tmp_speed.f32Value;
        SetCarPose(carPose);
    }else{
        LOG_WARNING(cString::Format("cCarPose::Error while computing car_speed!! %f -> was set to 0", tmp_speed.f32Value));
        tmp_speed.f32Value = 0;
    }
    latestSpeed = tmp_speed.f32Value;
    signal_value.writePin(car_speed_output, (void *) &tmp_speed, inputTime);
    //LOG_INFO(cString::Format("cCarPose::car_speed: %f ", tmp_speed.f32Value));
    RETURN_NOERROR;
}

tResult cCarPose::TransmitOverallDistance(tTimeStamp inputTime){
    //LOG_INFO("TransmitOverallDistance");
    TSignalValue::Data tmp_distance;
    tmp_distance.f32Value = overallDistanceRight.f32Value + overallDistanceLeft.f32Value;

    signal_value.writePin(overall_distance_output, (void *) &tmp_distance, inputTime);
    RETURN_NOERROR;
}

tResult cCarPose::TransmitPose(tTimeStamp inputTime){
   // LOG_INFO("TransmitPose");
   boost::lock_guard<boost::mutex> lock(criticalSectionTransmit);

   TPoseStruct::Data tmp_pose;
   tmp_pose = GetCarPose();
   tmp_pose.ui32ArduinoTimestamp = inputTime;
   if(displayPose)
       LOG_INFO(cString::Format("cCarPose::X %f, Y: %f, Yaw(in Deg): %f, CarSpeed: %f", tmp_pose.f32PosX, tmp_pose.f32PosY, tmp_pose.f32Yaw*180/M_PI, tmp_pose.f32CarSpeed));

   pose_struct.writePin(car_pose_output, (void *) &tmp_pose, inputTime);

   RETURN_NOERROR;
}

/* *
 * GETTER AND SETTER
 * */

tResult cCarPose::SetSpeedInput(TSignalValue::Data &speedInput){
    boost::lock_guard<boost::mutex> lock(cs_LatestSpeedInputAccess);
    lastSpeedControllerInput = speedInput;
    RETURN_NOERROR;
}


TSignalValue::Data cCarPose::GetSpeedInput(){
    boost::lock_guard<boost::mutex> lock(cs_LatestSpeedInputAccess);
    return lastSpeedControllerInput;
}

tResult cCarPose::SetCarPose(TPoseStruct::Data &carPose) {
        boost::lock_guard<boost::mutex> lock(cs_CarPoseAccess);
        last_carPose = carPose;
        RETURN_NOERROR;
}
TPoseStruct::Data cCarPose::GetCarPose() {
        boost::lock_guard<boost::mutex> lock(cs_CarPoseAccess);
        return last_carPose;
}
