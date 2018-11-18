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
#pragma once

#define CID_CAR_POSE_FILTER "car_pose.filter.user.aadc.cid"

#define OID_CAR_POSE_FILTER "adtf.aadc.FTO_CAR_POSE"

#include <boost/thread.hpp>

class cCarPose : public cTriggerFunction
{


    TSignalValue signal_value;
    TActionStruct action_struct;
    TBoolSignalValue bool_signal_value;
    TPoseStruct pose_struct;
    TFeedbackStruct feedback_struct;
    TFollowLaneStruct follow_lane_struct;
    TWheelData wheel_data;
    TInerMeasureData iner_measure_data;

private:
    //properties
    property_variable<tFloat32> f32_PropertyWheelBase, f32_PropertyTrackWidth, f32_PropertyWheelCircumference;
    property_variable<tFloat32> f32_PropertySteeringCorrection, f32_PropertyOdometerTicks;
    property_variable<tInt32> initCounter;
    property_variable<tBool> resetPose;
    property_variable<tBool> displayPose;

protected:

    /*! the input pins */
    //TSignalValue
    cPinReader speed_controller_input;

    //TInerMeasureData
    cPinReader iner_measure_input;

    //TWheelData
    cPinReader left_wheel_input;

    //TWheelData
    cPinReader right_wheel_input;

    //TSignalValue
    cPinReader reset_relative_pose;

    /*! the output pins*/
    //TPoseStruct
    cPinWriter car_pose_output;

    //TSignalValue
    cPinWriter car_speed_output;

    //TSignalValue
    cPinWriter overall_distance_output;


    /*! The Data structs */
    TSignalValue::Data lastSpeedControllerInput;
    TWheelData::Data lastLeftWheelData, lastRightWheelData;
    TSignalValue::Data overallDistanceRight, overallDistanceLeft;
    tFloat32 *leftTireSpeed, *rightTireSpeed;
    tUInt32 vectorSize, leftTireSpeedCounter, rightTireSpeedCounter;
    tFloat32 latestSpeed;


    TInerMeasureData::Data lastInerMeasureData;
    TInerMeasureData::Data lastCarOrientation;

    TPoseStruct::Data last_carPose;

    tBool initializedYaw, initializedDistance, firstCallWheelLeft, firstCallWheelRight, initializedInerMeasure;

    boost::mutex criticalSectionTransmit, cs_CarPoseAccess, criticalSectionWheelTach, criticalSectionOverallDistance;

    boost::mutex cs_LatestSpeedInputAccess;
    /*! The reference clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;
public:
    /*! default constructor for template class
           \param __info   [in] This is the name of the filter instance.
    */
    cCarPose();

    /*! default destructor */
    virtual ~cCarPose() = default;
    tResult Configure() override;

protected:
    tResult Process(tTimeStamp tmTimeOfTrigger);


    tResult TransmitPose(tTimeStamp );
    tResult TransmitSpeed(tTimeStamp);
    tResult ProcessCarSpeedInput();
    tResult ProcessRightWheelInput();
    tResult ProcessLeftWheelInput();
    tResult ProcessInerMeasureInput();
    tResult ProcessResetRelativePose();
    tResult TransmitOverallDistance(tTimeStamp);
    tFloat32 DeriveSpeedFromOdometer(TWheelData::Data *, TWheelData::Data *, tFloat32);


    tResult SetCarPose(TPoseStruct::Data &carPose);
    TPoseStruct::Data GetCarPose();
    tResult SetSpeedInput(TSignalValue::Data &speedInput);
    TSignalValue::Data GetSpeedInput();


};

//*************************************************************************************************

