#pragma once

/*This filter was translated and modified from Lord Lucifer, the Great one, the one and only Bringer of light, the powerfull and mighty King of Hell, the  [...] !
In case of any questions write an email to: lucifer.hemp@fau.de;
In case you really dare writing an email, please start with: "Euch zu untertänigsten Diensten, verehrter Lucifer, melde ich niederer Wurm mich mit folgendem Begehr:"*/

#define OID_ADTF_MOTE_TO_POINT_FILTER "adtf.aadc.FTO_Move_To_Point"

#include "tinyxml2.h"
#include <boost/thread.hpp>
#include <vector>
#include <list>

#define XMLCheckResult(a_eResult) if (a_eResult != XML_SUCCESS) { LOG_WARNING(cString::Format("Error while accessing the XML: %i\n", a_eResult)); RETURN_ERROR(-1); }
//#define DEBUG
//#define DEBUGTOFILE

#define CID_MOVE_TO_POINT_FILTER "move_to_point.filter.user.aadc.cid"

// namespaces
using namespace adtf_util;
using namespace ddl;
using namespace adtf::ucom;
using namespace adtf::base;
using namespace adtf::streaming;
using namespace adtf::mediadescription;
using namespace adtf::filter;
using namespace std;

// states
const tUInt32 IDLE            = 1;
const tUInt32 DILIGENT        = 2;
const tUInt32 FOLLOW_FILE     = 4;
const tUInt32 DONT_STOP       = 8;
const tUInt32 ON_ERROR        = 16;

const tFloat32 DistanceToRampBorder = 0.27;

// cMoveToPoint
class cMoveToPoint : public cTriggerFunction
{


    struct MTP_State {
       tUInt32 state;
        MTP_State(): state(IDLE){}
        MTP_State &operator=(tUInt32 s){
            state = s;
            return *this;
        }
        tBool is_idle(){
            return (state & IDLE);
        }
        tBool is_dont_stop(){
            return (state & DONT_STOP);
        }
        tBool is_follow_file(){
            return (state & FOLLOW_FILE);
        }
        void remove_state(tUInt32 s){
            this->state &= ~s;
        }

        void set_state(tUInt32 s){
            this->state |= s;

        }
    };

    TSignalValue signal_value;
    TBoolSignalValue bool_value;
    TActionStruct action_struct;
    TBoolSignalValue bool_signal_value;
    TPoseStruct pose_struct;
    TFeedbackStruct feedback_struct;

public:

    cMoveToPoint();
    //virtual ~cMoveToPoint() = default;
    virtual ~cMoveToPoint();
    tResult Configure() override;

protected:

    /*! the input pins */
    //Pose input from CarPose filter can be replaced with Audi filter for absolute Pose
    cPinReader        car_pose_input;
    TPoseStruct::Data latestCarPose;

    //GoalPose with Position the car is supposed to drive to
    cPinReader        goal_pose_input;
    TPoseStruct::Data goalPoseIn;

    //ActionStruct encoding the action the filter is supposed to do
    cPinReader        sm_action_input;

    //ActionStruct encoding the action the filter is supposed to do
    cPinReader        follow_ramp_input;

    //ActionStruct encoding the action the filter is supposed to do
    cPinReader        ls_steering_input;
    TSignalValue::Data steeringInput;

    /*! the output pins */
    //Output to the PID_Controller with the desired speed in m/s
    cPinWriter         speed_controller_output;
    TSignalValue::Data speedControllerOut;

    //output to the steeringController with the desired steeringAngle in deg (negative levt, positive right)
    cPinWriter         steering_controller_output;
    TSignalValue::Data steeringControllerOut;

    //output to the state controller returning the momentary state of the filter / requested information by the state machine
    cPinWriter            feedback_struct_output;
    TFeedbackStruct::Data feedbackStructOut;


    //output to the state controller returning the momentary state of the filter / requested information by the state machine
    cPinWriter            reset_pid;

    /*! Data Stucts to hold some data */
    TPoseStruct::Data   carPoseOffset;
    TSignalValue::Data  nextSpeedControllerOut;
    TSignalValue::Data  nextSteeringControllerOut;
    TSignalValue::Data  lastRampInput;

private:

    //properties

    //ramp stuff
    tFloat32 fl_integralValue;

    //Action
    tUInt32 command;
    tUInt32 last_completed_path;
    MTP_State intern_state;

    property_variable<tFloat32> mtp_steeringAngle;
    property_variable<tFloat32> mtp_movementSpeed;

    property_variable<tBool> mtp_use_absolute_pos;
    property_variable<tBool> mtp_stop_after_command;
    property_variable<tFloat32> max_speed;
    property_variable<tFloat32> min_speed;
    property_variable<cFilename> path_to_follow;

    property_variable<tFloat32> kalpha;
    property_variable<tFloat32> kbeta_t;
    property_variable<tFloat32> offset_steering;

    // Car control factor krho forwards
    property_variable<tFloat32> krho_forw;
    // Car control factor krho backwards
    property_variable<tFloat32> krho_back;
    // Speed Reduction Offset in m/s
    property_variable<tFloat32> reduction_offset;
    property_variable<tFloat32> goalDistanceShutOff;


    //Log Trajectory To XML
    tBool lt_enable, lt_erase;
    cFilename loggedXMLDirectory;
    cFilename debugFile;

    //Drive Trajectory
    const tChar * trajectoryPathPointsDirectory;
    //tFloat32 dt_CarSpeedXYMode;
    tFloat32 dt_DistanceToNextPathPoint;

#ifdef DEBUG
    tBool debugToFile, debugToConsole, debugToConsoleCounter;
    cFilename debugFileDirectory;
#endif


    /*! variables */
    //state machine
    tBool isRunning, transmitZeros;
    tFloat32 transmitZerosCounter;

    //move to point
    tUInt32 counter_movetopoint, counter_posesamples;
    tFloat32 direction, mtp_lastRho;

    //variables for first call
    tBool firstCallStateMachine, firstCallLogTrajectory;

    //variables for FollowLane
    tUInt32 pathToGoalIndex;

    //variables to LogTrajectoryToXML
    TPoseStruct::Data   lt_lastCarPose, lt_CarPoseOffset;
    int lt_counter, lt_devider;
    tinyxml2::XMLNode * pRoot;
    tinyxml2::XMLDocument xmlDoc;

    /*! critical sections */

    boost::mutex  cs_LatestCarPoseAccess, cs_ActionAccess, cs_PoseOffsetAccess;
    boost::mutex  cs_GoalPoseAccess, cs_TransmitFeedback,cs_TransmitReset, cs_Steering,
                  cs_FollowRamp;
    boost::mutex  cs_TransmitSpeed,cs_TransmitSteering, m_ocs_PathAccess, cs_ActionInternalCommandAccess, cs_ActionFirstOrderCommandAccess, cs_Drive;
    boost::mutex  cs_AccessRunningState;
    typedef struct fp_points_tmp {
            tFloat32 x;
            tFloat32 y;
            tFloat32 yaw;
            tFloat32 car_speed;
            tFloat32 cut_off_dist;
            tFloat32 steering_alpha;
            tBool stop_after;

            fp_points_tmp() :
                    x(0), y(0.0), yaw(0.0), car_speed(0.0), cut_off_dist(0.0), steering_alpha(1.0), stop_after(tFalse){}
    } fp_points;

    typedef struct fp_paths_tmp {
            tUInt32 path_id;
            const tChar * name;
            std::vector<fp_points> points;

        fp_paths_tmp() :
            path_id(0){}
    } fp_paths;
    std::vector<fp_paths> xml_paths; // read points from xml file
    std::vector<fp_points> path_to_goal; // set path points
    std::list<tUInt32> file_commands; // read command sequence from file
    tUInt32 lastCommand;


    /*! function declaration*/
    
    tResult Process(tTimeStamp tmTimeOfTrigger);

    //input/output handler
    
    tResult ProcessCarPoseInput();
    tResult ProcessGoalPointInput();
    tResult ProcessActionInput();
    tResult ProcessSteeringInput();
    tResult ProcessRampInput();

    tResult TransmitSpeedController(tTimeStamp);
    tResult TransmitSteeringController(tTimeStamp);
    tResult TransmitResetPID(tTimeStamp,bool);
    tResult TransmitFeedback(tUInt32 fb);
    //getter/setter
    tUInt32 GetInternalCommand();
    tResult SetInternalCommand(tUInt32);
    tUInt32 GetFirstOrderCommand();
    tResult SetFirstOrderCommand(tUInt32);
    tResult SetIfNoNewFirstOrderCommand(tUInt32,tUInt32);

    tBool   GetRunningState();
    tResult SetRunningState(tBool);


    TPoseStruct::Data GetLatestCarPose();
    tResult SetLatestCarPose(TPoseStruct::Data);

    TSignalValue::Data GetLatestSteeringInput();
    tResult SetLatestSteeringInput(TSignalValue::Data);

    TPoseStruct::Data GetCarPoseOffset();
    tResult SetCarPoseOffset(TPoseStruct::Data);

    TPoseStruct::Data GetGoalPose();
    tResult SetGoalPose(TPoseStruct::Data);
    tResult LoadPathPointData();
    tResult LoadProperties();
    tResult SetPath();


    //all the rest
    tResult ComputeMoveToPointAlgorithm();
    tResult ReadInFileCommands();
    tResult InternalStateMachine(tTimeStamp);
    tResult LogTrajectoryToXML();
    tResult DriveTrajectory();

    inline tBool DirChange(){
        //Speed is reduced when near next point and direction change in next point (reversal of sign)
        //car coord system: x is dir straight
        if (( path_to_goal[pathToGoalIndex].x > 0 &&
                (path_to_goal[pathToGoalIndex+1].x < path_to_goal[pathToGoalIndex].x )) ||
            ( path_to_goal[pathToGoalIndex].x < 0 &&
                (path_to_goal[pathToGoalIndex+1].x > path_to_goal[pathToGoalIndex].x )) )
        {
                return tTrue;
        }
        return tFalse;
    }

    inline void enable_transmit_zeros(){
        transmitZeros = tTrue;
        transmitZerosCounter = 0;
    }

    inline void reset_steering(tTimeStamp time_stamp){
        LOG_WARNING("steering resetted!");
        nextSteeringControllerOut.f32Value = 0;
        TransmitSteeringController(time_stamp);
    }
    inline void reset_speed(tTimeStamp time_stamp){
        LOG_WARNING("speed resetted!");
        nextSpeedControllerOut.f32Value = 0;
        TransmitSpeedController(time_stamp);
    }

    inline tBool transmit_zeros(tTimeStamp time_stamp) {
        if(transmitZeros){ // initial true
            //exp of 0 == 1, exp of -3.375 ~ 0.03, so values between build a smooth curve towards 0, so the car will not stop abruptly; further is 3.375/0.075 == 45 as before
          //  if(transmitZerosCounter > 3.375){  //about 1,5s
              if(transmitZerosCounter > 45){  //about 1,5s
                transmitZerosCounter = 0;
              //  nextSpeedControllerOut.f32Value = 0;
              //  TransmitSpeedController(time_stamp);
                transmitZeros = tFalse;
            } else {
              //  LOG_INFO("cMoveToPoint::transmit zero samples");
                //transmitZerosCounter+=0.075;
                transmitZerosCounter++;
               // nextSpeedControllerOut.f32Value = std::exp(-transmitZerosCounter) * lastTrajectorySpeed;
                 nextSpeedControllerOut.f32Value = 0;
                //nextSteeringControllerOut.f32Value = 0;
                TransmitSpeedController(time_stamp);
                //TransmitSteeringController(time_stamp);
            }
        }
        return transmitZeros;
    }

    /*! The reference clock */
    object_ptr<adtf::services::IReferenceClock> m_pClock;




};
