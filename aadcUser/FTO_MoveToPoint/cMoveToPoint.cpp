#include <math.h>
#include <iostream>
#include <stdlib.h>
#include <fstream>
#include <unistd.h>
#include <stdlib.h>


#include "stdafx.h"
#include "cMoveToPoint.h"
#include <common_helper.h>
#include <property_structs.h>
#include "ScmCommunication.h"




ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(
    CID_MOVE_TO_POINT_FILTER,		// references to header file
    "cMoveToPoint",              // label
    cMoveToPoint,                // class
    adtf::filter::pin_trigger({"car_pose_input"}));

cMoveToPoint::~cMoveToPoint(){}

cMoveToPoint::cMoveToPoint(){
    //move to point
    counter_movetopoint = 0, counter_posesamples = 0;
    direction = 0;
    dt_DistanceToNextPathPoint = 0.1;
    firstCallLogTrajectory = true;
    mtp_lastRho = -1.0; // before: firstcalllastrho
    isRunning = false;
    transmitZeros = true;
    transmitZerosCounter = 0;

#ifdef DEBUG
    debugToFile = true;
    debugToConsole = true;
    debugFileDirectory = add_date_to_filename("/home/aadc/AADC/src/aadcUser/FTO_MoveToPoint/debug/move_to_point","txt").c_str();

#endif

    /*! properties used by different functionalities */

    /*! properties for logging driven trajectory to xml*/
    lt_enable = false;
    lt_devider = 30;
    lt_erase = false;
    loggedXMLDirectory = "/home/aadc/AADC/src/aadcUser/FTO_MoveToPoint/xml/TrajectoryToXML.xml";


    /*! properties for following trajectory*/
    trajectoryPathPointsDirectory = "/home/aadc/AADC/src/aadcUser/FTO_MoveToPoint/xml/FollowPathPoints.xml";


    /*! properties to test filter as long as there is no filter inputting poses*/
    mtp_movementSpeed = 1.0;
    RegisterPropertyVariable("movement_speed", mtp_movementSpeed);
    mtp_steeringAngle = 80.0;
    RegisterPropertyVariable("steering_angle", mtp_steeringAngle);
    krho_forw = 1;
    RegisterPropertyVariable("krho_forw", krho_forw);
    krho_back = 1;
    RegisterPropertyVariable("krho_back", krho_back);
    offset_steering = 0;
    RegisterPropertyVariable("offset_steering", offset_steering);
    reduction_offset = 0;
    RegisterPropertyVariable("reduction_offset", reduction_offset);
    goalDistanceShutOff = 0.1;
    RegisterPropertyVariable("goalDistanceShutOff", goalDistanceShutOff);
    kbeta_t = 7;
    RegisterPropertyVariable("kbeta_t", kbeta_t);
    kalpha = 8;
    RegisterPropertyVariable("kalpha", kalpha);

    fl_integralValue = 0;

   // intern_state = IDLE; //is per default idle
    lastCommand = AC_NO_COMMAND;

    //OUTPUT BINHs
    signal_value.registerPin(this, speed_controller_output, "speed_controller_output");
    signal_value.registerPin(this, steering_controller_output, "steering_controller_output");
    feedback_struct.registerPin(this, feedback_struct_output, "feedback_struct_output");
    bool_value.registerPin(this, reset_pid, "reset_pid");

    //INPUT BINHs
    pose_struct.registerPin(this, car_pose_input, "car_pose_input");
    pose_struct.registerPin(this, goal_pose_input, "goal_pose_input");
    action_struct.registerPin(this, sm_action_input, "sm_action_input");
    signal_value.registerPin(this, ls_steering_input, "ls_steering_input");
    signal_value.registerPin(this, follow_ramp_input, "follow_ramp_input");


    //ActionStruct encoding the action the filter is supposed to do
    cPinReader        sm_action_input;

    //Properties
    mtp_use_absolute_pos = false; //TODO
    RegisterPropertyVariable("use absolute position", mtp_use_absolute_pos);
    mtp_stop_after_command = true;
    RegisterPropertyVariable("stop after command", mtp_stop_after_command);
    max_speed = 3.6;
    RegisterPropertyVariable("max speed", max_speed);
    min_speed = 0.6;
    RegisterPropertyVariable("min speed", min_speed);
    //path_to_follow = cFilename("/home/aadc/AADC/src/aadcUser/FTO_MoveToPoint/xml/StairwayToHeaven.xml");
    RegisterPropertyVariable("set command sequence", path_to_follow);

    goalPoseIn.ui32ArduinoTimestamp = 0;

    LoadPathPointData();

    //LOG_INFO("CONSTRUCTOR DONE!");

}

tResult cMoveToPoint::Configure()
{

    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    //LOG_INFO("CONFIG DONE!");

    LoadProperties();
    RETURN_NOERROR;
}
/*****************************************************************/
/* processing input and output pins                              */
/*****************************************************************/


tResult cMoveToPoint::Process(tTimeStamp tmTimeOfTrigger)
{


    // by comparing it to our member pin variable we can find out which pin received
    // the sample
    if (IS_OK(ProcessCarPoseInput()))
    {
        InternalStateMachine(tmTimeOfTrigger);
    }

    ProcessRampInput();
    ProcessGoalPointInput();
    ProcessActionInput();
    ProcessSteeringInput();
   // else if (IS_OK(ProcessFollowLaneInput()))
   // {
   // }

    RETURN_NOERROR;
}


//TODO: understand this shit: dOne!
tResult cMoveToPoint::ProcessRampInput(){
    boost::lock_guard<boost::mutex> lock(cs_FollowRamp);
/*
    static int counter = 0;
    ++counter;
    if(counter == 10) {
        counter = 0;

    } else RETURN_NOERROR;
*/

    tResult res;
    TSignalValue::Data rampInput; //always positive
    if(IS_FAILED(res = signal_value.readPin(follow_ramp_input, (void *) &rampInput, lastRampInput.ui32ArduinoTimestamp)))
        RETURN_ERROR(res);

    TPoseStruct::Data next_point;
    next_point.f32PosY = 2 * (0.32 - rampInput.f32Value);
    next_point.f32PosX = 0.3;
    next_point.f32CarSpeed = 0.7;
    if(next_point.f32PosY < -0.28) next_point.f32PosY = -0.28;
    if(next_point.f32PosY > 0.28) next_point.f32PosY = 0.28;
    nextSteeringControllerOut.f32Value = -next_point.f32PosY * 280;
    if(nextSteeringControllerOut.f32Value > 75) nextSteeringControllerOut.f32Value = 75;
    else if(nextSteeringControllerOut.f32Value < -75) nextSteeringControllerOut.f32Value = -75;
    nextSpeedControllerOut.f32Value = 0.7;
    LOG_SUCCESS(cString::Format("rampInput: %f   steering vs nextpoint: %f vs %f ", rampInput.f32Value, nextSteeringControllerOut.f32Value,  next_point.f32PosY ));

    TSignalValue::Data tmp_speed;
    tmp_speed.f32Value = 1.5;
    signal_value.writePin(speed_controller_output, (void *) &tmp_speed, rampInput.ui32ArduinoTimestamp);
// if(next_point.f32PosY < -0.1) next_point.f32PosY = -0.1;
   // else if(next_point.f32PosY > 0.1) next_point.f32PosY = 0.1;
    // next_point.f32Yaw = 0;
   // SetGoalPose(next_point);
   TransmitSteeringController(0);
   // SetFirstOrderCommand(AC_FP_GOTO_XY_NOSTOP);

    lastRampInput = rampInput;
    RETURN_NOERROR;
}


/*****************************************************************/
/* Inputs                                                        */
/*****************************************************************/


tResult cMoveToPoint::ProcessCarPoseInput(){

   TPoseStruct::Data carPoseIn;
    auto car_pose = GetLatestCarPose();
    tResult res;
    if(IS_FAILED(res = pose_struct.readPin(car_pose_input, (void *) &carPoseIn, car_pose.ui32ArduinoTimestamp)))
        RETURN_ERROR(res);
    // log pose to file
    #ifdef DEBUG
    if ( counter_posesamples % 5 == 0) { // 1 out of 5
        if(debugToFile){
            fstream file_pose;
            file_pose.open(debugFileDirectory, ios::out | ios::app);
            file_pose <<"f32PosX: " << carPoseIn.f32PosX << "  f32PosY: " << carPoseIn.f32PosY << "  f32Yaw(in Deg): " << carPoseIn.f32Yaw * 180/M_PI << "\n";
            file_pose.close();
        }
        counter_posesamples = 0;
    }
    counter_posesamples++;
    #endif
    SetLatestCarPose(carPoseIn);

    RETURN_NOERROR;
}

tResult cMoveToPoint::ProcessSteeringInput(){

   TSignalValue::Data steering_in = GetLatestSteeringInput();
    tResult res;
    if(IS_FAILED(res = signal_value.readPin(ls_steering_input, (void *) &steering_in, steering_in.ui32ArduinoTimestamp)))
        RETURN_ERROR(res);

    SetLatestSteeringInput(steering_in);

    RETURN_NOERROR;
}

tResult cMoveToPoint::ProcessGoalPointInput(){
    boost::lock_guard<boost::mutex> lock(cs_GoalPoseAccess);

    // save external goal pose sample
    tResult res;
    if(IS_FAILED(res = pose_struct.readPin(goal_pose_input, (void *) &goalPoseIn, goalPoseIn.ui32ArduinoTimestamp)))
        RETURN_ERROR(res);
    RETURN_NOERROR;
}


tResult cMoveToPoint::ProcessActionInput(){
    boost::lock_guard<boost::mutex> lock(cs_ActionAccess);

    // WARNING: NEVER EVER lock this shit mutex here in this function because it calls SetFirstOrderCommand!!
    //boost::lock_guard<boost::mutex> lock(cs_ActionFirstOrderCommandAccess);

    static TActionStruct::Data actionStruct;

    //TODO: assert F_FO..
    tResult res;
    if(IS_FAILED(res = action_struct.readPin(sm_action_input, (void *) &actionStruct, actionStruct.ui32ArduinoTimestamp)))
        RETURN_ERROR(res);

//#ifdef DEBUG TODO
     LOG_INFO(cString::Format("got action for filter: %d, with command: %d", actionStruct.ui8FilterId, actionStruct.ui32Command));
//#endif
     if(actionStruct.ui32Command == RELOAD_XML_FILES) {
             LOG_SUCCESS("reload xml files...");
             xml_paths.clear();
             LoadPathPointData();
             LoadProperties(); // TODO
     }
    else if(actionStruct.ui8FilterId == F_FOLLOW_PATH){
        //LOG_INFO("process action input:");
        nextSpeedControllerOut.f32Value = 0;
        TransmitResetPID(m_pClock->GetStreamTime(),true);
        SetFirstOrderCommand(actionStruct.ui32Command);

    }

    else {
        LOG_WARNING(cString::Format("wrong filter: %d, wrong command: %d", actionStruct.ui8FilterId, actionStruct.ui32Command));


    }
    RETURN_NOERROR;
}





/*****************************************************************/
/* Outputs                                                 */
/*****************************************************************/


tResult cMoveToPoint::TransmitSpeedController(tTimeStamp inputTime){
     boost::lock_guard<boost::mutex> lock(cs_TransmitSpeed);

     TSignalValue::Data tmp_speed;
     static tFloat32 speedIntegral = max_speed - min_speed;
     tmp_speed.ui32ArduinoTimestamp = inputTime;
     tmp_speed.f32Value = nextSpeedControllerOut.f32Value * speedIntegral;
  //   LOG_INFO(cString::Format("%f   %f    %f ", (float)max_speed, (float)min_speed, (float)tmp_speed.f32Value ));

    if(tmp_speed.f32Value > 0){
        tmp_speed.f32Value += min_speed;

       // if(tmp_speed.f32Value < 0.6 && !GetLatestCarPose().f32CarSpeed){ }
    }
    else if(tmp_speed.f32Value < 0){
        tmp_speed.f32Value -= min_speed;

       // if(tmp_speed.f32Value > 0.7 && !GetLatestCarPose().f32CarSpeed){ }
    }
//    LOG_INFO(cString::Format("%f   ", (float)tmp_speed.f32Value ));

#ifdef DEBUG
  //  LOG_SUCCESS(cString::Format("send to PID:  %f --> %f", (float)nextSpeedControllerOut.f32Value, (float)tmp_speed.f32Value));
    if(debugToFile){
        fstream file_pose;
        file_pose.open(debugFileDirectory, ios::out | ios::app);
        file_pose << inputTime << ":  send to PID: " << nextSpeedControllerOut.f32Value << " --- > " << tmp_speed.f32Value << "\n";
        file_pose.close();
    }
#endif
    return signal_value.writePin(speed_controller_output, (void *) &tmp_speed, inputTime);
}


tResult cMoveToPoint::TransmitSteeringController(tTimeStamp inputTime){
    //TODO: subtrakt 15 ?
     boost::lock_guard<boost::mutex> lock(cs_TransmitSteering);

     TSignalValue::Data tmp_steer;
     tmp_steer.f32Value = nextSteeringControllerOut.f32Value + offset_steering;
     tmp_steer.ui32ArduinoTimestamp = inputTime;
     return signal_value.writePin(steering_controller_output, (void *) &tmp_steer, inputTime);
}

tResult cMoveToPoint::TransmitResetPID(tTimeStamp inputTime,bool b){
      boost::lock_guard<boost::mutex> lock(cs_TransmitReset);

      TBoolSignalValue::Data tmp_steer;
      tmp_steer.bValue = b;
      tmp_steer.ui32ArduinoTimestamp = inputTime;
      return bool_value.writePin(reset_pid, (void *) &tmp_steer, inputTime);
}


tResult cMoveToPoint::TransmitFeedback(tUInt32 feedback_num){
    boost::lock_guard<boost::mutex> lock(cs_TransmitFeedback);
    TFeedbackStruct::Data feedback;
    feedback.ui8FilterId = F_FOLLOW_PATH;
    feedback.ui32FeedbackStatus = feedback_num;
#ifdef DEBUG
    LOG_SUCCESS(cString::Format("send feedback: %d",feedback_num));
#endif
    return feedback_struct.writePin(feedback_struct_output, (void *) &feedback, m_pClock->GetStreamTime());
}

/*****************************************************************/
/* Getter/Setter                                                 */
/*****************************************************************/


tResult cMoveToPoint::SetRunningState(tBool running){
    boost::lock_guard<boost::mutex> lock(cs_AccessRunningState);
    isRunning = running;
    RETURN_NOERROR;
}


tBool cMoveToPoint::GetRunningState(){
    boost::lock_guard<boost::mutex> lock(cs_AccessRunningState);
    return isRunning;
}

tUInt32 cMoveToPoint::GetInternalCommand(){
    boost::lock_guard<boost::mutex> lock(cs_ActionInternalCommandAccess);
    return command;
}

tResult cMoveToPoint::SetInternalCommand(tUInt32 command_){
    boost::lock_guard<boost::mutex> lock(cs_ActionInternalCommandAccess);
    command = command_;
    RETURN_NOERROR;
}

tUInt32 cMoveToPoint::GetFirstOrderCommand(){
    boost::lock_guard<boost::mutex> lock(cs_ActionFirstOrderCommandAccess);
    return lastCommand;
}

tResult cMoveToPoint::SetFirstOrderCommand(tUInt32 command_){
    boost::lock_guard<boost::mutex> lock(cs_ActionFirstOrderCommandAccess);
    lastCommand = command_;
    RETURN_NOERROR;
}

tResult cMoveToPoint::SetIfNoNewFirstOrderCommand(tUInt32 old_command,tUInt32 new_command){
    boost::lock_guard<boost::mutex> lock(cs_ActionFirstOrderCommandAccess);
    if(lastCommand == old_command){
        lastCommand = new_command;
    }
    RETURN_NOERROR;
}

TPoseStruct::Data cMoveToPoint::GetLatestCarPose(){
    boost::lock_guard<boost::mutex> lock(cs_LatestCarPoseAccess);
    return latestCarPose;
}

TSignalValue::Data cMoveToPoint::GetLatestSteeringInput(){
    boost::lock_guard<boost::mutex> lock(cs_Steering);
    return steeringInput;
}

tResult cMoveToPoint::SetLatestCarPose(TPoseStruct::Data pose){
    boost::lock_guard<boost::mutex> lock(cs_LatestCarPoseAccess);
    latestCarPose = pose;
    RETURN_NOERROR;
}

TPoseStruct::Data cMoveToPoint::GetCarPoseOffset(){
    boost::lock_guard<boost::mutex> lock(cs_PoseOffsetAccess);
    return carPoseOffset;
}

tResult cMoveToPoint::SetCarPoseOffset(TPoseStruct::Data offset){
    boost::lock_guard<boost::mutex> lock(cs_PoseOffsetAccess);
    carPoseOffset = offset;
    RETURN_NOERROR;
}

TPoseStruct::Data cMoveToPoint::GetGoalPose(){
    boost::lock_guard<boost::mutex> lock(cs_GoalPoseAccess);
    return goalPoseIn;
}

//TODO: not used
tResult cMoveToPoint::SetGoalPose(TPoseStruct::Data pose){
    boost::lock_guard<boost::mutex> lock(cs_GoalPoseAccess);
    goalPoseIn = pose;
    RETURN_NOERROR;
}


tResult cMoveToPoint::SetLatestSteeringInput(TSignalValue::Data steering){
    boost::lock_guard<boost::mutex> lock(cs_Steering);
    steeringInput = steering;
    RETURN_NOERROR;
}

// sets path_to_goal
// therefore checks command, set by InternalStateMachine and action_input
static int global_path_id;
tResult cMoveToPoint::SetPath(){
    tUInt32 tmp_command = GetInternalCommand();
    path_to_goal.clear();
    global_path_id = tmp_command;
    if(tmp_command == AC_FP_GOTO_XY || tmp_command == AC_FP_GOTO_XY_NOSTOP){
       path_to_goal.resize(1); // only one point
       TPoseStruct::Data tmp_external_goal = GetGoalPose();

       path_to_goal[0].x = tmp_external_goal.f32PosX;
       path_to_goal[0].y = tmp_external_goal.f32PosY;
       path_to_goal[0].yaw = tmp_external_goal.f32Yaw;
       path_to_goal[0].car_speed = tmp_external_goal.f32CarSpeed;
       path_to_goal[0].stop_after = (tmp_command == AC_FP_GOTO_XY);
       path_to_goal[0].cut_off_dist = tmp_external_goal.f32Radius;
       path_to_goal[0].steering_alpha = 1.0;
       if(!path_to_goal[0].x && path_to_goal[0].y){

           TransmitFeedback(FB_FP_REACHED_POINT);
       }
#ifdef DEBUG
       LOG_SUCCESS(cString::Format("%f   %f   %f   %f   %d    %f", path_to_goal[0].x, path_to_goal[0].y, path_to_goal[0].yaw, path_to_goal[0].car_speed, (int)path_to_goal[0].stop_after, path_to_goal[0].cut_off_dist));
#endif
    } else {
        std::vector<fp_paths>::iterator it = xml_paths.begin();
        // check if path for command id exists
        while(it != xml_paths.end()) {
            if(it->path_id == tmp_command) {
                break;
            }
            it++;
        }
        //add path to path_to_goal variable if path was found
        if (it != xml_paths.end() && it->points.size() > 0) {
                path_to_goal.resize(it->points.size());
                path_to_goal = it->points; // get points
/*#ifdef DEBUG
                for(unsigned int i = 0; i < path_to_goal.size(); ++i){
       LOG_SUCCESS(cString::Format("x: %f  y: %f   yaw: %f   speed: %f   stop: %d   cut: %f", path_to_goal[i].x, path_to_goal[i].y, path_to_goal[i].yaw, path_to_goal[i].car_speed, (int)path_to_goal[i].stop_after, path_to_goal[i].cut_off_dist));
                }
                LOG_INFO("---------from:");
                for(unsigned int i = 0; i < path_to_goal.size(); ++i){
       LOG_SUCCESS(cString::Format("x: %f  y: %f   yaw: %f   speed: %f   stop: %d   cut: %f", it->points[i].x, it->points[i].y, it->points[i].yaw, it->points[i].car_speed, (int)it->points[i].stop_after, it->points[i].cut_off_dist));
                }
#endif */
                //LOG_SUCCESS(cString::Format("load path : %s", it->name));
        } else {
                LOG_ERROR(cString::Format("FollowPath: No points in path for command %d", tmp_command));
                RETURN_ERROR(ERR_INVALID_STATE);
        }

    }
    RETURN_NOERROR;
}


tResult cMoveToPoint::LoadProperties(){


      //TEST CASE
        LOG_SUCCESS("here in load properties");
        mtp_properties properties;
        LoadXMLData("/home/aadc/AADC/utilities/propertyXMLs/mtp_properties.xml", (void*) (&properties));

        mtp_steeringAngle = properties.mtp_steeringAngle;
        mtp_movementSpeed = properties.mtp_movementSpeed;
        mtp_use_absolute_pos = properties.mtp_use_absolute_pos;
        mtp_stop_after_command = properties.mtp_stop_after_command;
        max_speed = properties.max_speed;
        min_speed = properties.min_speed;
        kalpha = properties.kalpha;
        kbeta_t = properties.kbeta_t;
        offset_steering = properties.offset_steering;
        krho_forw = properties.krho_forw;
        krho_back = properties.krho_back;
        reduction_offset = properties.reduction_offset;
        goalDistanceShutOff = properties.goalDistanceShutOff;


        LOG_SUCCESS(cString::Format("mtp_steeringAngle: %f", (float)mtp_steeringAngle));
        LOG_SUCCESS(cString::Format("mtp_movementSpeed: %f", (float)mtp_movementSpeed));
        LOG_SUCCESS(cString::Format("mtp_use_absolute_pos: %f", (float)mtp_use_absolute_pos));
        LOG_SUCCESS(cString::Format("mtp_stop_after_command: %f", (float)mtp_stop_after_command));
        LOG_SUCCESS(cString::Format("max_speed: %f", (float)max_speed));
        LOG_SUCCESS(cString::Format("min_speed: %f", (float)min_speed));
        LOG_SUCCESS(cString::Format("kalpha: %f", (float)kalpha));
        LOG_SUCCESS(cString::Format("kbeta_t: %f", (float)kbeta_t));
        LOG_SUCCESS(cString::Format("offset_steering: %f", (float)offset_steering));
        LOG_SUCCESS(cString::Format("krho_forw: %f", (float)krho_forw));
        LOG_SUCCESS(cString::Format("krho_back: %f", (float)krho_back));
        LOG_SUCCESS(cString::Format("reduction_offset: %f", (float)reduction_offset));
        LOG_SUCCESS(cString::Format("goalDistanceShutOff: %f", (float)goalDistanceShutOff));

    RETURN_NOERROR;


}
tResult cMoveToPoint::LoadPathPointData(){
    boost::lock_guard<boost::mutex> lock(m_ocs_PathAccess);
    using namespace tinyxml2;

    XMLDocument xmlDoc;
    XMLError eResult = xmlDoc.LoadFile(trajectoryPathPointsDirectory);
    if(eResult != XML_SUCCESS) {
        LOG_WARNING(cString::Format("Error while loding the trajectoryPathPointsXML: %i\n", eResult));
        LOG_WARNING(cString::Format("File: %s\n", trajectoryPathPointsDirectory));
        RETURN_ERROR(ERR_UNEXPECTED); //TODO: before -1
    }

    XMLNode * pRoot = xmlDoc.FirstChildElement("calibration");

    if(pRoot == NULL){
        LOG_WARNING(cString::Format("cMoveToPoint::No Root in FollowPathPoints XML"));
        RETURN_ERROR(ERR_UNEXPECTED); //TODO: before -1
    }

	//int cnt = 0;
    XMLElement * pListElementPath = pRoot->FirstChildElement("Path");
    while(pListElementPath != NULL){
        fp_paths new_path;
        pListElementPath->QueryUnsignedAttribute("id", &new_path.path_id);
        new_path.name = pListElementPath->Attribute("name");
        XMLElement * pListElementPoint = pListElementPath->FirstChildElement("point");
		//cnt = 0;
	    LOG_SUCCESS(cString::Format("path id :  %d ", (int)new_path.path_id ));
        while(pListElementPoint != NULL){
            fp_points tmpPoint;
		//++cnt;
		//LOG_INFO(cString::Format("%d",cnt));

            XMLElement * pChildElement = pListElementPoint->FirstChildElement("xValue");
            std::string value = pChildElement->GetText();
            std::replace(value.begin(), value.end(), '.', ',');
            tmpPoint.x = (tFloat32) strtof(value.c_str(), NULL);

            pChildElement = pListElementPoint->FirstChildElement("yValue");
             value = pChildElement->GetText();
            std::replace(value.begin(), value.end(), '.', ',');
            tmpPoint.y = (tFloat32) strtof(value.c_str(), NULL);

            pChildElement = pListElementPoint->FirstChildElement("yawValue");
            value = pChildElement->GetText();
            std::replace(value.begin(), value.end(), '.', ',');
            tmpPoint.yaw = (tFloat32) strtof(value.c_str(), NULL);

            pChildElement = pListElementPoint->FirstChildElement("speedValue");
            value = pChildElement->GetText();
            std::replace(value.begin(), value.end(), '.', ',');
            tmpPoint.car_speed = (tFloat32) strtof(value.c_str(), NULL);

            pChildElement = pListElementPoint->FirstChildElement("stopAfter");
            value = pChildElement->GetText();
            std::replace(value.begin(), value.end(), '.', ',');
            tmpPoint.stop_after = (tBool) strtof(value.c_str(), NULL);

            pChildElement = pListElementPoint->FirstChildElement("cutOffDist");
            value = pChildElement->GetText();
            std::replace(value.begin(), value.end(), '.', ',');
         //   LOG_INFO(cString::Format("%s",value.c_str()));
          //  tmpPoint.cut_off_dist = (tFloat32) stof(value.c_str());
            tmpPoint.cut_off_dist = (tFloat32) strtof(value.c_str(), NULL);


            pChildElement = pListElementPoint->FirstChildElement("steeringAlpha");
            value = pChildElement->GetText();
            std::replace(value.begin(), value.end(), '.', ',');
            tmpPoint.steering_alpha = (tFloat32) strtof(value.c_str(), NULL);

            new_path.points.push_back(tmpPoint);
            pListElementPoint = pListElementPoint->NextSiblingElement("point");
        }
        xml_paths.push_back(new_path);
        pListElementPath = pListElementPath->NextSiblingElement("Path");
    }

#ifdef DEBUG
        for(tUInt i = 0; i < xml_paths.size(); i++){
            fp_paths tmp = xml_paths[i];
            for(tUInt x = 0; x < tmp.points.size(); x++){
                fp_points tmpPoint = tmp.points[x];
                LOG_SUCCESS(cString::Format("cMoveToPoint::path: %d, point %d, f32PosX: %f, f32PosY: %f, f32Yaw: %f, f32cutoff: %f stopAfter: %d  steeringAlpha %f", tmp.path_id, i, tmpPoint.x, tmpPoint.y, tmpPoint.yaw, tmpPoint.cut_off_dist, (int)tmpPoint.stop_after, tmpPoint.steering_alpha));
            }
        }
#endif
    RETURN_NOERROR;
}

tResult cMoveToPoint::ReadInFileCommands(){
    boost::lock_guard<boost::mutex> lock(m_ocs_PathAccess);
    using namespace tinyxml2;

    XMLDocument xmlDoc;
    XMLError eResult = xmlDoc.LoadFile(cString(path_to_follow));
    if(eResult != XML_SUCCESS) {
        LOG_WARNING(cString::Format("Error while loding the path_to_follow: %i\n", eResult));
        LOG_WARNING(cString::Format("File: %s\n", cString(path_to_follow)));
        RETURN_ERROR(ERR_UNEXPECTED);
    }

    XMLNode * pRoot = xmlDoc.FirstChildElement("calibration");

    if(pRoot == NULL){
        LOG_WARNING(cString::Format("cMoveToPoint::No Root in path_to_follow XML"));
        RETURN_ERROR(ERR_UNEXPECTED);
    }

    XMLElement * pListElementPath = pRoot->FirstChildElement("Command");
    while(pListElementPath != NULL){
        tUInt32 command_id;
        tUInt32 command_num;
        const tChar * command_name;
        pListElementPath->QueryUnsignedAttribute("id", &command_id);
        pListElementPath->QueryUnsignedAttribute("num", &command_num);
        command_name = pListElementPath->Attribute("name");
        LOG_INFO(cString::Format("read command: %d %d %s", command_id, command_num, command_name));
        for(tUInt32 i = 0; i < command_num; ++i)
            file_commands.push_back(command_id);
        pListElementPath = pListElementPath->NextSiblingElement("Command");
    }


    RETURN_NOERROR;
}

/*****************************************************************/
/* Implementation of real intelligence of filter                 */
/*****************************************************************/
tResult cMoveToPoint::InternalStateMachine(tTimeStamp time_stamp){

   static boost::mutex cs_ism;
   cs_ism.lock();
    auto fo_command = GetFirstOrderCommand();
    auto car_pose = GetLatestCarPose();
    auto is_running = GetRunningState();

   // LOG_INFO(cString::Format("com: %d running: %d state: %d dont_stop: %d is_follow_file: %d", fo_command, is_running, intern_state.state, intern_state.is_dont_stop(), intern_state.is_follow_file() ));

    if(fo_command == AC_NO_COMMAND){
        //transmit zeros
        if(transmit_zeros(time_stamp)) goto unlock_and_ret;
        // CONTINUE TRAJECTORY
        if(is_running)
        {
            cs_ism.unlock();

            DriveTrajectory();

            //ComputeMoveToPointAlgorithm();
            TransmitSteeringController(time_stamp);
            TransmitSpeedController(time_stamp);
            goto only_return;
        }

        else if(intern_state.is_follow_file())
        {
            if(file_commands.size()){
                fo_command = file_commands.front();
                file_commands.pop_front();
             //   LOG_INFO(cString::Format("set_command from file : %s", FP_ACString.at(fo_command)));
                SetIfNoNewFirstOrderCommand(AC_NO_COMMAND, fo_command);
                goto unlock_and_ret;
            } else {
            //    LOG_WARNING("no commands from file left to execute!");
                intern_state.remove_state(DONT_STOP & FOLLOW_FILE );
                goto unlock_and_ret;
            }
        } else { //IDLE MODE //TODO: stop or not stop criterium
            //if(car_pose.f32CarSpeed != 0){
            //   LOG_INFO(cString::Format("car speed : %d", car_pose.f32CarSpeed));
            //    SetIfNoNewFirstOrderCommand(fo_command, AC_FP_STOP);// <--- should not stop here, only now
            //}
            //enable_transmit_zeros();
           // LOG_INFO("no further commands, IDLE!");

            //transmit zeros
            transmit_zeros(time_stamp);
            goto unlock_and_ret;
        }
    }

    if(fo_command == AC_FP_STOP){
        /*reset steering*/
        if(car_pose.f32CarSpeed == 0){
            //LOG_WARNING("trying to stop car but car already stopped");
            intern_state.remove_state(DONT_STOP & FOLLOW_FILE );
            if(is_running) SetRunningState(tFalse); // do this before changing command
            SetIfNoNewFirstOrderCommand(fo_command, AC_NO_COMMAND);
        } else {
              enable_transmit_zeros();
        }
        reset_steering(time_stamp);
        reset_speed(time_stamp);
    }
    else if(fo_command == AC_FP_SET_FOLLOW_FILE){

        ReadInFileCommands();
        intern_state.set_state(DONT_STOP & FOLLOW_FILE);
        SetIfNoNewFirstOrderCommand(fo_command, AC_NO_COMMAND);
    }
    else if(!is_running){//INIT NEW TRAJECTORY COMMAND

#ifdef DEBUG
        LOG_SUCCESS(cString::Format("init new command:  %d", fo_command));
        if(debugToFile){
            fstream file_pose;
            file_pose.open(debugFileDirectory, ios::out | ios::app);
            file_pose << time_stamp << ":  init new command: " << fo_command << "\n";
            file_pose.close();
        }
#endif
        direction = 0;
        pathToGoalIndex = 0;
        mtp_lastRho = -1;
        SetInternalCommand(fo_command);
        SetCarPoseOffset(GetLatestCarPose());
        tResult res;
        if(IS_FAILED(res = SetPath())){
            LOG_WARNING(cString::Format("Set Path failed: %d",fo_command));
            SetIfNoNewFirstOrderCommand(fo_command, AC_NO_COMMAND);
            cs_ism.unlock();
            RETURN_ERROR(res);
        }
#ifdef DEBUG
        else
            LOG_SUCCESS(cString::Format("Set Path worked: %d",fo_command));
#endif
        //reset_steering(time_stamp);

        SetRunningState(tTrue);
        //counter_drive = 0;
        SetIfNoNewFirstOrderCommand(fo_command, AC_NO_COMMAND);
#ifdef DEBUG
        LOG_SUCCESS(cString::Format("will drive trajectory:  %d", fo_command));
        if(debugToFile){
            fstream file_pose;
            file_pose.open(debugFileDirectory, ios::out | ios::app);
            file_pose << time_stamp << ":  will drive trajectory: " << fo_command << "\n";
            file_pose.close();
        }
#endif
    }
    else // INTERRUPT TRAJECTORY COMMAND (new command while old one still running)
    {
#ifdef DEBUG
        LOG_SUCCESS(cString::Format("interrupted command:  %d", GetInternalCommand()));
        if(debugToFile){
            fstream file_pose;
            file_pose.open(debugFileDirectory, ios::out | ios::app);
            file_pose << time_stamp << ":  interrupted command:  %d " << GetInternalCommand() << "\n";
            file_pose.close();
        }
#endif
        SetRunningState(tFalse);
        //TODO: FEEDBACK f
    }

unlock_and_ret:
    cs_ism.unlock();
only_return:
    RETURN_NOERROR;

}
tResult cMoveToPoint::DriveTrajectory(){
    boost::lock_guard<boost::mutex> lock(cs_Drive);

    if(pathToGoalIndex >= path_to_goal.size()){
        LOG_ERROR(cString::Format("DriveTr: %d", pathToGoalIndex));
        SetRunningState(tFalse);
        RETURN_ERROR(ERR_UNEXPECTED); //TODO: added
    }

    // get car pose and car pose offset
    TPoseStruct::Data car_pose_tmp = GetLatestCarPose();
    TPoseStruct::Data car_pose_offset_tmp = GetCarPoseOffset();

    //-----------------------------------------------------------------------------------------------

    //converting coords from global in local coord system
    /*
     * relativeX = B.x - A.x
     * relativeY = B.y - A.y
     * rotatedX = Cos(-Angle) * relativeX - Sin(-Angle) * relativeY
     * rotatedY = Cos(-Angle) * relativeY + Sin(-Angle) * relativeX //TODO: correct?
     *
     *
    */
    //-----------------------------------------------------------------------------------------------

    // calculate car offset
    car_pose_tmp.f32PosX -= car_pose_offset_tmp.f32PosX;
    car_pose_tmp.f32PosY -= car_pose_offset_tmp.f32PosY;
    car_pose_tmp.f32Yaw -= car_pose_offset_tmp.f32Yaw;


    if(car_pose_offset_tmp.f32Yaw < -M_PI) {
            car_pose_offset_tmp.f32Yaw += 2*M_PI;
    } else if(car_pose_offset_tmp.f32Yaw > M_PI) {
            car_pose_offset_tmp.f32Yaw -= 2*M_PI;
    }

    // rotate coordinates for local pose
    tFloat32 y_sin = car_pose_tmp.f32PosY * sin(car_pose_offset_tmp.f32Yaw);
    tFloat32 x_cos = car_pose_tmp.f32PosX * cos(car_pose_offset_tmp.f32Yaw);
    tFloat32 x_sin = car_pose_tmp.f32PosX * sin(car_pose_offset_tmp.f32Yaw);
    tFloat32 y_cos = car_pose_tmp.f32PosY * cos(car_pose_offset_tmp.f32Yaw);


    tFloat32 temp_1 = x_cos + y_sin;
    tFloat32 temp_2 = y_cos - x_sin; // TODO: seems interchanged to the rezept

    // car_pose_tmp contains now local pose starting at x 0, y 0, yaw 0
    car_pose_tmp.f32PosX = temp_1;
    car_pose_tmp.f32PosY = temp_2;

    //-----------------------------------------------------------------------------------------------

    //-----------------------------------------------------------------------------------------------
    // update speed and mtp_lastRho (distance to next goal point)
    //-----------------------------------------------------------------------------------------------

    // calculate delta x and delta y (car pose - goal point)
    tFloat32 delta_x = car_pose_tmp.f32PosX - path_to_goal[pathToGoalIndex].x;
    tFloat32 delta_y = car_pose_tmp.f32PosY - path_to_goal[pathToGoalIndex].y;

    // distance to goal
    tFloat32 rho = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
    //set lastRho only if distance decreased or first call (where last rho is set to -1)
    if(rho < mtp_lastRho || mtp_lastRho < 0) mtp_lastRho = rho;

    //calculate speed

    TSignalValue::Data car_speed;
    car_speed.f32Value = path_to_goal[pathToGoalIndex].car_speed;
    //-----------------------------------------------------------------------------------------------

    //-----------------------------------------------------------------------------------------------
    //if near next goal (range, point with speed 0.0)
    //-----------------------------------------------------------------------------------------------
    float cutoff_region =  max((float)dt_DistanceToNextPathPoint, path_to_goal[pathToGoalIndex].cut_off_dist);

    if(rho < cutoff_region || car_speed.f32Value == 0.0 || (rho - mtp_lastRho) > 0.1)
        //if(rho < dt_DistanceToNextPathPoint || car_speed.f32Value == 0.0 || (rho - mtp_lastRho) > 0.1)
    {


            //LOG_INFO("%d: stepped into rho < dt_DistanceToNextPathPoint function with own co %f and cor %f", pathToGoalIndex, path_to_goal[pathToGoalIndex].cut_off_dist,cutoff_region);

        //-------------------------------
        // -------------- MISSED POINT
        //-------------------------------
        if(rho - mtp_lastRho > 0.2){
            TransmitFeedback(FB_FP_REACHED_POINT);
            SetRunningState(tFalse);

            LOG_WARNING("%d: distance to goal is increasing -> missed point, stops action",pathToGoalIndex);
            //TODO: what to do here?
          //  if(mtp_stop_after_command)
          //          enable_transmit_zeros();
          //  RETURN_NOERROR;

        }

        //-------------------------------
        // -------------- NEAR POINT
        //-------------------------------
        if(rho < cutoff_region){ // cut off region reached

            if(path_to_goal[pathToGoalIndex].stop_after){
                    enable_transmit_zeros();
                    //LOG_SUCCESS(cString::Format("%d breaked because of cutoff: %f", pathToGoalIndex, cutoff_region));
            }
        }

        //-------------------------------
        // -------------- 0 POINT OOR
        //-------------------------------
        if(car_speed.f32Value == 0.0 && rho > dt_DistanceToNextPathPoint) {
            LOG_WARNING("%d: skipped point with speed 0.0 (and out of range)",pathToGoalIndex);
            enable_transmit_zeros();
        }

        //-------------------------------
        // -------------- NEXT GOAL
        //-------------------------------
        if(pathToGoalIndex < path_to_goal.size() -1){
#ifdef DEBUG
            if(counter_movetopoint%10 == 0 && debugToConsole == true)
                LOG_WARNING("cMoveToPoint:: adding the next point in the path");
#endif
            //if direction changes or car has to stop send a few 0 samples to prevent any damage to drive train of car
            // next point with new direction: set flag and wait a few samples before transmitting new car speed
            if (path_to_goal[pathToGoalIndex+1].car_speed == 0.0 //stop car if next speed is 0 ---better: set stop flag
                    ||   DirChange()
                    ||   path_to_goal[pathToGoalIndex].stop_after)
            {

                    enable_transmit_zeros();
#ifdef DEBUG
                // debug message
                if(counter_movetopoint%10 == 0 && debugToConsole == true) LOG_WARNING(cString::Format("cMoveToPoint:: change in car direction or car speed 0 at next path point, waiting few samples"));
#endif

            }

            // increase path point counter
            ++pathToGoalIndex;
            direction = 0; // reset direction


            //-------------------------------
            // -------------- RESET POSE
            //-------------------------------
            if(!mtp_use_absolute_pos){
                //LOG_SUCCESS("resetted pose for index %d", (int)pathToGoalIndex);
                SetCarPoseOffset(GetLatestCarPose());
                mtp_lastRho =-1;
                RETURN_NOERROR;
            } else {
                // => new delta x and delta y (car pose - goal point)
                delta_x = car_pose_tmp.f32PosX - path_to_goal[pathToGoalIndex].x;
                delta_y = car_pose_tmp.f32PosY - path_to_goal[pathToGoalIndex].y;

                // => new distance to goal
                rho = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
                mtp_lastRho = rho;
                // => new car speed from xml
                car_speed.f32Value = path_to_goal[pathToGoalIndex].car_speed;
            }

        } else { //END OF TRAJECTORY
#ifdef DEBUG
            if(counter_movetopoint%10 == 0 && debugToConsole == true) LOG_WARNING("cMoveToPoint:: no more points in path!");
#endif

            if(mtp_stop_after_command || path_to_goal[pathToGoalIndex].stop_after){
                    TransmitResetPID(m_pClock->GetStreamTime(),true);
                    enable_transmit_zeros();
            }
            TransmitFeedback(FB_FP_REACHED_POINT);
            SetRunningState(false);
        }
    }
    //-----------------------------------------------------------------------------------------------

        tFloat32 reduced_speed = car_speed.f32Value;
if(0){
#ifdef DEBUG
    if(counter_movetopoint%10 == 0 && debugToConsole == true){
        LOG_INFO(cString::Format("MoveToPoint::LatestCarPoseX: %f, LatestCarPoseY: %f, LatestCarPoseYaw: %f", latestCarPose.f32PosX, latestCarPose.f32PosY, latestCarPose.f32Yaw * 180/M_PI));
        LOG_INFO(cString::Format("MoveToPoint::CarPoseOffsetX: %f, CarPoseOffsetY: %f, CarPoseOffsetYaw: %f", car_pose_offset_tmp.f32PosX, car_pose_offset_tmp.f32PosY, car_pose_offset_tmp.f32Yaw * 180/M_PI));
        LOG_INFO(cString::Format("MoveToPoint::DesiredCarPoseX: %f, DesiredCarPoseY: %f, DesiredCarPoseYaw %f", path_to_goal[pathToGoalIndex].x, path_to_goal[pathToGoalIndex].y, path_to_goal[pathToGoalIndex].yaw * 180/M_PI));
        LOG_INFO(cString::Format("MoveToPoint::y_sin: %f, y_cos: %f, x_cos: %f, x_sin: %f", y_sin, y_cos, x_cos, x_sin));
        LOG_INFO(cString::Format("MoveToPoint::TransformedX: %f, TransformedY: %f, TransformedYaw %f", car_pose_tmp.f32PosX, car_pose_tmp.f32PosY, car_pose_tmp.f32Yaw * 180/M_PI));
        LOG_INFO(cString::Format("MoveToPoint::tmpx: %f,  tmpy: %f", temp_1, temp_2));
        LOG_INFO(cString::Format("MoveToPoint::Delta_x: %f, Delta_y: %f, Rho: %f", delta_x, delta_y, rho));
        LOG_INFO(cString::Format("MoveToPoint::car_speed: %f, reduced_speed: %f",  car_speed.f32Value , reduced_speed));
    }
    if(counter_movetopoint%10 == 0 && debugToFile == true){
        fstream file_pose;
        file_pose.open( debugFileDirectory, ios::out | ios::app);
        file_pose << cString::Format("MoveToPoint::LatestCarPoseX: %f, LatestCarPoseY: %f, LatestCarPoseYaw: %f", latestCarPose.f32PosX, latestCarPose.f32PosY, latestCarPose.f32Yaw * 180/M_PI) ;
        file_pose << cString::Format("MoveToPoint::CarPoseOffsetX: %f, CarPoseOffsetY: %f, CarPoseOffsetYaw: %f", car_pose_offset_tmp.f32PosX, car_pose_offset_tmp.f32PosY, car_pose_offset_tmp.f32Yaw * 180/M_PI);
        file_pose << cString::Format("MoveToPoint::DesiredCarPoseX: %f, DesiredCarPoseY: %f, DesiredCarPoseYaw %f", path_to_goal[pathToGoalIndex].x, path_to_goal[pathToGoalIndex].y, path_to_goal[pathToGoalIndex].yaw * 180/M_PI);
        file_pose << cString::Format("MoveToPoint::y_sin: %f, y_cos: %f, x_cos: %f, x_sin: %f", y_sin, y_cos, x_cos, x_sin);
        file_pose << cString::Format("MoveToPoint::TransformedX: %f, TransformedY: %f, TransformedYaw %f", car_pose_tmp.f32PosX, car_pose_tmp.f32PosY, car_pose_tmp.f32Yaw * 180/M_PI);
        file_pose << cString::Format("MoveToPoint::tmpx: %f,  tmpy: %f", temp_1, temp_2);
        file_pose << cString::Format("MoveToPoint::Delta_x: %f, Delta_y: %f, Rho: %f", delta_x, delta_y, rho);
        file_pose << cString::Format("MoveToPoint::car_speed: %f, reduced_speed: %f",  car_speed.f32Value , reduced_speed);
        file_pose.close();
    }
#endif
}

    //-----------------------------------------------------------------------------------------------
    // conditions to reduce speed (in external pose mode, enabled with enable input and last point, direction change in next path pose, stop in next path point (add same pose with 0.0 speed to stop car))
    //-----------------------------------------------------------------------------------------------
    if(GetInternalCommand() == AC_FP_GOTO_XY_NOSTOP){
        //do nothing
    } else if(pathToGoalIndex == path_to_goal.size() - 1 && !mtp_stop_after_command){ //near last point

            reduced_speed = krho_forw * rho * car_speed.f32Value + reduction_offset; //ro = 0,
    // additional points are in the trajectory but a direction change / break (speed =0) happens
    } else if( pathToGoalIndex + 1 <  path_to_goal.size()) {

        if(path_to_goal[pathToGoalIndex+1].car_speed == 0.0 || DirChange() || path_to_goal[pathToGoalIndex].stop_after) { //TODO: set in relation to absolut value of dir change
             reduced_speed = krho_forw * rho * car_speed.f32Value + reduction_offset;
        }
    }
    car_speed.f32Value = reduced_speed;
    //-----------------------------------------------------------------------------------------------

    //-----------------------------------------------------------------------------------------------
    // calculation based on 'polar_sfunc.m' (p. corke robotics toolbox) STEERING
    //-----------------------------------------------------------------------------------------------
    tFloat32 alpha, beta, gamma, beta_;
    if(direction == 0) { // first run
        beta = -atan2(-delta_y, -delta_x);
        alpha = -car_pose_tmp.f32Yaw - beta;

        // first run: direction of travel
        if(alpha > M_PI_2 || alpha < -M_PI_2) {
            direction = -1;
        }else{
              direction = 1;
        }
    } else if(direction == -1) { // backwards
        beta = -atan2(delta_y, delta_x);
        alpha = -car_pose_tmp.f32Yaw - beta;
    } else { // forwards
        beta = -atan2(-delta_y, -delta_x);
        alpha = -car_pose_tmp.f32Yaw - beta;
    }

    beta_ = beta + path_to_goal[pathToGoalIndex].yaw;

    clamp(alpha,(float)-M_PI_2, (float)M_PI_2);

    gamma = direction * (-kbeta_t * beta_ + kalpha * alpha); // gamma
    nextSteeringControllerOut.f32Value = gamma * 180/M_PI;
    //---------------------------------------------------------------------------
--------------------



    //-----------------------------------------------------------------------------------------------
    //limit steering to +-mtp_steeringAngle degrees
    //-----------------------------------------------------------------------------------------------
    nextSteeringControllerOut.f32Value *= - 1.0;
    clamp(nextSteeringControllerOut.f32Value,-(float)mtp_steeringAngle, (float) mtp_steeringAngle);
    if(global_path_id == AC_FP_TURN_LEFT_AFTER_OUTTER_CURVE_DIRECTLY){
        clamp(nextSteeringControllerOut.f32Value,-(float)80, (float) 80);
    }
    TSignalValue::Data ls_steering = GetLatestSteeringInput();
    tFloat32 steering_alpha = path_to_goal[pathToGoalIndex].steering_alpha;
    nextSteeringControllerOut.f32Value *= steering_alpha;
    nextSteeringControllerOut.f32Value += (1.f - steering_alpha) * ls_steering.f32Value;

    nextSpeedControllerOut.f32Value = car_speed.f32Value * direction;
    clamp(nextSpeedControllerOut.f32Value,-(float)mtp_movementSpeed, (float)mtp_movementSpeed);

    //-----------------------------------------------------------------------------------------------

if(0){
    // log MoveToPoint to fileenable mtp
 #ifdef DEBUG
    if ( counter_movetopoint % 10 == 0) { // 1 out of 5
        if(debugToFile){
            fstream file_pose;
            file_pose.open( debugFileDirectory, ios::out | ios::app);
            file_pose <<"distance to goal: " << rho << "  beta: "<< beta << "  alpha: "<< alpha << "  direction: " << direction <<"\n";
            file_pose <<"steering_output: " << nextSteeringControllerOut.f32Value << "  speed_out: " << nextSpeedControllerOut.f32Value << "\n\n";
            file_pose.close();
        }
        if(debugToConsole){
            LOG_INFO(cString::Format("MoveToPoint::beta: %f, beta_: %f, alpha: %f", beta, beta_, alpha));
            LOG_INFO(cString::Format("MoveToPoint::SpeedControllerOut: %f, SteeringControllerOut: %f", nextSpeedControllerOut.f32Value, nextSteeringControllerOut.f32Value));
            LOG_INFO(cString::Format(""));

        }
    }
    counter_movetopoint++;
#endif
}

    RETURN_NOERROR;
}

tResult cMoveToPoint::LogTrajectoryToXML(){
        using namespace tinyxml2;

        if(lt_erase){
            if( remove(loggedXMLDirectory) != 0 ){
    #ifdef DEBUG

               if(debugToConsole){
                    LOG_INFO(cString::Format("MoveToPoint::Error deleting xml file in LogTrajectoryToXML()"));
               }
    #endif
            }
            lt_CarPoseOffset = GetLatestCarPose();
            xmlDoc.Clear();
            pRoot = xmlDoc.NewElement("Path");
            xmlDoc.InsertFirstChild(pRoot);
            RETURN_NOERROR;
        }

        if(firstCallLogTrajectory){
            pRoot = xmlDoc.NewElement("Path");
            xmlDoc.InsertFirstChild(pRoot);
            firstCallLogTrajectory = false;
            lt_CarPoseOffset = GetLatestCarPose();
            lt_counter = 0;
        }

        if(lt_counter % lt_devider == 0){

            XMLElement * pElementPose = xmlDoc.NewElement(cString::Format("Point"));

            // get car pose and car pose offset
            TPoseStruct::Data car_pose_tmp = GetLatestCarPose();
            TPoseStruct::Data car_pose_offset_tmp = lt_CarPoseOffset;

            // calculate car offset
            car_pose_tmp.f32PosX -= car_pose_offset_tmp.f32PosX;
            car_pose_tmp.f32PosY -= car_pose_offset_tmp.f32PosY;
            car_pose_tmp.f32Yaw -= car_pose_offset_tmp.f32Yaw;

            // car pose yaw: convert yaw angle to be between -180 and 180 degrees
            if(car_pose_offset_tmp.f32Yaw < -M_PI) {
                    car_pose_offset_tmp.f32Yaw += 2*M_PI;
            } else if(car_pose_offset_tmp.f32Yaw > M_PI) {
                    car_pose_offset_tmp.f32Yaw -= 2*M_PI;
            }


            // rotate coordinates for local pose
            tFloat32 y_sin = car_pose_tmp.f32PosY * sin(car_pose_offset_tmp.f32Yaw);
            tFloat32 y_cos = car_pose_tmp.f32PosY * cos(car_pose_offset_tmp.f32Yaw);
            tFloat32 x_cos = car_pose_tmp.f32PosX * cos(car_pose_offset_tmp.f32Yaw);
            tFloat32 x_sin = -car_pose_tmp.f32PosX * sin(car_pose_offset_tmp.f32Yaw);

            // car_pose_tmp contains now local pose starting at x 0, y 0, yaw 0
            car_pose_tmp.f32PosX = y_sin + x_cos;
            car_pose_tmp.f32PosY = x_sin + y_cos;


            pElementPose->SetAttribute("f32PosX", car_pose_tmp.f32PosX);
            pElementPose->SetAttribute("f32PosY", car_pose_tmp.f32PosY);
            pElementPose->SetAttribute("f32Yaw", car_pose_tmp.f32Yaw);

            pRoot->InsertEndChild(pElementPose);

            XMLError eResult = xmlDoc.SaveFile(loggedXMLDirectory);
            if (eResult != XML_SUCCESS) {
                LOG_WARNING(cString::Format("cMoveToPoint::Error: %i\n", eResult));

            }
        }
        lt_counter++;

        RETURN_NOERROR;
    }
