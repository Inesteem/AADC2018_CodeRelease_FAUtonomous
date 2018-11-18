/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. Team FAUtonomous. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

This filter will represent our state machine.
First, we will read the structure file, to configure our state machine.
 - Every maneuver consist of steps, that are necessary to execute the maneuver
 - Structure file contains all maneuvers and necessary steps

**********************************************************************/

#include "stdafx.h"
#include "ScmCommunication.h"
#include "OCStateControlManagement.h"

#define A_UTILS_NO_DEPRECATED_WARNING
//#define SCM_STRUCTURE_FILEPATH "/home/aadc/AADC/utilities/SCM_maneuver_structures/ADTF_SCM_maneuvers_FINAL.xml"

// This will define the filter and expose it via plugin class factory.
// Class OCStateControlManagement will be embedded to the filter and called by
// data trigger of "speed" by default.
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(
        CID_OC_STATE_CONTROL_MANAGEMENT_FILTER,	// references to header file
        "OCStateControlManagement",               // label
        OCStateControlManagement,                 // class
//        adtf::filter::pin_trigger({ "TriggerPin" }));	// set trigger pin
 adtf::filter::timer_trigger(2));

// -------------------------------
// --------- CONSTRUCTOR ---------
// -------------------------------
// in the constructor, we will set the adtf streamtypes for signal values and pins
OCStateControlManagement::OCStateControlManagement()
{

    // ------------------------------------------
    SetName("OCStateControlManagement Constructor");

    // -----------------------------------------
    // set pins
    // input
    o_TSignalValue.registerPin(this, m_ReaderTriggerPin           , "TriggerPin"     );
    o_TFeedbackStruct.registerPin(this, m_ReaderFeedbackLineSpecifier           , "feedbackLineSpecifierIn"     );
    o_TFeedbackStruct.registerPin(this, m_ReaderFeedbackMoveToPoint             , "feedbackMoveToPointIn"     );
    o_TFeedbackStruct.registerPin(this, m_ReaderFeedbackMarkerDetector          , "feedbackMarkerDetectorIn"     );
    o_TFeedbackStruct.registerPin(this, m_ReaderFeedbackTimer                   , "feedbackTimerIn"     );
    o_TFeedbackStruct.registerPin(this, m_ReaderFeedbackUltrasonicCheck         , "feedbackUltrasonicCheckIn"     );
    o_TFeedbackStruct.registerPin(this, m_ReaderFeedbackObstacleDetector        , "feedbackObstacleDetectorIn"     );
    o_TFeedbackStruct.registerPin(this, m_ReaderFeedbackLightControl            , "feedbackLightControlIn"     );
    o_TFeedbackStruct.registerPin(this, m_ReaderFeedbackSelectSpeed              , "feedbackSelectSpeedIn"     );
    o_TFeedbackStruct.registerPin(this, m_ReaderFeedbackUltrasonicACC           , "feedbackUltrasonicACCIn"     );
    o_TFeedbackStruct.registerPin(this, m_ReaderFeedbackJuryComm                , "feedbackJuryCommIn"     );
    o_TFeedbackStruct.registerPin(this, m_ReaderFeedbackSpeedController         , "feedbackSpeedControllerIn"     );
    o_TFeedbackStruct.registerPin(this, m_ReaderFeedbackIntersectionDetector    , "feedbackIntersectionDetectorIn"     );

    //output
    o_TActionStruct.registerPin(this, m_WriterActionLineSpecifier           , "ActionLineSpecifierOut"     );
    o_TActionStruct.registerPin(this, m_WriterActionMoveToPoint             , "ActionMoveToPointOut"     );
    o_TActionStruct.registerPin(this, m_WriterActionMarkerDetector          , "ActionMarkerDetectorOut"     );
    o_TActionStruct.registerPin(this, m_WriterActionTimer                   , "ActionTimerOut"     );
    o_TActionStruct.registerPin(this, m_WriterActionUltrasonicCheck         , "ActionUltrasonicCheckOut"     );
    o_TActionStruct.registerPin(this, m_WriterActionObstacleDetector        , "ActionObstacleDetectorOut"     );
    o_TActionStruct.registerPin(this, m_WriterActionLightControl            , "ActionLightControlOut"     );
    o_TActionStruct.registerPin(this, m_WriterActionSelectSpeed              , "ActionSelectSpeedOut"     );
    o_TActionStruct.registerPin(this, m_WriterActionUltrasonicACC           , "ActionUltrasonicACCOut"     );
    o_TActionStruct.registerPin(this, m_WriterActionJuryComm                , "ActionJuryCommOut"     );
    o_TActionStruct.registerPin(this, m_WriterActionSpeedController         , "ActionSpeedControllerOut"     );
    o_TActionStruct.registerPin(this, m_WriterActionIntersectionDetector    , "ActionIntersectionDetectorOut"     );

    // -----------------------------------------
    // set variables
    m_i8CurrentActLvlMode     = NOT_INITIALIZED;
    m_i16CurrentScmManeuverID = -1;
    m_i16CurrentScmStepID     = -1;

    RegisterPropertyVariable("scm communication file", m_scmFile);

}

// ------------------------------
// --------- CONFIGURE ----------
// ------------------------------
// implement configure() to read ALL properties
tResult OCStateControlManagement::Configure()
{
    // load xml files for defining structure of stateController
    RETURN_IF_FAILED(LoadSCMStructureData());

    // set variable after successfully loading the structure files
    m_i16CurrentScmManeuverID = 0;
    m_i16CurrentScmStepID     = 0;
    m_bSpecialCase = tFalse;
    m_ui32CurrentManeuver = -1;

    // START StateControl Management in startup-state, wait for first PASSIVE request to start,
    //  e.g. verification of loaded maneuver list from JuryCommunication
    m_i8CurrentActLvlMode = PASSIVE;

    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    //////LOG_INFO(cString::Format("State Control Management Configuration finished!"));

    // done
    RETURN_NOERROR;
}


// ------------------------------
// ---------- PROCESS -----------
// ------------------------------
// process() is always called each time a trigger occured (new data sample arrive).
// IMPORTANT: Make sure to read ALL samples of ALL readers!
// This is how to read a sample
tResult OCStateControlManagement::Process(tTimeStamp tmTimeOfTrigger)
{
//   static int counter;
//   counter++;
//   LOG_ERROR("%d", counter);

      if(IS_OK(o_TFeedbackStruct.readPin(m_ReaderFeedbackIntersectionDetector, (void *) & m_dataFeedbackIntersectionDetector, timestamp_ISD)))
      {
          timestamp_ISD = m_dataFeedbackIntersectionDetector.ui32ArduinoTimestamp;
          // process data
  //        LOG_WARNING("feedback ISD received: %d", m_dataFeedbackIntersectionDetector.ui32FeedbackStatus);
          RETURN_IF_FAILED(ProcessFeedback(m_dataFeedbackIntersectionDetector));
      }

  if(IS_OK(o_TFeedbackStruct.readPin(m_ReaderFeedbackMarkerDetector, (void *) & m_dataFeedbackMarkerDetector, timestamp_markerDetector)))
  {
      timestamp_markerDetector = m_dataFeedbackMarkerDetector.ui32ArduinoTimestamp;

      if (m_dataFeedbackMarkerDetector.ui32FeedbackStatus == FB_RS_CROSSING_SIGN_SPECIAL_CASE)
      {
          m_bSpecialCase = tTrue;
      }else if (m_dataFeedbackMarkerDetector.ui32FeedbackStatus == FB_RS_CROSSING_SIGN_NORMAL_CASE)
      {
          m_bSpecialCase = tFalse;
      }





      // process data
//        LOG_WARNING("feedback MarkerDetector received: %d", m_dataFeedbackMarkerDetector.ui32FeedbackStatus);
      RETURN_IF_FAILED(ProcessFeedback(m_dataFeedbackMarkerDetector));
  }

    if(IS_OK(o_TFeedbackStruct.readPin(m_ReaderFeedbackJuryComm, (void *) & m_dataFeedbackJuryComm, timestamp_juryComm)) )
    {
        // process data
        timestamp_juryComm = m_dataFeedbackJuryComm.ui32ArduinoTimestamp;

        if(m_dataFeedbackJuryComm.ui32FeedbackStatus == 10531)
        {
            m_bSpecialCase = tFalse;

        }




        LOG_INFO("feedback JuryComm received: %d", m_dataFeedbackJuryComm.ui32FeedbackStatus);
        RETURN_IF_FAILED(ProcessFeedback(m_dataFeedbackJuryComm));
    }

    if(IS_OK(o_TFeedbackStruct.readPin(m_ReaderFeedbackLineSpecifier, (void *) & m_dataFeedbackLineSpecifier, timestamp_lineSpecifier)))
    {
        timestamp_lineSpecifier = m_dataFeedbackLineSpecifier.ui32ArduinoTimestamp;
        // process data
        //LOG_INFO("feedback LineSpecifier received: %d", m_dataFeedbackLineSpecifier.ui32FeedbackStatus);

        RETURN_IF_FAILED(ProcessFeedback(m_dataFeedbackLineSpecifier));
    }

    if(IS_OK(o_TFeedbackStruct.readPin(m_ReaderFeedbackMoveToPoint, (void *) & m_dataFeedbackMoveToPoint, timestamp_moveToPoint)))
    {
        timestamp_moveToPoint = m_dataFeedbackMoveToPoint.ui32ArduinoTimestamp;

        // process data
        //LOG_INFO("feedback MoveToPoint received: %d", m_dataFeedbackMoveToPoint.ui32FeedbackStatus);
        RETURN_IF_FAILED(ProcessFeedback(m_dataFeedbackMoveToPoint));
    }




    if(IS_OK(o_TFeedbackStruct.readPin(m_ReaderFeedbackTimer, (void *) & m_dataFeedbackTimer, timestamp_timer)))
    {
        timestamp_timer = m_dataFeedbackTimer.ui32ArduinoTimestamp;
        // process data
        //LOG_INFO("feedback Timer received: %d", m_dataFeedbackTimer.ui32FeedbackStatus);
        RETURN_IF_FAILED(ProcessFeedback(m_dataFeedbackTimer));
    }


    if(IS_OK(o_TFeedbackStruct.readPin(m_ReaderFeedbackUltrasonicCheck, (void *) & m_dataFeedbackUltrasonicCheck, timestamp_usCheck)))
    {
        timestamp_usCheck = m_dataFeedbackUltrasonicCheck.ui32ArduinoTimestamp;
        // process data
        //LOG_INFO("feedback UltrasonicCheck received: %d", m_dataFeedbackUltrasonicCheck.ui32FeedbackStatus);
        RETURN_IF_FAILED(ProcessFeedback(m_dataFeedbackUltrasonicCheck));
    }


    if(IS_OK(o_TFeedbackStruct.readPin(m_ReaderFeedbackObstacleDetector, (void *) & m_dataFeedbackObstacleDetector, timestamp_obstacleDetector)))
    {
        timestamp_obstacleDetector = m_dataFeedbackObstacleDetector.ui32ArduinoTimestamp;
        // process data
        //LOG_INFO("feedback ObstacleDetector received: %d", m_dataFeedbackObstacleDetector.ui32FeedbackStatus);
        RETURN_IF_FAILED(ProcessFeedback(m_dataFeedbackObstacleDetector));
    }


    if(IS_OK(o_TFeedbackStruct.readPin(m_ReaderFeedbackLightControl, (void *) & m_dataFeedbackLightControl, timestamp_lightControl)))
    {
        timestamp_lightControl = m_dataFeedbackLightControl.ui32ArduinoTimestamp;
        // process data
        //LOG_INFO("feedback LightControl received: %d", m_dataFeedbackLightControl.ui32FeedbackStatus);
        RETURN_IF_FAILED(ProcessFeedback(m_dataFeedbackLightControl));
    }


    if(IS_OK(o_TFeedbackStruct.readPin(m_ReaderFeedbackSelectSpeed, (void *) & m_dataFeedbackSelectSpeed, timestamp_SelectSpeed)))
    {
        ////////LOG_INFO("scm: SelectSpeed received!");
        timestamp_SelectSpeed = m_dataFeedbackSelectSpeed.ui32ArduinoTimestamp;
        //LOG_INFO("feedback SelectSpeed received: %d", m_dataFeedbackSelectSpeed.ui32FeedbackStatus);
        RETURN_IF_FAILED(ProcessFeedback(m_dataFeedbackSelectSpeed));
    }

    if(IS_OK(o_TFeedbackStruct.readPin(m_ReaderFeedbackUltrasonicACC, (void *) & m_dataFeedbackUltrasonicACC, timestamp_usACC)))
    {
        timestamp_usACC = m_dataFeedbackUltrasonicACC.ui32ArduinoTimestamp;
        // process data
//        LOG_INFO("feedback UltrasonicACC received: %d", m_dataFeedbackUltrasonicACC.ui32FeedbackStatus);
        RETURN_IF_FAILED(ProcessFeedback(m_dataFeedbackUltrasonicACC));
    }



    if(IS_OK(o_TFeedbackStruct.readPin(m_ReaderFeedbackSpeedController, (void *) & m_dataFeedbackSpeedController, timestamp_speedController)))
    {
        timestamp_speedController = m_dataFeedbackSpeedController.ui32ArduinoTimestamp;
        // process data
        //LOG_INFO("feedback speedcontroller received: %d", m_dataFeedbackSpeedController.ui32FeedbackStatus);
        RETURN_IF_FAILED(ProcessFeedback(m_dataFeedbackSpeedController));
    }

    // done
    RETURN_NOERROR;
}


// ------------------------------
// ------ OTHER FUNCTIONS -------
// ------------------------------

// LoadSCMStructureData
// reads the xml file which is set in the filter properties
tResult OCStateControlManagement::LoadSCMStructureData()
{
    // Get path of structure file
    m_strStructureFileName = m_scmFile;

    // check if file exits
    if (m_strStructureFileName.IsEmpty())
    {

        RETURN_AND_LOG_ERROR_STR(ERR_INVALID_FILE,cString::Format("SCM: XML-File for structural configuration not found, please check property!"));
    }

    // create absolute path
    m_strStructureFileName = m_strStructureFileName.CreateAbsolutePath(".");
    LOG_INFO(cString::Format(m_strStructureFileName));

    // load file, parse configuration, print the data
    if (cFileSystem::Exists(m_strStructureFileName))
    {

        LOG_INFO("found file");
        cDOM oDOM;
        oDOM.Load(m_strStructureFileName);

        m_vecScmStructureList.clear();

        cDOMElementRefList oScManeuverElems;
        cDOMElementRefList oScStepElems;
        cDOMElement*       oScLevelActiveElem;
        cDOMElement*       oScLevelPassiveElem;
        cDOMElementRefList oScFilterIDActiveElems;
        cDOMElementRefList oScFilterIDPassiveElems;
        cDOMElement*       oScActionElem;
        cDOMElementRefList oScRequestElems;

        /* counters for checking if structural configuration file of OCStateControlManagement was correctly implemented by user, which means
         * that both maneuvers and steps start at id=0 and are implemented in ascending order in the xml-file, with NO gaps existing between ids! */
        tUInt32 manCounter = 0;
        tUInt32 stepCounter = 0;

        //read first scManeuver Element
        if(IS_OK(oDOM.FindNodes("SCM-Structure-List/SCM-Maneuver", oScManeuverElems)))
        {
            LOG_INFO("read first man");
            //iterate through scManeuvers
            for (cDOMElementRefList::iterator itScManeuverElem = oScManeuverElems.begin(); itScManeuverElem != oScManeuverElems.end(); ++itScManeuverElem)
            {
                LOG_INFO("iterate through mans");
                //if scManeuver found
                sc_Maneuver scManeuver; 	// declare object of scManeuver for saving purpose
                scManeuver.id = (*itScManeuverElem)->GetAttributeUInt32("id");
                LOG_INFO("found man id %d", scManeuver.id);
                // make sure that id is in ascending order
                if(manCounter != scManeuver.id)
                {
                    RETURN_AND_LOG_ERROR_STR(ERR_INVALID_FILE,cString::Format("SCM: Invalid structure of xml-file at maneuver %d. Maneuvers must have ascending order and start at ID = 0. No gaps allowed. Check file!",scManeuver.id));
                }

                if(IS_OK((*itScManeuverElem)->FindNodes("SCM-Step", oScStepElems)))
                {
                    stepCounter = 0;
                    // iterate through scSteps
                    for(cDOMElementRefList::iterator itScStepsElem = oScStepElems.begin(); itScStepsElem != oScStepElems.end(); ++itScStepsElem)
                    {
                        // if scStep found
                        sc_Step scStep;
                        scStep.id = (*itScStepsElem)->GetAttributeUInt32("id");
                        // make sure stepIDs are in ascending order
                        if(stepCounter != scStep.id)
                        {
                            RETURN_AND_LOG_ERROR_STR(ERR_INVALID_FILE,cString::Format("SCM: Invalid structure of xml-file in maneuver %d, step %d. Steps must have ascending order and start at ID = 0. No gaps allowed. Check file!",scManeuver.id,scStep.id));
                        }

                        // PATH FOR ACTIVITYLEVEL: ACTIVE
                        if(IS_OK((*itScStepsElem)->FindNode("SCM-ActivityLevel/SCM-Active", oScLevelActiveElem)))
                        {
                            // if scActivityLevel_ACTIVE found
                            if(IS_OK(oScLevelActiveElem->FindNodes("SCM-FilterID", oScFilterIDActiveElems)))
                            {
                                // iterate through scFilterID_Active
                                for(cDOMElementRefList::iterator itScFilterIDActiveElem = oScFilterIDActiveElems.begin(); itScFilterIDActiveElem != oScFilterIDActiveElems.end(); ++itScFilterIDActiveElem)
                                {
                                    // if ScFilterID_Active found, Element of List FilterList active
                                    sc_Filter_active scFilterActive;
                                    scFilterActive.filterID = (*itScFilterIDActiveElem)->GetAttributeUInt32("id");
                                    LOG_INFO("active filter id %d in step %d of manuever %d found!", scFilterActive.filterID, scStep.id, scManeuver.id);
                                    // inside ID, there is only one SCM-Action, so only search for specific node
                                    if(IS_OK((*itScFilterIDActiveElem)->FindNode("SCM-Action", oScActionElem)))
                                    {
                                        scFilterActive.action.enabled = oScActionElem->GetAttributeBool  ("enable");
                                        scFilterActive.action.started = oScActionElem->GetAttributeBool  ("start");
                                        scFilterActive.action.command = oScActionElem->GetAttributeUInt32("command");

                                    }
                                    // Push found FilterActive Commands into Filter-Active-list for current step
                                    scStep.activityLvl.active.FilterList.push_back(scFilterActive);
                                }
                            }
                        }

                        // PATH FOR ACTIVITYLEVEL: PASSIVE
                        if(IS_OK((*itScStepsElem)->FindNode("SCM-ActivityLevel/SCM-Passive", oScLevelPassiveElem)))
                        {
                            //if scActivityLevel_PASSIVE found
                            if(IS_OK(oScLevelPassiveElem->FindNodes("SCM-FilterID", oScFilterIDPassiveElems)))
                            {
                                //iterate through scFilterID_Passive
                                for(cDOMElementRefList::iterator itScFilterIDPassiveElem = oScFilterIDPassiveElems.begin(); itScFilterIDPassiveElem != oScFilterIDPassiveElems.end(); ++itScFilterIDPassiveElem)
                                {
                                    //if ScFilterID_Passive found, Element of List FilterList passive
                                    sc_Filter_passive scFilterPassive;
                                    scFilterPassive.filterID= (*itScFilterIDPassiveElem)->GetAttributeUInt32("id");

                                    // inside ID, there can be several SCM-Requests
                                    if(IS_OK((*itScFilterIDPassiveElem)->FindNodes("SCM-Request", oScRequestElems)))
                                    {
                                        //iterate through scFilterID_Passive
                                        for(cDOMElementRefList::iterator itScReqeustElems = oScRequestElems.begin(); itScReqeustElems != oScRequestElems.end(); ++itScReqeustElems)
                                        {
                                            //if sc_Request found, Element of List Request
                                            sc_Request scRequest;
                                            scRequest.request = (*itScReqeustElems)->GetAttributeUInt32("status");
                                            scRequest.command = (*itScReqeustElems)->GetAttributeUInt32("command");
                                            //                                            //////LOG_INFO("passive filter id %d with request %d and command %d in step %d of manuever %d found!", scFilterPassive.filterID,scRequest.request, scRequest.command,scStep.id, scManeuver.id);
                                            scFilterPassive.Requests.push_back(scRequest);
                                        }

                                    }
                                    // Push found FilterPassive Commands into Filter-Passive-list for current step
                                    scStep.activityLvl.passive.FilterList.push_back(scFilterPassive);

                                }
                            }
                        }
                        // Push found steps into step-list
                        scManeuver.Steps.push_back(scStep);
                        stepCounter++; // increase step counter
                    }
                }

                m_vecScmStructureList.push_back(scManeuver);
                manCounter++; // increase maneuver counter
            }
        }
        cPinWriter m_WriterAction;         // 14: Object Specifier
        // if maneuvers were not loaded succesfully
        if(!(oScManeuverElems.size() > 0))
        {
            RETURN_AND_LOG_ERROR_STR(ERR_INVALID_FILE, cString::Format("SCM: no valid Configuration Data found in file!"));
        }
    }
    else
    {
        LOG_INFO("cant find file");
        RETURN_AND_LOG_ERROR_STR(ERR_INVALID_FILE,cString::Format("SCM: Structural configuration file not found!"));
    }

    LOG_INFO("list size %d", m_vecScmStructureList.size());

    // done
    RETURN_NOERROR;
}

tResult OCStateControlManagement::ProcessFeedback(TFeedbackStruct::Data feedback)
{

    // -------------------------------------------------
    // HANDLING OF SPONTANEOUS INTERRUPTIONS BY JURY -> action_STOP signal sent! has to be treated in every mode
    if(feedback.ui32FeedbackStatus == FB_JURY_COMMAND_RECEIVED_STOP)
    {
        LOG_WARNING(cString::Format("SCM: Interrupted via action_STOP signal from jury! System will be set to startup mode."));

        // set SCM to startup maneuver, as it can be started again from this position via action_START from jury
        JumpToManeuver(0);
        m_i8CurrentActLvlMode = ACTIVE;
        ExecAction();

        /* all filters will be disabled in this way, except of JuryCom, which sends alive-signal;
         *  afterwards, system is reset to beginning of current sector, is in CarState startup mode and
         *  waiting for new Start-Signal from jury */
        RETURN_NOERROR;
    }

    // -------------------------------------------------
    // regular, normal behaviour without interruptions by action_STOP:

    // check if current activity level is active
    if(m_i8CurrentActLvlMode == ACTIVE)
    {
        LOG_WARNING(cString::Format("SCM: System is in ACTIVE mode, but input registered: ID %d, status %d. Input will not be processed.", feedback.ui8FilterId, feedback.ui32FeedbackStatus));
    }
    // check if current activity level is passive, so feedback is requested
    else if(m_i8CurrentActLvlMode == PASSIVE)
    {
        // check if input status is relevant to current SCM-State, return command that should be executed
        tUInt32 ui32ScmCommand = CheckRequestRelevance(feedback);
        // check if command was relevant and successfully decoded (command should not be 0)
        if (ui32ScmCommand != 0)
        {







            // change our internal state (maneuverID and stepID) to current command
            RETURN_IF_FAILED(ExecRequest(ui32ScmCommand));
            // check if ExecRequest was successful
            // Change current activity level mode to ACTIVE, as passive command has been successfully executed
            m_i8CurrentActLvlMode = ACTIVE;
            // corresponding command will be executed
            ExecAction();
        }
    }
    // NOT_INITIALIZED or error occurred
    else
    {
        RETURN_AND_LOG_ERROR_STR(ERR_INVALID_STATE, cString::Format("SCM: ERROR occurred!"));
    }

    // done
    RETURN_NOERROR;
}


// ExecRequest
// Execute the request command by changing the STATE (maneuverId and stepId) of the StateController to inputCommand.
// Begin and End of defined intervals for commands specified in 'ScmCommunication.h'
tResult OCStateControlManagement::ExecRequest(tUInt32 inputCommand)
{
    // save the offset in the following variables
    tUInt32 ui32StepPlusCommandOffset = C_STEP_PLUS_COMMAND_BEGIN;
    tUInt32 ui32StepJumpCommandOffset = C_STEP_JUMP_COMMAND_BEGIN;
    tUInt32 ui32ManeuverCommandOffset = C_MAN_JUMP_COMMAND_BEGIN;
    tUInt32 ui32ManeuverCommandEnd    = C_MAN_JUMP_COMMAND_END;

    // if command is in C_STEP_PLUS_COMMAND_BEGIN to C_STEP_JUMP_COMMAND_BEGIN
    if(inputCommand >= ui32StepPlusCommandOffset && inputCommand < ui32StepJumpCommandOffset)
    {
        // then it is a 'normal' command and we change our internal stepId to the command
        RETURN_IF_FAILED(ChangeCurrentScmStepId(inputCommand - ui32StepPlusCommandOffset));
    }

    // if command is in C_STEP_JUMP_COMMAND_BEGIN to C_MAN_JUMP_COMMAND_BEGIN
    else if(inputCommand >= ui32StepJumpCommandOffset && inputCommand < ui32ManeuverCommandOffset)
    {
        // then it is a 'jump command'
        RETURN_IF_FAILED(JumpToStep(inputCommand- ui32StepJumpCommandOffset));
    }
    // if command is in C_MAN_JUMP_COMMAND_BEGIN to C_MAN_JUMP_COMMAND_END
    else if(inputCommand >= ui32ManeuverCommandOffset && inputCommand < ui32ManeuverCommandEnd)
    {
        // then jump to certain maneuver
        RETURN_IF_FAILED(JumpToManeuver(inputCommand - ui32ManeuverCommandOffset));
    }
    else
    {
        RETURN_AND_LOG_ERROR_STR(ERR_INVALID_STATE, cString::Format("SCM: ERROR in ExecRequest(). Undefined Command. Check ENUM-file"));
    }

    // done
    RETURN_NOERROR;
}

// ChangeCurrentScmStepId
// change internal stepId of state machine
tResult OCStateControlManagement::ChangeCurrentScmStepId(tUInt32 numberOfStepsToDo)
{
    tUInt32 numberOfStepsInCurrentManeuver = m_vecScmStructureList[m_i16CurrentScmManeuverID].Steps.size();
    if((m_i16CurrentScmStepID + numberOfStepsToDo) < numberOfStepsInCurrentManeuver)
    {
        m_i16CurrentScmStepID = m_i16CurrentScmStepID + numberOfStepsToDo;
    }
    else
    {
        RETURN_AND_LOG_ERROR_STR(ERR_INVALID_INDEX, cString::Format("SCM: ERROR in changeStep(). Index exceeds."));
    }

    // done
    RETURN_NOERROR;
}

// JumpToStep
tResult OCStateControlManagement::JumpToStep(tUInt32 inputScmStepId)
{
    tUInt32 stepCount = m_vecScmStructureList[m_i16CurrentScmManeuverID].Steps.size();
    if (inputScmStepId < stepCount)
    {
        m_i16CurrentScmStepID = inputScmStepId;
    }
    else
    {
        RETURN_AND_LOG_ERROR_STR(ERR_INVALID_INDEX, cString::Format("SCM: ERROR in JumpStep(). Index exceeds."));
    }

    // done
    RETURN_NOERROR;
}

// JumpToManeuver
tResult OCStateControlManagement::JumpToManeuver(tUInt32 inputScmManeuverId)
{
    tUInt32 maneuverCount = m_vecScmStructureList.size();

    if (m_bSpecialCase == tTrue && inputScmManeuverId < 6 &&inputScmManeuverId > 2 )
    {
        if (m_ui32CurrentManeuver == 3) // last maneuver left turn
        {
            inputScmManeuverId += 15;
        }
        else if (m_ui32CurrentManeuver == 4) // last maneuver right turn
        {

            inputScmManeuverId += 18;
        }
        m_bSpecialCase = tFalse;
    }

    if (inputScmManeuverId < maneuverCount)
    {
        m_i16CurrentScmManeuverID = inputScmManeuverId;
        m_i16CurrentScmStepID = 0;
    }
    else
    {
        LOG_INFO("index is %d, max is %d", inputScmManeuverId, maneuverCount);
        RETURN_AND_LOG_ERROR_STR(ERR_INVALID_INDEX, cString::Format("SCM: ERROR in JumpToManeuver(). Index exceeds."));
    }

    // done
    RETURN_NOERROR;
}

// ExecAction
// Function that is called for processing the data in activityLevel:Active of current SCM state;
//  calls a transmit function to transmit necessary activation signals & commands to corresponding filters
tResult OCStateControlManagement::ExecAction()
{
    if(m_i8CurrentActLvlMode == ACTIVE)
    {
        tUInt8  tmp_filterId;

        // create ActionStruct of type Data for sending purpose
        TActionStruct::Data tempAction;
        std::vector<sc_Filter_active> tmp_filterlist = m_vecScmStructureList[m_i16CurrentScmManeuverID].Steps[m_i16CurrentScmStepID].activityLvl.active.FilterList;
        // 2018: TActionStruct has only one action command
        m_i8CurrentActLvlMode = PASSIVE;
        // assign values
        for(tUInt32 i = 0; i < tmp_filterlist.size(); i++)
        {
            // get actions
            tempAction.ui32ArduinoTimestamp = m_pClock->GetStreamTime();
            tempAction.ui8FilterId = tmp_filterlist[i].filterID;
            tempAction.bEnabled    = tmp_filterlist[i].action.enabled;
            tempAction.bStarted    = tmp_filterlist[i].action.started;
            tempAction.ui32Command = tmp_filterlist[i].action.command;
            if(tempAction.ui32Command == 3154)
            {
                m_ui32CurrentManeuver = m_i16CurrentScmManeuverID; // get last maneuver, check if left / right turn

            }
            /* change activity level to passive, has to be done here in order to receive all signals/events! */

            /* call transmit function */

            tmp_filterId = tempAction.ui8FilterId;
            LOG_WARNING("sent action: %d", tempAction.ui32Command );
            switch(tmp_filterId)
            {
            case 1:// 01: Line Specifier
                RETURN_IF_FAILED(o_TActionStruct.writePin(m_WriterActionLineSpecifier, (void *) &tempAction, m_pClock->GetStreamTime()));
                break;
            case 20:// 02: Move To Point
                RETURN_IF_FAILED(o_TActionStruct.writePin(m_WriterActionMoveToPoint, (void *) &tempAction, m_pClock->GetStreamTime()));
                break;
            case 3:

                RETURN_IF_FAILED(o_TActionStruct.writePin(m_WriterActionMarkerDetector, (void *) &tempAction, m_pClock->GetStreamTime()));
                break;

            case 4:
                RETURN_IF_FAILED(o_TActionStruct.writePin(m_WriterActionTimer, (void *) &tempAction, m_pClock->GetStreamTime()));
                break;
            case 5:
                RETURN_IF_FAILED(o_TActionStruct.writePin(m_WriterActionUltrasonicCheck, (void *) &tempAction, m_pClock->GetStreamTime()));
                break;
            case 6:
                RETURN_IF_FAILED(o_TActionStruct.writePin(m_WriterActionObstacleDetector, (void *) &tempAction, m_pClock->GetStreamTime()));
                break;
            case 7:
                RETURN_IF_FAILED(o_TActionStruct.writePin(m_WriterActionLightControl, (void *) &tempAction, m_pClock->GetStreamTime()));
                break;
            case 21:
                RETURN_IF_FAILED(o_TActionStruct.writePin(m_WriterActionSelectSpeed, (void *) &tempAction, m_pClock->GetStreamTime()));
                break;
            case 9:
                RETURN_IF_FAILED(o_TActionStruct.writePin(m_WriterActionUltrasonicACC, (void *) &tempAction, m_pClock->GetStreamTime()));
                break;
            case 10:
                RETURN_IF_FAILED(o_TActionStruct.writePin(m_WriterActionJuryComm, (void *) &tempAction, m_pClock->GetStreamTime()));
                break;
            case 19:
                RETURN_IF_FAILED(o_TActionStruct.writePin(m_WriterActionSpeedController, (void *) &tempAction, m_pClock->GetStreamTime()));
                break;
            case 15:
                RETURN_IF_FAILED(o_TActionStruct.writePin(m_WriterActionIntersectionDetector, (void *) &tempAction, m_pClock->GetStreamTime()));
                break;
            }
        }
    }
    else
    {
        RETURN_AND_LOG_ERROR_STR(ERR_INVALID_STATE,cString::Format("SCM: ERROR occurred in ExecAction! ActivityLvlMode is NOT 'ACTIVE'!"));
    }

    // done
    RETURN_NOERROR;
}


// CheckRequestRelevance
tUInt32 OCStateControlManagement::CheckRequestRelevance(TFeedbackStruct::Data feedback)
{
    tUInt32 ret_command = 0;
    // temporary copy of FilterList for current ScmState(Maneuver and step)
    std::vector<sc_Filter_passive> tmp_filterlist = m_vecScmStructureList[m_i16CurrentScmManeuverID].Steps[m_i16CurrentScmStepID].activityLvl.passive.FilterList;
    for(tUInt32 i = 0; i < tmp_filterlist.size(); i++)
    {

        if(tmp_filterlist[i].filterID == feedback.ui8FilterId)
        {

            for(tUInt32 k = 0; k < tmp_filterlist[i].Requests.size(); k++)
            {
                if(tmp_filterlist[i].Requests[k].request == feedback.ui32FeedbackStatus)
                {

                    ret_command = tmp_filterlist[i].Requests[k].command;
                    return ret_command;
                }
            }
        }
    }

    //    //////LOG_INFO(cString::Format("SCM: CheckRequestRelevance failed, status received but not in list: %d",feedback.ui32FeedbackStatus));
    return ret_command;
}
