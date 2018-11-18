/*********************************************************************
Copyright (c) 2018
Audi Autonomous Driving Cup. Team FAUtonomous. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: ?This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.?
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

This Filter is only used for testing purposes!!

**********************************************************************/

#include "stdafx.h"
#include "ActionStructReceiver.h"

// This will define the filter and expose it via plugin class factory.
// Class cTemplateFilter will be embedded to the filter and called by
// data trigger of "speed" by default.
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(
    CID_ACTION_STRUCT_RECEIVER_FILTER,		// references to header file
    "ActionStructReceiver",              // label
    ActionStructReceiver,                // class
    adtf::filter::pin_trigger({"inputAction"}));	// set trigger pin


// -------------------------------
// --------- CONSTRUCTOR ---------
// -------------------------------
// in the constructor, we will set the adtf streamtypes for signal values and pins
ActionStructReceiver::ActionStructReceiver()
{
    // ------------------------------------------
    // create pointers for adtf streamtypes
    // coding convention:	p******DataType
    object_ptr<IStreamType> pActionDataType;

    // ------------------------------------------
    // load the streamtypes
    // action struct
    createTActionStructMediaDescription(pActionDataType, m_ActionSampleFactory, o_ActionId, "tActionStruct");

    // -----------------------------------------
    // set pins
    Register(m_ReaderAction , "inputAction"  , pActionDataType ); // input
}


// ------------------------------
// --------- CONFIGURE ----------
// ------------------------------
// implement configure() to read ALL properties
tResult ActionStructReceiver::Configure()
{
    // done
    RETURN_NOERROR;
}


// ------------------------------
// ---------- PROCESS -----------
// ------------------------------
// process() is always called each time a trigger occured (new data sample arrive).
// IMPORTANT: Make sure to read ALL samples of ALL readers!
// This is how to read a sample
tResult ActionStructReceiver::Process(tTimeStamp tmTimeOfTrigger)
{
    // read actionstruct
    tActionStruct actionStruct;
    getLastSampleTActionStruct(m_ReaderAction, o_ActionId, m_ActionSampleFactory, actionStruct);

    tActionSub actionSub = readTActionStruct(2, actionStruct);

    LOG_INFO(cString::Format("ActionStructReceiver: rcv %d%d%d%d/%d%d%d%d/%d%d%d%d/%d%d%d%d/%d%d%d%d", actionStruct.action1.ui8FilterId, actionStruct.action1.subAction.bEnabled, actionStruct.action1.subAction.bStarted, actionStruct.action1.subAction.ui32Command,
                                                                                                       actionStruct.action2.ui8FilterId, actionStruct.action2.subAction.bEnabled, actionStruct.action2.subAction.bStarted, actionStruct.action2.subAction.ui32Command,
                                                                                                       actionStruct.action3.ui8FilterId, actionStruct.action3.subAction.bEnabled, actionStruct.action3.subAction.bStarted, actionStruct.action3.subAction.ui32Command,
                                                                                                       actionStruct.action4.ui8FilterId, actionStruct.action4.subAction.bEnabled, actionStruct.action4.subAction.bStarted, actionStruct.action4.subAction.ui32Command,
                                                                                                       actionStruct.action5.ui8FilterId, actionStruct.action5.subAction.bEnabled, actionStruct.action5.subAction.bStarted, actionStruct.action5.subAction.ui32Command));
    LOG_INFO(cString::Format("ActionStructReceiver: looking for filterId2: %d%d%d", actionSub.bEnabled, actionSub.bStarted, actionSub.ui32Command));

    // done
    RETURN_NOERROR;
}


// ------------------------------
// ------ OTHER FUNCTIONS -------
// ------------------------------
