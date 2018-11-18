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
#include "FileWriter.h"
#include "ScmCommunication.h"

// This will define the filter and expose it via plugin class factory.
ADTF_TRIGGER_FUNCTION_FILTER_PLUGIN(
        CID_FILE_WRITER_FILTER,                         // references to header file
        "FileWriter",                                   // label
        FileWriter,                                      // class
        adtf::filter::pin_trigger({"roadSignInput" }));	// set trigger pin


// -------------------------------
// --------- CONSTRUCTOR ---------
// -------------------------------
// in the constructor, we will set the adtf streamtypes for signal values and pins
FileWriter::FileWriter()
{

    // ------------------------------------------
    SetName("FileWriter Constructor");

    // -----------------------------------------
    // set pins
    o_TSignalValue.registerPin(this, m_ReaderSpeed    , "input"     );
    o_TRoadSignExt.registerPin(this, m_ReaderRoadSignExt, "roadSignInput");

    o_TSignalValue.registerPin(this, m_WriterCarSpeed   , "output");


    // -----------------------------------------
    // set property variables
    RegisterPropertyVariable("file (set: 1, meas: 2, out: 3)", mode);
    LOG_INFO("CONSTRUCTOR DONE!");
}


// ------------------------------
// --------- CONFIGURE ----------
// ------------------------------
// implement configure() to read ALL properties
tResult FileWriter::Configure()
{
if (mode == 1)    fs.open ("/home/aadc/AADC/SetSpeed.txt", std::fstream::in | std::fstream::out | std::ios::trunc);
else if (mode == 2)    fs.open ("/home/aadc/AADC/MeasuredSpeed.txt", std::fstream::in | std::fstream::out | std::ios::trunc);
else if (mode == 3)    fs.open ("/home/aadc/AADC/OutputSpeed.txt", std::fstream::in | std::fstream::out | std::ios::trunc);
else if (mode == 4)    fs.open ("/home/aadc/AADC/RoadSigns.txt", std::fstream::in | std::fstream::out | std::ios::trunc);

m_ui32LastTimeStampSpeed = 0;
    m_ui32LastTimeStampSteering = 0;
    m_ui32LastTimeStampRoadSign = 0;
    //get clock object
    RETURN_IF_FAILED(_runtime->GetObject(m_pClock));
    LOG_INFO("CONFIG DONE!");
    // done
    RETURN_NOERROR;
}


// ------------------------------
// ---------- PROCESS -----------
// ------------------------------
// process() is always called each time a trigger occured (new data sample arrive).
// IMPORTANT: Make sure to read ALL samples of ALL readers!
tResult FileWriter::Process(tTimeStamp)
{
	// if new data is roadsign signal
    if(IS_OK(o_TRoadSignExt.readPin(m_ReaderRoadSignExt, (void *) & m_dataRoadSign, m_ui32LastTimeStampRoadSign)))
    {
        m_ui32LastTimeStampRoadSign = m_dataRoadSign.ui32ArduinoTimestamp;


            fs << (m_dataRoadSign.ui32ArduinoTimestamp / 1000000.0)
                << "   "<< m_dataRoadSign.i16Identifier
                << "   "<< m_dataRoadSign.af32TVec[0]
                <<"   " << m_dataRoadSign.af32TVec[1]
                <<"   " << m_dataRoadSign.af32TVec[2]
                <<"   " << m_dataRoadSign.af32RVec[0]
                <<"   " << m_dataRoadSign.af32RVec[1]
                <<"   " << m_dataRoadSign.af32RVec[2] <<"\n";
//            myfile.close();


        // process data

    }
    
    

    // if new data is speed signal
    if(IS_OK(o_TSignalValue.readPin(m_ReaderSpeed, (void *) & m_dataCarSpeedOut, m_ui32LastTimeStampSpeed)))
    {
        m_ui32LastTimeStampSpeed = m_dataCarSpeedOut.ui32ArduinoTimestamp;


            fs << (m_dataCarSpeedOut.ui32ArduinoTimestamp / 1000000.0) << "   " << m_dataCarSpeedOut.f32Value << "\n";
//            myfile.close();


        // process data

    }
    
    


    // done
    RETURN_NOERROR;
}
// ------------------------------
// ------ OTHER FUNCTIONS -------
// ------------------------------

