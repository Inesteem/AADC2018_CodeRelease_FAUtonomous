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


#include "GraphPlotFilter.h"


ADTF_PLUGIN(LABEL_ADTF_GRAPH_PLOT, cGraphPlotFilter)


cGraphPlotFilter::cGraphPlotFilter() : m_pUiFileWidget(nullptr)
{
    signal_value.registerPin(this, m_oInputError      , "error"   );
    create_pin(*this, m_oInputError, "error", signal_value.object);
}

cGraphPlotFilter::~cGraphPlotFilter()
{

}


QWidget* cGraphPlotFilter::CreateView()
{
    // use single UI File in background
    m_pUiFileWidget = new cGraphPlotWidget(nullptr);


    tBool isConnected = tTrue;

    isConnected &= tBool(connect(this, SIGNAL(setError(TSignalValue::Data)), m_pUiFileWidget, SLOT(setError(TSignalValue::Data))));

    if (!isConnected)
    {
        LOG_WARNING("GraphPlot not all signal could be connected to a slot!");
    }
    return m_pUiFileWidget;
}

tVoid cGraphPlotFilter::ReleaseView()
{
    delete m_pUiFileWidget;
    m_pUiFileWidget = nullptr;
}

tResult cGraphPlotFilter::OnIdle()
{
    std::lock_guard<std::mutex> oGuard(m_oMutex);

    RETURN_NOERROR;
}

tResult cGraphPlotFilter::OnTimer()
{
    std::lock_guard<std::mutex> oGuard(m_oMutex);


    tResult res;
    TSignalValue::Data inputError;
    static tTimeStamp ts_LastErr = 0;

    if(IS_OK(res = signal_value.readPin(m_oInputError, (void *) &inputError, ts_LastErr))){
        emit(setError(inputError));
        ts_LastErr = inputError.ui32ArduinoTimestamp;
    }

    RETURN_NOERROR;
}

tResult cGraphPlotFilter::Init(tInitStage eStage)
{
    RETURN_IF_FAILED(adtf::ui::cQtUIFilter::Init(eStage));
    RETURN_NOERROR;
}






