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

#include "GraphPlotWidget.h"

cGraphPlotWidget::cGraphPlotWidget(QWidget *parent) : QWidget(parent), ui(new Ui_SensorVisualizationUi)
{
    ui->setupUi(this);

    //setWindowFlags(Qt::FramelessWindowHint);
    //setWindowFlags(Qt::Window | Qt::FramelessWindowHint);

    //this->setStyleSheet("background-color: white;");

    memset(&m_error, 0, sizeof(tSignalValue));

    QTimer *timer_plot = new QTimer(this);



    /* Add graph and set the curve line color to green */
        ui->CustomPlot->addGraph();
        ui->CustomPlot->graph(0)->setPen(QPen(Qt::red));
        ui->CustomPlot->graph(0)->setAntialiasedFill(false);

        /* Configure x-Axis as time in secs */
        QSharedPointer<QCPAxisTickerTime> timeTicker(new QCPAxisTickerTime);
        timeTicker->setTimeFormat("%s");
        ui->CustomPlot->xAxis->setTicker(timeTicker);
        ui->CustomPlot->axisRect()->setupFullAxesBox();


        /* Configure x and y-Axis to display Labels */
        ui->CustomPlot->xAxis->setTickLabelFont(QFont(QFont().family(),8));
        ui->CustomPlot->yAxis->setTickLabelFont(QFont(QFont().family(),8));
        ui->CustomPlot->xAxis->setLabel("Time(s)");
        ui->CustomPlot->yAxis->setLabel("ADC Raw Counts");

        /* Make top and right axis visible, but without ticks and label */
        ui->CustomPlot->xAxis2->setVisible(true);
        ui->CustomPlot->yAxis->setVisible(true);
        ui->CustomPlot->xAxis2->setTicks(false);
        ui->CustomPlot->yAxis2->setTicks(false);
        ui->CustomPlot->xAxis2->setTickLabels(false);
        ui->CustomPlot->yAxis2->setTickLabels(false);



   // connect(ui->doubleSpinBox_ls_max_dist, SIGNAL(valueChanged(double)), ui->us_widget, SLOT(setLSMaxDist(double)));

    connect(timer_plot, SIGNAL(timeout()), this, SLOT(update_gui()));
    timer->start(50);

  //  emit ui->doubleSpinBox_ls_max_dist->setValue(5.0);
}


cGraphPlotWidget::~cGraphPlotWidget()
{
    delete ui;
}

void cGraphPlotWidget::setError(TSignalValue::Data err)
{
    m_error.ui32ArduinoTimestamp = err.ui32ArduinoTimestamp;
    m_error.f32Value = err.f32Value;

    //m_frameRates[aadc::US_SIDELEFT].calc();
}


void cGraphPlotWidget::update_gui()
{
    static long lastTime =0;
    if(lastTime == m_error.ui32ArduinoTimestamp) return;
    double key = (m_error.ui32ArduinoTimestamp-lastTime)/1000.0;
    lastTime = m_error.ui32ArduinoTimestamp;

        static double lastPointKey = 0;
        if(key - lastPointKey > 0.002)
        {
            ui->CustomPlot->graph(0)->addData(key, (double) m_error.f32Value);
            lastPointKey = key;
        }

        /* make key axis range scroll right with the data at a constant range of 8. */
        ui->CustomPlot->graph(0)->rescaleValueAxis();
        ui->CustomPlot->xAxis->setRange(key, 8, Qt::AlignRight);
    ui->CustomPlot->replot();

}
