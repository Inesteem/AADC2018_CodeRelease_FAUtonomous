#include <iostream>
//#include <ADTF3_helper.h>
//#include "ScmCommunication.h"

#include <fstream>



#include <iomanip>
#include <complex>
#include <cmath>
#include <vector>
//**************************************************

using namespace std;


//###########  My Functions  #######################
void LoadCompareSignalFromFile(int readLength,kiss_fft_cpx cpxFftCompareSignal[]) //Readout compare signal values from file
{
    fstream file_compareSignal;
    string line ={};

    file_compareSignal.open(fileDirectoryRawCompareSignal, ios::in | ios::app);

    float tmp[readLength][2]={};
    tInt32 i = 0, k = 0;

    while (getline(file_compareSignal, line) && i<readLength)
    {
        float value;
        k = 0;
        stringstream ss(line);

        while (ss >> value)
        {
            tmp[i][k] = value;
            ++k;
        }
        ++i;
    }
    for(tInt32 u=0; u < readLength; ++u)
    {
//        if(m_propBDebugToConsole == tTrue){LOG_INFO("ReadFromFile: %f, %f", tmp[u][0], tmp[u][1]);}
        cpxFftCompareSignal[u].r = tmp[u][0];
        cpxFftCompareSignal[u].i = tmp[u][1];
    }

    file_compareSignal.close();
}



int main()
{
    cout << "Hello World!" << endl;


    return 0;
}
