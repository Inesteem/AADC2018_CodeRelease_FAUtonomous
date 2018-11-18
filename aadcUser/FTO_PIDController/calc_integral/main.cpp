#include <iostream>
#include <time.h>
#include <string>
#include <cmath>
#include <vector>

//compile with g++ -o out main.cpp --std=c++11

using std::cout;
using std::endl;
using std::string;
using std::vector;


double get_max(double min, double step, double hungry, vector <double> &vec){
    if(hungry < min ) return 0;
    if(hungry >= 1.0 - step) return vec[vec.size() - 1]; // not good! prevent this
    double idx = ((hungry - min)) / step;
        double diff = (idx - (int)idx);
    double diff2 = vec[(int)idx + 1] - vec[(int)idx];
//    cout << (int) idx << " " << (double)idx << " -- " << diff  << " " << diff2 <<  endl;
    return vec[(int) idx] + diff * diff2;
}

int main(int argc, char **argv){


    vector<double> vec;


    if(argc < 4){
        cout << "warning, to less arguments! ./prog error min_hungry step_size" << endl;
        exit(1);
    }


    double max_err = std::stod(string(argv[1]));
    double min_hungry = std::stod(string(argv[2]));
    double step_size = std::stod(string(argv[3]));


    for(double hungry = min_hungry; hungry < 1.0; hungry += step_size){
    
        double last_acc_err = 0;
        double acc_err = max_err;

        while(std::abs((double)(last_acc_err - acc_err)) > 0.00000001){
            last_acc_err = acc_err;
            acc_err *= hungry;
            acc_err += max_err; 
        }
        vec.push_back(acc_err);
 //       cout << hungry << ":  " << acc_err << endl;
    }
    cout << "tFloat32 max_error = " << max_err << ";" << endl;
    cout << "tFloat32 min_hungry = " << min_hungry << ";" << endl;
    cout << "tFloat32 step_size = " <<  step_size << ";" << endl;
    cout << "tFloat32 max_integral_val [" << vec.size() << "] = {";
    for(int i = 0; i < vec.size(); ++i){
        if(!(i%5)) cout << "\n           ";
        cout << vec[i];
        if(i < vec.size() -1) cout << ", ";
    }
    cout << "};" << endl;
/*
    for(double hungry = min_hungry; hungry < 1.0; hungry += step_size){
        double num = hungry + (step_size/2.); 
        double max = get_max(min_hungry, step_size, num , vec) ;
        cout << num << ":   " << max << endl;
        //cout << hungry << ":   " <<  get_max(min_hungry, step_size, hungry , vec) << endl;
   }
   */ 
    return 0;
}      
           
           
           
