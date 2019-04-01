// TMA_calculator.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include <iostream>
#include <math.h>
#include <vector>
#include <thread>
#include "iterator.h"
#include <dlib/matrix.h>
#include <dlib/optimization.h>
#include <dlib/global_optimization.h>
#include <wchar.h>
#include <locale.h>
#include <io.h>
#include <cstdio>
#include <cwchar>
#include <fcntl.h>


using namespace std;

vector<double> bearing = vector<double>(20, 0.0); //target bearing
vector<double> recording_time = vector<double>(20, 0.0);

//ownship coordinate (m,n)
vector<double> m = vector<double>(20, 0.0); 
vector<double> n = vector<double>(20, 0.0);

unsigned int _j = 2; //iteration index
double own_ship_hdg = 0.0; //ownship heading
double current_distance = 0.0; //target distance at last bearing

typedef dlib::matrix<double, 0, 1> column_vector;

// Simple upper and lower limiter
double limit(double input, double lower_limit, double upper_limit)
{
    if (input > upper_limit)
    {
        return upper_limit;
    }
    else if (input < lower_limit)
    {
        return lower_limit;
    }
    else
    {
        return input;
    }
}

double calculate_error(const double &L1_distance, const double &spd, const double &crs)
{

    double u = spd * sin(crs * deg_to_rad); //target speed component on x axis
    double v = spd * cos(crs * deg_to_rad); //target speed component on y axis

    //target coordinate (a,b) at first bearing
    double a = L1_distance * sin(bearing[0] * deg_to_rad); //L1_distance = target distance at first bearing
    double b = L1_distance * cos(bearing[0] * deg_to_rad);

    double total_error = 0.0;

    for (unsigned int i = 0; i < _j + 1; i++)
    {
        //target coordinate (x,y) at time
        double x = a + u * recording_time[i];
        double y = b + v * recording_time[i];
        double line_error = (y - n[i]) * sin(bearing[i] * deg_to_rad) - (x - m[i]) * cos(bearing[i] * deg_to_rad);
        total_error += pow(line_error, 2);

        current_distance = sqrt(pow(y - n[i], 2) + pow(x - m[i], 2));
    }

    return total_error;
}


int main()
{
    //_setmode(_fileno(stdout), _O_U16TEXT); //For displaying Chinese texts.

    double optimize_L1_distance;
    double optimize_spd;
    double optimize_current_distance;

    //target function to minimize using BFGS algorithm
    auto target_function = [](const column_vector& mStartingPoint)
    {
        const double L1_distance = mStartingPoint(0);
        const double spd = mStartingPoint(1);
        const double crs = mStartingPoint(2);

        double u = spd * sin(crs * deg_to_rad);
        double v = spd * cos(crs * deg_to_rad);
        double a = L1_distance * sin(bearing[0] * deg_to_rad);
        double b = L1_distance * cos(bearing[0] * deg_to_rad);

        double total_error = 0.0;

        for (unsigned int i = 0; i < _j + 1; i++)
        {
            double x = a + u * recording_time[i];
            double y = b + v * recording_time[i];
            double line_error = (y - n[i]) * sin(bearing[i] * deg_to_rad) - (x - m[i]) * cos(bearing[i] * deg_to_rad);
            total_error += pow(line_error, 2);
        }

        double penalty_for_spd = pow(limit(0.5 - spd, 0.0, 999999.0) * 100.0, 2) + pow(limit(spd - 6.0, 0.0, 999999.0) * 100.0, 2); //set speed limit: from 0.5 to 6.0 m/s
        double penalty_for_range = pow(limit(500 - L1_distance, 0.0, 999999.0) * 0.1, 2) + pow(limit(L1_distance - 10000.0, 0.0, 999999.0) * 0.1, 2); //set limit for target distance at t1: from 500m to 10km

        return total_error + penalty_for_spd + penalty_for_range;
    };


    //wcout << L"说明：潜艇初始位置为坐标原点，Y轴方向为正北。" << endl;
    cout << "NOTE: Ownship initiates at coordinate (0,0). Y axis points at North. X axis points at East." << endl;
    cout << endl;

    //wcout << L"潜艇航向 h1: ";
    cout << "ownship heading h1 (deg): ";
    cin >> own_ship_hdg;
    cout << endl;


    //wcout << L"第1次测得方位角（相对于潜艇）: ";
    cout << "target bearing b1 (deg): ";
    cin >> bearing[0];
    cout << endl;
    bearing[0] += own_ship_hdg;

    //wcout << L"时间点t1 = 0（秒）" << endl;
    cout << "time t1 = 0 (sec)" << endl;
    //wcout << L"潜艇 X轴位置 m1 = 0（米）" << endl;
    cout << "ownship X axis position m1 = 0 (meter)" << endl;
    //wcout << L"潜艇 Y轴位置 n1 = 0（米）" << endl;
    cout << "ownship Y axis position n1 = 0 (meter)" << endl;

    cout << "*******************************" << endl;

    //wcout << L"潜艇航向 h2: ";
    cout << "ownship heading h2: ";
    cin >> own_ship_hdg;
    cout << endl;

    //wcout << L"第2次测得方位角: ";
    cout << "target bearing b2 (deg): ";
    cin >> bearing[1];
    cout << endl;
    bearing[1] += own_ship_hdg;

    //wcout << L"时间点t2（秒）: ";
    cout << "time t2 (sec): ";
    cin >> recording_time[1];
    cout << endl;

    //wcout << L"潜艇 X轴位置 m2（米）: ";
    cout << "ownship X axis position m2 (meter): ";
    cin >> m[1];
    cout << endl;

    //wcout << L"潜艇 Y轴位置 n2（米）: ";
    cout << "ownship Y axis position n2 (meter): ";
    cin >> n[1];
    cout << endl;

    //start input iteration
    for (unsigned int j = 2; j < 20; j++)
    {
        _j = j;

        cout << "*******************************" << endl;

        //wcout << L"潜艇航向 h" << j + 1 << ": ";
        cout << "ownship heading h" << j + 1 << ": ";
        cin >> own_ship_hdg;
        cout << endl;

        //wcout << L"第" << j + 1 << L"次测得方位角: ";
        cout << "target bearing b"<< j + 1 <<" (deg): ";
        cin >> bearing[j];
        cout << endl;
        bearing[j] += own_ship_hdg;

        //wcout << L"时间点t" << j + 1 << L"（秒）: ";
        cout << "time t"<< j + 1 <<" (sec): ";
        cin >> recording_time[j];
        cout << endl;

        //wcout << L"潜艇 X轴位置 m" << j + 1 << L"（米）: ";
        cout << "ownship X axis position m" << j + 1 << " (meter): ";
        cin >> m[j];
        cout << endl;

        //wcout << L"潜艇 Y轴位置 n" << j + 1 << L"（米）: ";
        cout << "ownship Y axis position n" << j + 1 << " (meter): ";
        cin >> n[j];
        cout << endl;

        cout << "" << endl;
        column_vector starting_point = { 1000.0,1.0,0.0 };
        vector<double> optimal_crs;
        
        //multiple starting point for BFGS algorithm to find for multiple local minimal. (We need to keep all possible results for TMA)

        for (double L1_distance = 1000.0; L1_distance <= 5000.0; L1_distance += 1000.0)
        {
            for (double spd = 1.0; spd < 10.0; spd += 2.0)
            {
                for (double crs = 0.0; crs <= 360.0; crs += 60.0)
                {
                    starting_point = { L1_distance,spd,crs };

                    dlib::find_min_using_approximate_derivatives(dlib::bfgs_search_strategy(), dlib::objective_delta_stop_strategy(1e-7), target_function, starting_point, -1);
                    double total_error = calculate_error(starting_point(0), starting_point(1), starting_point(2));

                    //adjust course result within 0-360 range
                    while (starting_point(2) < 0)
                    {
                        starting_point(2) += 360;
                    }

                    while (starting_point(2) >= 360)
                    {
                        starting_point(2) -= 360;
                    }

                    starting_point(2) = round(starting_point(2) * 100.0) / 100.0;

                    if (starting_point(1) > 0 && find(optimal_crs.begin(), optimal_crs.end(), starting_point(2)) == optimal_crs.end()) {
                        optimal_crs.push_back(starting_point(2));
                        optimize_L1_distance = starting_point(0);
                        optimize_spd = starting_point(1);
                        optimize_current_distance = current_distance;

                        //wcout << L"敌舰航向: " << optimize_crs << L"度, 速度: " << optimize_spd * ms_to_kts << L"节, 距离: " << optimize_current_distance << L"米, 误差: " << total_error << L"平方米" << endl;
                        cout << "target course: " << starting_point(2) << "deg, speed: " << optimize_spd * ms_to_kts << "knots, distance: " << optimize_current_distance << "m, error: " << total_error << " squared m." << endl;
                    }
                }
            }
        }  
    }
}
