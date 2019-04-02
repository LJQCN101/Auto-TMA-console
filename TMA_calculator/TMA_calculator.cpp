// TMA_calculator.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include <iostream>
#include <math.h>
#include <vector>
#include <thread>
#include <dlib/matrix.h>
#include <dlib/optimization.h>
#include <dlib/global_optimization.h>


using namespace std;

vector<double> bearing = vector<double>(20, 0.0); //target bearing
vector<double> recording_time = vector<double>(20, 0.0);

//ownship coordinate (m,n)
vector<double> m = vector<double>(20, 0.0); 
vector<double> n = vector<double>(20, 0.0);

unsigned int _j = 2; //iteration index
double own_ship_hdg = 0.0; //ownship heading
double current_distance = 0.0; //target distance at last bearing
double last_travel_distance = 0.0; //ownship travel straight distance from last position
double last_travel_direction = 0.0; //ownship travel straight direction from last position

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

        //double penalty_for_spd = pow(limit(0.5 - spd, 0.0, 999999.0) * 100.0, 2) + pow(limit(spd - 6.0, 0.0, 999999.0) * 100.0, 2); //set speed limit: from 0.5 to 6.0 m/s
        //double penalty_for_range = pow(limit(500 - L1_distance, 0.0, 999999.0) * 0.1, 2) + pow(limit(L1_distance - 10000.0, 0.0, 999999.0) * 0.1, 2); //set limit for target distance at t1: from 500m to 10km

        return total_error;
    };


    cout << "NOTE: Ownship straight direction from last position means the direction pointing from your position at last time interval to your current position. It may not be the same as ownship heading after steering the boat." << endl;
    cout << endl;

    cout << "time t1 = 0 (sec)" << endl;

    cout << "ownship heading at t1 (deg): ";
    cin >> own_ship_hdg;
    cout << endl;

    cout << "target bearing at t1 (deg): ";
    cin >> bearing[0];
    cout << endl;
    bearing[0] += own_ship_hdg;

    cout << "ownship straight direction from last position = 0 (deg)" << endl;
    cout << "ownship straight distance from last position = 0 (meter)" << endl;

    cout << "*******************************" << endl;

    cout << "time t2 (sec): ";
    cin >> recording_time[1];
    cout << endl;

    cout << "ownship heading at t2: ";
    cin >> own_ship_hdg;
    cout << endl;

    cout << "target bearing at t2 (deg): ";
    cin >> bearing[1];
    cout << endl;
    bearing[1] += own_ship_hdg;

    cout << "ownship straight direction from last position (deg): ";
    cin >> last_travel_direction;
    cout << endl;

    cout << "ownship straight distance from last position (meter): ";
    cin >> last_travel_distance;
    cout << endl;

    m[1] = last_travel_distance * sin(last_travel_direction * deg_to_rad);
    n[1] = last_travel_distance * cos(last_travel_direction * deg_to_rad);

    //start input iteration
    for (unsigned int j = 2; j < 20; j++)
    {
        _j = j;

        cout << "*******************************" << endl;

        cout << "time t" << j + 1 << " (sec): ";
        cin >> recording_time[j];
        cout << endl;

        cout << "ownship heading at t" << j + 1 << ": ";
        cin >> own_ship_hdg;
        cout << endl;

        cout << "target bearing at t"<< j + 1 <<" (deg): ";
        cin >> bearing[j];
        cout << endl;
        bearing[j] += own_ship_hdg;


        cout << "ownship straight direction from last position (deg): ";
        cin >> last_travel_direction;
        cout << endl;

        cout << "ownship straight distance from last position (meter): ";
        cin >> last_travel_distance;
        cout << endl;

        m[j] = m[j - 1] + last_travel_distance * sin(last_travel_direction * deg_to_rad);
        n[j] = n[j - 1] + last_travel_distance * cos(last_travel_direction * deg_to_rad);

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

                        cout << "target course: " << starting_point(2) << "deg, speed: " << optimize_spd * ms_to_kts << "knots, distance: " << optimize_current_distance << "m, error: " << total_error << " squared m." << endl;
                    }
                }
            }
        }  
    }
}
