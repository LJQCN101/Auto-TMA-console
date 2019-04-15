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
vector<double> bearing_noisy = vector<double>(20, 0.0); //target bearing with random inaccuracies
vector<double> recording_time = vector<double>(20, 0.0);

//ownship coordinate (m,n)
vector<double> m = vector<double>(20, 0.0); 
vector<double> n = vector<double>(20, 0.0);

unsigned int _j = 2; //iteration index
double own_ship_hdg = 0.0; //ownship heading
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

vector<double> calculate_last_brg(const double &L1_distance, const double &spd, const double &crs)
{
    vector<double> out = vector<double>(3, 0.0);
    double u = spd * sin(crs * deg_to_rad); //target speed component on x axis
    double v = spd * cos(crs * deg_to_rad); //target speed component on y axis

    //target coordinate (a,b) at first bearing
    double a = L1_distance * sin(bearing[0] * deg_to_rad); //L1_distance = target distance at first bearing
    double b = L1_distance * cos(bearing[0] * deg_to_rad);

    //target coordinate (x,y) at last bearing
    double x = a + u * recording_time[_j];
    double y = b + v * recording_time[_j];

    out[0] = x;
    out[1] = y;
    out[2] = sqrt(pow(y - n[_j], 2) + pow(x - m[_j], 2));

    return out;
}

vector<double> calculate_last_brg_noisy(const double &L1_distance, const double &spd, const double &crs)
{
    vector<double> out = vector<double>(3, 0.0);
    double u = spd * sin(crs * deg_to_rad); //target speed component on x axis
    double v = spd * cos(crs * deg_to_rad); //target speed component on y axis

    //target coordinate (a,b) at first bearing
    double a = L1_distance * sin(bearing_noisy[0] * deg_to_rad); //L1_distance = target distance at first bearing
    double b = L1_distance * cos(bearing_noisy[0] * deg_to_rad);

    //target coordinate (x,y) at last bearing
    double x = a + u * recording_time[_j];
    double y = b + v * recording_time[_j];

    out[0] = x;
    out[1] = y;
    out[2] = sqrt(pow(y - n[_j], 2) + pow(x - m[_j], 2));

    return out;
}


int main()
{
    double optimize_L1_distance;
    double optimize_spd;
    double optimize_current_distance;
    double optimize_x;
    double optimize_y;

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

    auto target_function_noisy = [](const column_vector& mStartingPoint)
    {
        const double L1_distance = mStartingPoint(0);
        const double spd = mStartingPoint(1);
        const double crs = mStartingPoint(2);

        double u = spd * sin(crs * deg_to_rad);
        double v = spd * cos(crs * deg_to_rad);
        double a = L1_distance * sin(bearing_noisy[0] * deg_to_rad);
        double b = L1_distance * cos(bearing_noisy[0] * deg_to_rad);

        double total_error = 0.0;

        for (unsigned int i = 0; i < _j + 1; i++)
        {
            double x = a + u * recording_time[i];
            double y = b + v * recording_time[i];
            double line_error = (y - n[i]) * sin(bearing_noisy[i] * deg_to_rad) - (x - m[i]) * cos(bearing_noisy[i] * deg_to_rad);
            total_error += pow(line_error, 2);
        }
        return total_error;
    };


    std::cout << "NOTE: Ownship straight direction from last position means the direction pointing from your position at last time interval to your current position. It may not be the same as ownship heading after steering the boat." << endl;
    std::cout << endl;

    std::cout << "time t1 = 0 (sec)" << endl;

    std::cout << "ownship heading at t1 (deg): ";
    cin >> own_ship_hdg;
    std::cout << endl;

    std::cout << "target bearing at t1 (deg): ";
    cin >> bearing[0];
    std::cout << endl;
    bearing[0] += own_ship_hdg;

    std::cout << "ownship straight direction from last position = 0 (deg)" << endl;
    std::cout << "ownship straight distance from last position = 0 (meter)" << endl;

    std::cout << "*******************************" << endl;

    std::cout << "time t2 (sec): ";
    cin >> recording_time[1];
    std::cout << endl;

    std::cout << "ownship heading at t2: ";
    cin >> own_ship_hdg;
    std::cout << endl;

    std::cout << "target bearing at t2 (deg): ";
    cin >> bearing[1];
    std::cout << endl;
    bearing[1] += own_ship_hdg;

    std::cout << "ownship straight direction from last position (deg): ";
    cin >> last_travel_direction;
    std::cout << endl;

    std::cout << "ownship straight distance from last position (meter): ";
    cin >> last_travel_distance;
    std::cout << endl;

    m[1] = last_travel_distance * sin(last_travel_direction * deg_to_rad);
    n[1] = last_travel_distance * cos(last_travel_direction * deg_to_rad);

    //start input iteration
    for (unsigned int j = 2; j < 20; j++)
    {
        _j = j;

        std::cout << "*******************************" << endl;

        std::cout << "time t" << j + 1 << " (sec): ";
        cin >> recording_time[j];
        std::cout << endl;

        std::cout << "ownship heading at t" << j + 1 << ": ";
        cin >> own_ship_hdg;
        std::cout << endl;

        std::cout << "target bearing at t"<< j + 1 <<" (deg): ";
        cin >> bearing[j];
        std::cout << endl;
        bearing[j] += own_ship_hdg;


        std::cout << "ownship straight direction from last position (deg): ";
        cin >> last_travel_direction;
        std::cout << endl;

        std::cout << "ownship straight distance from last position (meter): ";
        cin >> last_travel_distance;
        std::cout << endl;

        m[j] = m[j - 1] + last_travel_distance * sin(last_travel_direction * deg_to_rad);
        n[j] = n[j - 1] + last_travel_distance * cos(last_travel_direction * deg_to_rad);

        std::cout << "" << endl;
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
                    vector<double> last_brg = calculate_last_brg(starting_point(0), starting_point(1), starting_point(2));

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

                    if (starting_point(0) > 0.1 && starting_point(1) > 0.0 && find(optimal_crs.begin(), optimal_crs.end(), starting_point(2)) == optimal_crs.end()) {
                        optimal_crs.push_back(starting_point(2));
                        optimize_L1_distance = starting_point(0);
                        optimize_spd = starting_point(1);
                        optimize_current_distance = last_brg[2];
                        optimize_x = last_brg[0];
                        optimize_y = last_brg[1];

                        std::cout << "target course: " << starting_point(2) << "deg, speed: " << optimize_spd * ms_to_kts << "knots, distance: " << optimize_current_distance << "m" << endl;
                    }
                }
            }
        }

        //error analysis
        if (optimal_crs.size() > 0)
        {
            double last_opt_crs = optimal_crs[optimal_crs.size() - 1];
            int total_error_count = 0;
            int error_within_75m = 0;
            int error_within_150m = 0;
            int error_within_300m = 0;
            int error_within_5deg = 0;
            int error_within_10deg = 0;
            int error_within_20deg = 0;

            for (double L1_distance = optimize_L1_distance; L1_distance <= optimize_L1_distance + 1.0; L1_distance += 0.1)
            {
                for (double spd = optimize_spd; spd <= optimize_spd + 0.1; spd += 0.01)
                {
                    for (double crs = last_opt_crs; crs <= last_opt_crs + 1.0; crs += 0.1)
                    {
                        starting_point = { L1_distance,spd,crs };

                        for (unsigned int k = 0; k < _j + 1; k++)
                        {
                            bearing_noisy[k] = bearing[k] + (rand() % 11 - 5) / 10.0;
                        }

                        dlib::find_min_using_approximate_derivatives(dlib::bfgs_search_strategy(), dlib::objective_delta_stop_strategy(1e-7), target_function_noisy, starting_point, -1);
                        vector<double> last_brg = calculate_last_brg_noisy(starting_point(0), starting_point(1), starting_point(2));

                        //adjust course result within 0-360 range
                        while (starting_point(2) < 0)
                        {
                            starting_point(2) += 360;
                        }

                        while (starting_point(2) >= 360)
                        {
                            starting_point(2) -= 360;
                        }

                        if (starting_point(0) > 0.1 && starting_point(1) > 0.0) {
                            double last_x = last_brg[0];
                            double last_y = last_brg[1];
                            double distance_error = sqrt(pow(last_x - optimize_x, 2) + pow(last_y - optimize_y, 2));
                            double course_error = min(abs(starting_point(2) - last_opt_crs), 360.0 - abs(starting_point(2) - last_opt_crs));
                            if (distance_error < 75.0)
                            {
                                error_within_75m += 1;
                            }
                            
                            if (distance_error < 150.0)
                            {
                                error_within_150m += 1;
                            }
                            
                            if (distance_error < 300.0)
                            {
                                error_within_300m += 1;
                            }

                            if (course_error < 5.0)
                            {
                                error_within_5deg += 1;
                            }

                            if (course_error < 10.0)
                            {
                                error_within_10deg += 1;
                            }

                            if (course_error < 20.0)
                            {
                                error_within_20deg += 1;
                            }

                            total_error_count += 1;
                        }
                    }
                }
            }
            double error_prob_75m = error_within_75m * 100.0 / total_error_count;
            double error_prob_150m = error_within_150m * 100.0 / total_error_count;
            double error_prob_300m = error_within_300m * 100.0 / total_error_count;
            double error_prob_5deg = error_within_5deg * 100.0 / total_error_count;
            double error_prob_10deg = error_within_10deg * 100.0 / total_error_count;
            double error_prob_20deg = error_within_20deg * 100.0 / total_error_count;
            std::cout << endl;
            std::cout << "probability of target course error within 5deg: " << error_prob_5deg << " %" << endl;
            std::cout << "probability of target course error within 10deg: " << error_prob_10deg << " %" << endl;
            std::cout << "probability of target course error within 20deg: " << error_prob_20deg << " %" << endl;
            std::cout << endl;
            std::cout << "probability of target positional error within 75m: " << error_prob_75m << " %" << endl;
            std::cout << "probability of target positional error within 150m: " << error_prob_150m << " %" << endl;
            std::cout << "probability of target positional error within 300m: " << error_prob_300m << " %" << endl;
            std::cout << endl;
        }
        else
        {
            std::cout << "No solution found!" << endl;
        }
    }
}
