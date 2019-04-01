#include "pch.h"
#include <iostream>
#include <math.h>
#include <vector>
#include "iterator.h"

using namespace std;

Iterator::Iterator(const double range, const unsigned int j, vector<double> &bearing, vector<double> &time, vector<double> &travel_distance) :
    _range(1.0), _j(3), _bearing(bearing), _time(time), _travel_distance(travel_distance)
{
    _range = range;
    _j = j;
}

void Iterator::Run()
{
    double optimize_L1_distance;
    double optimize_spd;
    double optimize_crs;
    double min_error = 99999.0;

    double u;
    double v;
    double a;
    double b;

    for (double L1_distance = _range; L1_distance < _range + 500.0; L1_distance += 1.0)
    {
        for (double spd = 1.0; spd <= 5.0; spd += 0.1)
        {
            for (double crs = 0.0; crs <= 359.0; crs += 0.5)
            {
                u = spd * sin(crs * deg_to_rad);
                v = spd * cos(crs * deg_to_rad);
                a = L1_distance * sin(_bearing[0] * deg_to_rad);
                b = L1_distance * cos(_bearing[0] * deg_to_rad);

                double total_error = 0.0;

                for (unsigned int i = 0; i < _j + 1; i++)
                {
                    double x = a + u * _time[i];
                    double y = b + v * _time[i];
                    double line_error = (y - _travel_distance[i]) * sin(_bearing[i] * deg_to_rad) - x * cos(_bearing[i] * deg_to_rad);
                    total_error += pow(line_error, 2);
                }

                if (total_error < min_error)
                {
                    min_error = total_error;
                    optimize_crs = crs;
                    optimize_L1_distance = L1_distance;
                    optimize_spd = spd;
                }

            }
        }
    }

    cout << "== crs: " << optimize_crs << ", error: " << min_error << " ==" << endl;
}