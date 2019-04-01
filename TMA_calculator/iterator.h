#pragma once
#include "pch.h"
#include <iostream>
#include <math.h>
#include <vector>

using namespace std;

class Iterator
{
public:
    Iterator(const double range, const unsigned int j, vector<double> &bearing, vector<double> &time, vector<double> &travel_distance);
    void Run();

protected:
    double _range;
    unsigned int _j;
    vector<double> _bearing;
    vector<double> _time;
    vector<double> _travel_distance;
};