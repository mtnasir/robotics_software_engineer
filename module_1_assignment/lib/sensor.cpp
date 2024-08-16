#include "sensor.hpp"
#include <iostream>
using namespace std;

// template <typename T> 
Robot::Robot() : _temp(25), _distance(0)
{
    cout << "new Robot is defined" <<  endl;
}
float Robot::readT()
{
    cout << "the robot current temperature is  " << _temp << " degree C." << endl;
    return _temp;
}
float Robot::readD()
{
    cout << "the robot current distance is  " << _distance << " meters." << endl;
    return _distance;
}

