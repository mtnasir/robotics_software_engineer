#ifndef SENSOR_HPP
#define SENSOR_HPP
#include <iostream>
using namespace std;

// template <typename T> 
class Robot
{
public:
    Robot();
    float readT();
    float readD();
    template <typename T> 
    void multsensor(T Data);

private:
    float _temp;
    float _distance;
};

template <typename T> 
void Robot::multsensor(T Data)
{

        cout << "sensor data is " << Data << endl;

}
#endif