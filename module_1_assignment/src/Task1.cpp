#include <iostream>
using namespace std;

class Robot 
{
public:
    Robot(float w,int n):_speed(0),_weight(w),_num_of_sensors(n)
    {
        cout<<"new Robot is defined: weight= "<<_weight<<". number of sensors = "<<_num_of_sensors<<endl;
    }
    void moveForward(float s)
     {   _speed=s;
            cout<<"the robot is moving forward in "<< _speed<< " speed."<<endl;

    }
    void moveBackward(float s)
     {   _speed=s;
            cout<<"the robot is moving backward in "<< _speed<< " speed."<<endl;
    }
    void stop()
     {   _speed=0;
            cout<<"the robot is stopped"<<endl;
    }
   
private:
    string _name;
    float _speed;
    float _weight;
    int _num_of_sensors;
};

int main()
{

Robot R1(2.9f,3);
R1.moveForward(200.0);
R1.moveBackward(100);
R1.stop();
Robot R2(3,3);

}