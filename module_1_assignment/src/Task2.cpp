#include <iostream>
using namespace std;

template<typename T> 

class Robot 
{
public:

        Robot(T data): _temp(25), _distance(0)
    {
        cout<<"new Robot is defined " << data <<endl;
    }
    float readT()
     {    cout<<"the robot current temperature is  "<< _temp<< " degree C."<<endl;
    return _temp;
    }
     float readD()
     {    cout<<"the robot current distance is  "<< _distance<< " meters."<<endl;
    return _distance;
    }
   
private:
    float _temp;
    float _distance;
};

int main()
{
Robot R1("hello");
R1.readD();
R1.readT();
Robot R2(2);

}