#include <iostream>
#include <Eigen/Dense>
#include <chrono>
using namespace std;
using namespace Eigen;

int main(int argc, char **argv)
{
    const int k = 6;
    Matrix<float, k, k> A;
    A.setConstant(10);
    A(0, 0) = 5;
    A(1, 1) = 1;
    A(2, 2) = 3;
    A(3, 3) = 0.7;
    A(4, 4) = 13;
    A(5, 5) = 0.9;
    auto start = chrono::high_resolution_clock::now();
    for (int i = 1; i <= 1000000; i++)
    {
        Matrix<float, k, k> B;
        B = A.inverse();
        // cout << A.inverse() << endl;
    }
    auto end = chrono::high_resolution_clock::now();
    chrono::duration<double> duration = end - start;

    cout << duration.count() << endl;
}