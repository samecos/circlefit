#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <cstring>
#include <ctime>
#include "circlefitsolver.h"

using namespace std;
int main(int argc, char* argv[])
{
    char line[256];
    ifstream myfile("C:\\samecos\\circlefit\\data1.txt");
    int count = 0;
    int max = 0;
    double center_x = 0.1, center_y = 1.0;
    double radius = 0.0;
    std::vector<POINT> points = std::vector<POINT>();
    while (myfile.good())
    {
        myfile.getline(line, 256);
        istringstream iss(line);
        if (count == 0)
        {
            iss >> max;
        }
        else if (count == 1)
        {
            iss >> radius;
            radius /= 2.0;
        }
        else if (count == 2)
        {
            iss >> center_x >> center_y;
        }
        else
        {
            double a, b;
            iss >> a >> b;
            POINT point;
            point.real(a);
            point.imag(b);
            points.push_back(point);
        }
        count++;
    }

    CircleFitSolver cfs = CircleFitSolver();
    cfs.setMaxIter(2000);
    int iter = 0;
    double H11 = 1.0, H12 = 0.0, H13 = 0.0;
    double H21 = 0.0, H22 = 1.0, H23 = 0.0;
    double H31 = 0.0, H32 = 0.0, H33 = 1.0;
    double pr = 0.0;
    clock_t start = clock();
    cfs.circleFitL1(pr, iter, points, center_x, center_y, radius,
                    H11, H12, H13, H21, H22, H23, H31, H32, H33);
    clock_t end = clock();
    cout << (double)(end - start) / CLOCKS_PER_SEC << endl;
    cout << iter << " " << center_x << " " << center_y << " " << radius << "   " << pr << endl;

    return 0;
}