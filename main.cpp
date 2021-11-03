#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <cstring>
#include <ctime>
#include "circlefitsolver.h"

using namespace std;

struct Cal_Data
{
    int size;
    double radius;
    double x;
    double y;
    std::vector<POINT> points;
};

void read_data(const char *path, std::vector<Cal_Data> &all_data)
{
    char line[512];
    ifstream myfile(path);
    int count = 0;
    myfile.getline(line, 512);
    myfile.getline(line, 512);
    myfile.getline(line, 512);
    int max_cp_num = 0;
    string temp;
    istringstream iss(line);
    iss >> temp >> max_cp_num;
    myfile.getline(line, 512);
    while (myfile.good())
    {
        for (int i = 0; i < max_cp_num; i++)
        {
            double center_x, center_y, radius;
            myfile.getline(line, 512);
            iss.str(line);
            iss >> temp >> temp >> center_x >> center_y >> temp >> temp >> temp >> radius;
            vector<POINT> p = vector<POINT>();
            Cal_Data data = {0, radius / 2.0, center_x, center_y, p};
            all_data.push_back(data);
        }
        myfile.getline(line, 512);
        for (int i = 0; i < max_cp_num; i++)
        {
            myfile.getline(line, 512);
            myfile.getline(line, 512);
            myfile.getline(line, 512);
            iss.str(line);
            iss >> all_data[i].size;
            for (int j = 0; j < all_data[i].size; j++)
            {
                myfile.getline(line, 512);
                iss.str(line);
                double x1, y1;
                iss >> x1 >> y1;
                POINT point1 = {x1, y1};
                all_data[i].points.push_back(point1);
            }
        }
        break;
    }
}

int main(int argc, char *argv[])
{
    std::vector<Cal_Data> all_data = std::vector<Cal_Data>();
    read_data("../data/L_Cam/D550MM_2.cbt", all_data);
    clock_t start = clock();
    for (int i = 0; i < all_data.size(); i++)
    {
        cout << "                                              " << endl;
        CircleFitSolver cfs = CircleFitSolver();
        cfs.setMaxIter(2000);
        int iter = 0;
        double H11 = 1.0, H12 = 0.0, H13 = 0.0;
        double H21 = 0.0, H22 = 1.0, H23 = 0.0;
        double H31 = 0.0, H32 = 0.0, H33 = 1.0;
        double pr = 0.0;

        cfs.circleFitL1(pr, iter, all_data[i].points, all_data[i].x, all_data[i].y, all_data[i].radius,
                        H11, H12, H13, H21, H22, H23, H31, H32, H33);

        cout << iter << " " << all_data[i].x << " " << all_data[i].y << " " << all_data[i].radius << "   " << pr << endl;
        cout << "                                              " << endl;
    }
    clock_t end = clock();
    cout << "----------------------------------------------" << endl;
    cout << (double)(end - start) / CLOCKS_PER_SEC << endl;

    return 0;
}