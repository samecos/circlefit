#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <cstring>
#include <filesystem>
#include <ctime>
#include "circlefitsolver.h"

using namespace std;
namespace fs = std::filesystem;
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
            iss.clear();
            iss.str(line);
            string X, Y, Z;
            iss >> temp >> temp >> center_x >> center_y >> X >> Y >> Z >> radius;
            vector<POINT> p = vector<POINT>();
            Cal_Data data = {0, radius / 2.0, center_x, center_y, X, Y, Z, p};
            all_data.push_back(data);
        }
        myfile.getline(line, 512);
        for (int i = 0; i < max_cp_num; i++)
        {
            myfile.getline(line, 512);
            myfile.getline(line, 512);
            myfile.getline(line, 512);
            iss.clear();
            iss.str(line);
            iss >> all_data[i].size;
            for (int j = 0; j < all_data[i].size; j++)
            {
                myfile.getline(line, 512);
                iss.clear();
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
    std::string paths = "./data/";
    for (const auto &entry : fs::directory_iterator(paths))
    {
        std::vector<Cal_Data> all_data = std::vector<Cal_Data>();
        read_data(entry.path().string().c_str(), all_data);
        clock_t start = clock();
        for (int i = 0; i < all_data.size(); i++)
        {
            //cout << "                                              " << endl;
            CircleFitSolver cfs = CircleFitSolver();
            cfs.setMaxIter(2000);
            int iter = 0;
            double H11 = 1.0, H12 = 0.0, H13 = 0.0;
            double H21 = 0.0, H22 = 1.0, H23 = 0.0;
            double H31 = 0.0, H32 = 0.0, H33 = 1.0;
            double pr = 0.0;

            cfs.circleFitL1(pr, iter, all_data[i].points, all_data[i].x, all_data[i].y, all_data[i].radius,
                            H11, H12, H13, H21, H22, H23, H31, H32, H33);

            cout << all_data[i].X << "\t" << all_data[i].Y << "\t" << all_data[i].Z << "\t"
                 << all_data[i].x << "\t" << all_data[i].y << "\t"
                 << iter << "\t" << all_data[i].radius << "\t" << pr << endl;
            //cout << "                                              " << endl;
        }
        clock_t end = clock();
        cout << "----------------------------------------------" << endl;
        cout << (double)(end - start) / CLOCKS_PER_SEC << endl;
        cout << "----------------------------------------------" << endl;
    }
    return 0;
}