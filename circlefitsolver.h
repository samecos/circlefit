#ifndef CIRCLEFITSOLVER_H
#define CIRCLEFITSOLVER_H

#include <complex>
#include <vector>
#include <iostream>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_multimin.h>

using namespace std;

typedef complex<double> POINT;

struct Cal_Data
{
    int size;
    double radius;
    double x;
    double y;
    std::vector<POINT> points;
};

class CircleFitSolver
{
public:
    CircleFitSolver();
    ~CircleFitSolver();
    void setMaxIter(int iter) { m_max_iter = iter; }

    /**
     * @brief circleFitL1  拟合圆，拟合判据为数据点到拟合圆的距离绝对值之和最小。
     * @param points 输入参数，存储各个数据点。
     * @param center_x radius > 0 时作为迭代算法的初始值。计算完成后返回拟合圆的圆心 X 坐标
     * @param center_y radius > 0 时作为迭代算法的初始值。计算完成后返回拟合圆的圆心 Y 坐标
     * @param radius   radius < 0 时，用最小二乘拟合的结果作为迭代算法的初始值。计算完成后返回拟合圆的半径。
     * @return true 表示拟合成功，否则拟合失败。
     */
    bool circleFitL1(double &pr, int &iter, const vector<POINT> &points, double &center_x, double &center_y, double &radius,
                     double &H11, double &H12, double &H13, double &H21, double &H22, double &H23, double &H31, double &H32, double &H33);
    //bool circleFitL_AllPoints(double &pr, int &iter, vector<Cal_Data> &all_data, double &H11, double &H12, double &H13, double &H21, double &H22, double &H23, double &H31, double &H32, double &H33);

private:
    gsl_multimin_function m_function;
    gsl_multimin_fminimizer *m_fminimizer;

    int m_max_iter; // 迭代算法的最大迭代次数

    gsl_vector *m_start_point; // 迭代算法的初始值
    gsl_vector *m_step_size;   // 迭代算法的初始步长

    void setStartPoint(double center_x, double center_y, double radius, double &H11, double &H12, double &H13, double &H21, double &H22, double &H23, double &H31, double &H32, double &H33);

    static double L1_distance(const gsl_vector *v, void *params);
};

#endif // CIRCLEFITSOLVER_H