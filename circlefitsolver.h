#pragma once
#include <complex>
#include <vector>
#include <iostream>
#include <string>
#include <tuple>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_multimin.h>
#include "transform.h"
using namespace std;

typedef Point2d POINT;

struct Cal_Data
{
public:
    int size;
    double radius;
    double x;
    double y;
    string X;
    string Y;
    string Z;
    std::vector<POINT> points;
};

class CircleFitSolver
{
public:
    CircleFitSolver(CameraParams &cp);
    ~CircleFitSolver();
    void setMaxIter(int iter) { m_max_iter = iter; }
    void setRadius(double radius) { m_radius = radius; }
    /**
     * @brief circleFitL1  拟合圆，拟合判据为数据点到拟合圆的距离绝对值之和最小。
     * @param CloudPoints 输入参数，存储各个数据点。
     * @param CameraParams 相机的参数
     * @return true 表示拟合成功，否则拟合失败。
     */
    bool circleFitL1(double &pr, int &iter, CloudPoints &cloudPoints);

private:
    gsl_multimin_function m_function;
    gsl_multimin_fminimizer *m_fminimizer;
    CameraParams m_cp;         //相机参数
    int m_max_iter;            // 迭代算法的最大迭代次数
    double m_radius;           //圆的半径
    gsl_vector *m_start_point; // 迭代算法的初始值
    gsl_vector *m_step_size;   // 迭代算法的初始步长

    void setStartPoint(double center_x, double center_y);
    void setStepMove();
    static double L1_distance(const gsl_vector *v, void *params);
};
