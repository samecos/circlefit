#include "circlefitsolver.h"
#include <cmath>

using namespace std;

double CircleFitSolver::L1_distance(const gsl_vector *v, void *params)
{
    vector<Point3d> *vect = (vector<Point3d> *)params;
    int N = vect->size();
    TransFormat trs = TransFormat(cameraParams);
    double camera_center_x = gsl_vector_get(v, 0);
    double camera_center_y = gsl_vector_get(v, 1);
    double planeVector[3] = {0, 0, 1};
    double planePoint[3] = {1, 1, 0};
    double *pw = new double[3];
    Point3d center_cal = trs.c2w(cloudPoints.Img);
    Point3d vector_temp = (center_cal - camera_center);
    double lineVector[3] = {vector_temp.x, vector_temp.y, vector_temp.z};
    double linePoint[3] = {center_cal.x, center_cal.y, center_cal.z};
    pw = trs.CalPlaneLineIntersectPoint(planeVector, planePoint, lineVector, linePoint);

    double sum = 0.0;
    for (int i = 0; i < N; i++)
    {

        sum += abs(dist);
    }
    return sum;
}

inline void CircleFitSolver::setStepMove()
{
    gsl_vector_set(m_step_size, 0, 0.001);
    gsl_vector_set(m_step_size, 1, 0.001);
}

inline void CircleFitSolver::setStartPoint(double center_x, double center_y)
{
    gsl_vector_set(m_start_point, 0, center_x);
    gsl_vector_set(m_start_point, 1, center_y);
}

bool CircleFitSolver::circleFitL1(double &pr, int &iter, CloudPoints &cloudPoints, CameraParams &cameraParams)
{
    TransFormat trs = TransFormat(cameraParams);
    vector<Point3d> circle_points;
    //相机像心的坐标
    Point3d camera_center = trs.c2w_zero();
    double planeVector[3] = {0, 0, 1};
    double planePoint[3] = {1, 1, 0};

    //首先将像平面的圆周点变换到世界坐标系
    for (auto cp : cloudPoints.points)
    {
        double *pw = new double[3];
        Point3d pt = trs.c2w(cp);
        Point3d temp = (pt - camera_center);
        //然后计算一下像点和标定板平面的交点
        double lineVector[3] = {temp.x, temp.y, temp.z};
        double linePoint[3] = {pt.x, pt.y, pt.z};

        pw = trs.CalPlaneLineIntersectPoint(planeVector, planePoint, lineVector, linePoint);
        pt.x = pw[0];
        pt.y = pw[1];
        pt.z = pw[2];
        circle_points.push_back(pt);
    }

    //
    m_function.params = (void *)&circle_points;

    setStartPoint(cloudPoints.Img.x, cloudPoints.Img.y);
    setStepMove();

    gsl_multimin_fminimizer_set(m_fminimizer, &m_function, m_start_point, m_step_size);

    iter = 0;
    int status;
    do
    {
        iter++;
        status = gsl_multimin_fminimizer_iterate(m_fminimizer);
        if (status == GSL_ENOPROG) // 表示无法找到更好的解了
        {
            break;
        }
        double size = gsl_multimin_fminimizer_size(m_fminimizer);
        status = gsl_multimin_test_size(size, 0.0001);
        pr = size;
    } while (status == GSL_CONTINUE && iter < m_max_iter);

    gsl_vector *out = gsl_multimin_fminimizer_x(m_fminimizer);

    cloudPoints.Img.x = gsl_vector_get(out, 0);
    cloudPoints.Img.y = gsl_vector_get(out, 1);

    return true;
}

CircleFitSolver::CircleFitSolver()
{
    m_max_iter = 2000; // 默认最大迭代 100 步

    m_function.n = 2;
    m_function.f = L1_distance;

    m_start_point = gsl_vector_alloc(m_function.n);
    m_step_size = gsl_vector_alloc(m_function.n);

    m_fminimizer = gsl_multimin_fminimizer_alloc(gsl_multimin_fminimizer_nmsimplex, m_function.n);
}

CircleFitSolver::~CircleFitSolver()
{
    gsl_vector_free(m_start_point);
    gsl_vector_free(m_step_size);
    gsl_multimin_fminimizer_free(m_fminimizer);
}