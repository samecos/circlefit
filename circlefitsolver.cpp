#include "circlefitsolver.h"
#include <cmath>

using namespace std;

double CircleFitSolver::L1_distance(const gsl_vector *v, void *params)
{
    std::tuple<CloudPoints, vector<Point3d>, CameraParams> *vetc = (std::tuple<CloudPoints, vector<Point3d>, CameraParams> *)params;
    auto [cps, circle_points, cp] = *vetc;
    int N = cps.points.size();
    TransFormat trs = TransFormat(cp);

    //相机像心的坐标
    Point3d camera_center = trs.c2w_zero();
    Point3d planeVector = {0, 0, 1};
    Point3d planePoint = {1, 1, 0};
    double camera_center_x = gsl_vector_get(v, 0);
    double camera_center_y = gsl_vector_get(v, 1);
    Point2d cc = {camera_center_x, camera_center_y};
    Point3d center_cal = trs.c2w(cc);

    Point3d lineVector = (center_cal - camera_center);
    Point3d point_circle_center = trs.CalPlaneLineIntersectPoint(planeVector, planePoint, lineVector, center_cal);

    double sum = 0.0;
    for (int i = 0; i < N; i++)
    {
        auto dist = circle_points[i] - point_circle_center;
        double pre = sqrt(dist.x * dist.x + dist.y * dist.y + dist.z * dist.z) - cps.radius;
        sum += abs(pre);
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

bool CircleFitSolver::circleFitL1(double &pr, int &iter, CloudPoints &cloudPoints)
{
    TransFormat trs = TransFormat(m_cp);

    vector<Point3d> circle_points;
    //相机像心的坐标
    Point3d camera_center = trs.c2w_zero();
    Point3d planeVector = {0, 0, 1};
    Point3d planePoint = {1, 1, 0};

    //首先将像平面的圆周点变换到世界坐标系
    for (auto cpt : cloudPoints.points)
    {
        Point3d linePoint = trs.c2w(cpt);
        Point3d lineVector = (linePoint - camera_center);
        circle_points.push_back(trs.CalPlaneLineIntersectPoint(planeVector, planePoint, lineVector, linePoint));
    }

    //
    m_function.params = (void *)&std::make_tuple(cloudPoints, circle_points, m_cp);

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

CircleFitSolver::CircleFitSolver(CameraParams &cp) : m_cp(cp)
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