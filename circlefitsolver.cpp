#include "circlefitsolver.h"
#include <cmath>

using namespace std;

double CircleFitSolver::L1_distance(const gsl_vector *v, void *params)
{
    vector<POINT> *vect = (vector<POINT> *)params;
    int N = vect->size();

    double a, b, r;
    double H11, H12, H13, H21, H22, H23, H31, H32, H33;
    a = gsl_vector_get(v, 0);
    b = gsl_vector_get(v, 1);
    r = gsl_vector_get(v, 2);
    H11 = gsl_vector_get(v, 3);
    H12 = gsl_vector_get(v, 4);
    H13 = gsl_vector_get(v, 5);
    H21 = gsl_vector_get(v, 6);
    H22 = gsl_vector_get(v, 7);
    H23 = gsl_vector_get(v, 8);
    H31 = gsl_vector_get(v, 9);
    H32 = gsl_vector_get(v, 10);
    H33 = gsl_vector_get(v, 11);
    double cal_a = 0.0, cal_b = 0.0;
    cal_a = (H11 * a + H12 * b + H13) / (H31 * a + H32 * b + H33);
    cal_b = (H21 * a + H22 * b + H23) / (H31 * a + H32 * b + H33);
    double sum = 0;
    for (int i = 0; i < N; i++)
    {
        const POINT p = vect->at(i);
        double x = p.real();
        double y = p.imag();
        const double cal_x = (H11 * x + H12 * y + H13) / (H31 * x + H32 * y + H33);
        const double cal_y = (H21 * x + H22 * y + H23) / (H31 * x + H32 * y + H33);
        double xi = cal_x - cal_a;
        double yi = cal_y - cal_b;
        double dist = sqrt(xi * xi + yi * yi) - r;
        sum += fabs(dist);
    }
    return sum;
}

inline void CircleFitSolver::setStartPoint(double center_x, double center_y, double radius, double &H11, double &H12, double &H13, double &H21, double &H22, double &H23, double &H31, double &H32, double &H33)
{
    gsl_vector_set(m_start_point, 0, center_x);
    gsl_vector_set(m_start_point, 1, center_y);
    gsl_vector_set(m_start_point, 2, radius);
    gsl_vector_set(m_start_point, 3, H11);
    gsl_vector_set(m_start_point, 4, H12);
    gsl_vector_set(m_start_point, 5, H13);
    gsl_vector_set(m_start_point, 6, H21);
    gsl_vector_set(m_start_point, 7, H22);
    gsl_vector_set(m_start_point, 8, H23);
    gsl_vector_set(m_start_point, 9, H31);
    gsl_vector_set(m_start_point, 10, H32);
    gsl_vector_set(m_start_point, 11, H33);
}

bool CircleFitSolver::circleFitL1(double &pr, int &iter, const vector<POINT> &points, double &center_x, double &center_y, double &radius, double &H11, double &H12, double &H13, double &H21, double &H22, double &H23, double &H31, double &H32, double &H33)
{
    m_function.params = (void *)&points;

    setStartPoint(center_x, center_y, radius, H11, H12, H13, H21, H22, H23, H31, H32, H33);
    /* 经验值，初始步长设置为半径的十分之一 */
    gsl_vector_set(m_step_size, 0, radius / 10.0);
    gsl_vector_set(m_step_size, 1, radius / 10.0);
    gsl_vector_set(m_step_size, 2, radius / 10.0);
    gsl_vector_set(m_step_size, 3, 0.01);
    gsl_vector_set(m_step_size, 4, 0.01);
    gsl_vector_set(m_step_size, 5, 0.01);
    gsl_vector_set(m_step_size, 6, 0.01);
    gsl_vector_set(m_step_size, 7, 0.01);
    gsl_vector_set(m_step_size, 8, 0.01);
    gsl_vector_set(m_step_size, 9, 0.01);
    gsl_vector_set(m_step_size, 10, 0.01);
    gsl_vector_set(m_step_size, 11, 0.01);

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

    center_x = gsl_vector_get(out, 0);
    center_y = gsl_vector_get(out, 1);
    radius = gsl_vector_get(out, 2);
    H11 = gsl_vector_get(out, 3);
    H12 = gsl_vector_get(out, 4);
    H13 = gsl_vector_get(out, 5);
    H21 = gsl_vector_get(out, 6);
    H22 = gsl_vector_get(out, 7);
    H23 = gsl_vector_get(out, 8);
    H31 = gsl_vector_get(out, 9);
    H32 = gsl_vector_get(out, 10);
    H33 = gsl_vector_get(out, 11);
    std::cout << H11 << " " << H12 << " " << H13 << endl;
    std::cout << H21 << " " << H22 << " " << H23 << endl;
    std::cout << H31 << " " << H32 << " " << H33 << endl;
    return true;
}

CircleFitSolver::CircleFitSolver()
{
    m_max_iter = 100; // 默认最大迭代 100 步

    m_function.n = 12;
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