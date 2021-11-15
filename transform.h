#pragma once
#include <vector>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

struct CloudPoints
{
public:
    Point3d Pannel;         //标定版的圆心坐标
    double radius;          //标定板的圆半径
    Point2d Img;            //纠正后影像，圆心坐标
    vector<Point2d> points; //纠正后影像，圆边缘点集合
};

struct CameraParams
{
public:
    double fx, fy, cx, cy;
    double Rw2c[9];
    double tx, ty, tz;
};

class TransFormat
{
public:
    TransFormat() {}
    TransFormat(CameraParams &cp) : m_cp(cp)
    {
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                m_Rc2w[i * 3 + j] = cp.Rw2c[j * 3 + i];
            }
        }
    }
    ~TransFormat(){};
    Point2d w2c(Point3d w_point);
    Point3d c2w(Point2d c_point);
    Point3d c2w_zero();
    double *CalPlaneLineIntersectPoint(double planeVector[3], double planePoint[3], double lineVector[3], double linePoint[3]);

    CameraParams get(); //获取参数
    void set(CameraParams &cp);

private:
    CameraParams m_cp; //w2c
    double m_Rc2w[9];
};