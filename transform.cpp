#include "transform.h"

CameraParams TransFormat::get()
{
    return m_cp;
}

void TransFormat::set(CameraParams &cp)
{
    m_cp = cp;
}

Point2d TransFormat::w2c(Point3d w_point)
{
    Point2d c_point;
    double x1, y1, z1;
    x1 = m_cp.Rw2c[0] * w_point.x + m_cp.Rw2c[1] * w_point.y + m_cp.Rw2c[2] * w_point.z + m_cp.tx;
    y1 = m_cp.Rw2c[3] * w_point.x + m_cp.Rw2c[4] * w_point.y + m_cp.Rw2c[5] * w_point.z + m_cp.ty;
    z1 = m_cp.Rw2c[6] * w_point.x + m_cp.Rw2c[7] * w_point.y + m_cp.Rw2c[8] * w_point.z + m_cp.tz;
    c_point.x = m_cp.fx * x1 / z1 + m_cp.cx;
    c_point.y = m_cp.fy * y1 / z1 + m_cp.cy;
    return c_point;
}

Point3d TransFormat::c2w(Point2d c_point)
{
    Point3d w_point;

    double x1, y1, z1;
    x1 = (c_point.x - m_cp.cx) / m_cp.fx - m_cp.tx;
    y1 = (c_point.y - m_cp.cy) / m_cp.fy - m_cp.ty;
    z1 = 1 - m_cp.tz;

    w_point.x = m_Rc2w[0] * x1 + m_Rc2w[1] * y1 + m_Rc2w[2] * z1;
    w_point.y = m_Rc2w[3] * x1 + m_Rc2w[4] * y1 + m_Rc2w[5] * z1;
    w_point.z = m_Rc2w[6] * x1 + m_Rc2w[7] * y1 + m_Rc2w[8] * z1;

    return w_point;
}

Point3d TransFormat::c2w_zero()
{
    Point3d w_point;
    double x1, y1, z1;
    x1 = -m_cp.tx;
    y1 = -m_cp.ty;
    z1 = -m_cp.tz;

    w_point.x = m_Rc2w[0] * x1 + m_Rc2w[1] * y1 + m_Rc2w[2] * z1;
    w_point.y = m_Rc2w[3] * x1 + m_Rc2w[4] * y1 + m_Rc2w[5] * z1;
    w_point.z = m_Rc2w[6] * x1 + m_Rc2w[7] * y1 + m_Rc2w[8] * z1;

    return w_point;
}

/// <summary>
/// 求一条直线与平面的交点
/// </summary>
/// <param name="planeVector">平面的法线向量，长度为3</param>
/// <param name="planePoint">平面经过的一点坐标，长度为3</param>
/// <param name="lineVector">直线的方向向量，长度为3</param>
/// <param name="linePoint">直线经过的一点坐标，长度为3</param>
/// <returns>返回交点坐标，长度为3</returns>
Point3d TransFormat::CalPlaneLineIntersectPoint(Point3d planeVector, Point3d planePoint, Point3d lineVector, Point3d linePoint)
{
    Point3d returnResult = Point3d(0.0, 0.0, 0.0);
    double vp1, vp2, vp3, n1, n2, n3, v1, v2, v3, m1, m2, m3, t, vpt;
    vp1 = planeVector.x;
    vp2 = planeVector.y;
    vp3 = planeVector.z;
    n1 = planePoint.x;
    n2 = planePoint.y;
    n3 = planePoint.z;
    v1 = lineVector.x;
    v2 = lineVector.y;
    v3 = lineVector.z;
    m1 = linePoint.x;
    m2 = linePoint.y;
    m3 = linePoint.z;
    vpt = v1 * vp1 + v2 * vp2 + v3 * vp3;
    //首先判断直线是否与平面平行
    if (vpt > 1e-6 || vpt < -1e-6)
    {
        t = ((n1 - m1) * vp1 + (n2 - m2) * vp2 + (n3 - m3) * vp3) / vpt;
        returnResult.x = m1 + v1 * t;
        returnResult.y = m2 + v2 * t;
        returnResult.z = m3 + v3 * t;
    }
    return returnResult;
}