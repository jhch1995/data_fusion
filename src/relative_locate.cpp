//#include "common/relative_locate/relative_locate.h"
#include "relative_locate.h"


using namespace common;
void RelativeLocate::Initialize(const Posture& posture, const CameraParameter& camera_para)
{
    m_posture = posture;
    m_camera_para = camera_para;
    // 初始化东北天坐标系 (左手系)
    m_axis[0] = VectorR3(1, 0, 0); // right
    m_axis[1] = VectorR3(0, 1, 0); // see
    m_axis[2] = VectorR3(0, 0, 1); // up

    // 产生变换矩阵， 绕东方向旋转alfa(向上为正)
    RotationMapR3 mtrix1 = VrRotate(m_posture.alfa, VectorR3(1, 0, 0));

    // 绕天方向旋转gamma（逆时针为正）
    RotationMapR3 mtrix2 = VrRotate(m_posture.gamma, VectorR3(0, 0, 1));
    // 矩阵左乘
    mtrix1 = mtrix2 * mtrix1;

    // 此时应该获得视线的方向（横滚角是绕视线旋转的角度, 顺时针为正）
    VectorR3 eye_see = mtrix1 * m_axis[1];
    RotationMapR3 mtrix3 = VrRotate(m_posture.beta, eye_see);
    // 获取最终的变换矩阵
    mtrix1 = mtrix3 * mtrix1;
    for (int i = 0; i < 3; ++i)
    {
        m_axis[i] = mtrix1 * m_axis[i];
        m_axis[i].Normalize();
    }

    // init camera parameters
    m_camera_w_length = tan(m_camera_para.stretch_angle_w / 2.0) * 2.0;
    m_camera_h_length = tan(m_camera_para.stretch_angle_h / 2.0) * 2.0;
    m_camera_w_ratio = 1.0 / m_camera_para.pixel_x_number;
    m_camera_h_ratio = 1.0 / m_camera_para.pixel_y_number;
    m_camera_position = VectorR3(m_camera_para.camera_pos_x, m_camera_para.camera_pos_y,
            m_camera_para.camera_pos_z);
    // + add optical center
    m_opt_center_u = m_camera_para.cu / m_camera_para.pixel_x_number;
    m_opt_center_v = m_camera_para.cv / m_camera_para.pixel_y_number;
}

void RelativeLocate::GetCoordXYZ(std::vector<VectorR3>* axis)
{
    axis->clear();
    for (int i = 0; i < 3; ++i)
    {
        axis->push_back(m_axis[i]);
    }
}

void RelativeLocate::GetOrientation(int pixel_x, int pixel_y, VectorR3* orientation)
{
    // 0.5 is center
    double step_y = (m_opt_center_v - 0.5 * m_camera_h_ratio - pixel_y * m_camera_h_ratio) * m_camera_h_length;
    double step_x = (-m_opt_center_u + 0.5 * m_camera_w_ratio + pixel_x * m_camera_w_ratio) * m_camera_w_length;
    VectorR3 y_direction = m_axis[2] * step_y;
    VectorR3 x_direction = m_axis[0] * step_x;
    VectorR3 z_direction = x_direction + y_direction;
    z_direction = m_axis[1] + z_direction;
    z_direction.Normalize();
    *orientation = z_direction;
}

void RelativeLocate::GetGeoCoordinate(int pixel_x, int pixel_y,
        double* geo_x, double* geo_y, double* geo_z)
{
    VectorR3 orientation;
    GetOrientation(pixel_x, pixel_y, &orientation);

    // 平面法向量
    VectorR3 normal(0, 0, 1);
    double elevation = 0;
    VectorR3 point_in_plane(0, 0, elevation); // the third is elevation

    double d = -normal.z * elevation;
    double a = normal.x;
    double b = normal.y;
    double c = normal.z;

    double x = orientation.x;
    double y = orientation.y;
    double z = orientation.z;

    double x0 = m_camera_position.x;
    double y0 = m_camera_position.y;
    double z0 = m_camera_position.z;

    double denominator = a * x + b * y + c * z;
    if (fabs(denominator) < eps) {
        *geo_x = -1;
        *geo_y = -1;
    } else {
        double numerator = -a * x0 - b * y0 - c * z0 - d;
        double t = numerator / denominator;
        *geo_x = t * x + x0;
        *geo_y = t * y + y0;
        *geo_z = t * z + z0;
    }
}

void RelativeLocate::GetPixelCoordinate(double geo_x, double geo_y, double geo_z,
        int* pixel_x, int* pixel_y)
{
    VectorR3 ray(geo_x - m_camera_position.x, geo_y - m_camera_position.y,
            geo_z - m_camera_position.z);
    ray.Normalize();

    double x_res = m_camera_w_length / m_camera_para.pixel_x_number;
    double y_res = m_camera_h_length / m_camera_para.pixel_y_number;
    double angle = SolidAngle(ray, m_axis[1]);
    if (angle >= PIhalves) {
        angle = PI - angle;
        ray = -1 * ray;
//        return; // no intersection
    }
    double length = 1.0 / cos(angle);
    ray = ray * length;
    VectorR3 sub = ray - m_axis[1];
    length = sub.Norm();

    angle = SolidAngle(sub, m_axis[2]);
    double y_dis = length * cos(angle) / y_res;
    angle = SolidAngle(sub, m_axis[0]);
    double x_dis = length * cos(angle) / x_res;
    *pixel_y = static_cast<int>(m_camera_para.cv - y_dis + 0.5);
    *pixel_x = static_cast<int>(m_camera_para.cu + x_dis + 0.5);
}

