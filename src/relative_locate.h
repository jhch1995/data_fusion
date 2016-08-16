/// @file relative_locate.h
/// @brief
/// @author Devin (devin@minieye.cc)
/// @date 2015-06-19
/// Copyright (C) 2015 - MiniEye INC.

#ifndef  RELATIVE_LOCATE_H
#define  RELATIVE_LOCATE_H

#if defined(_MSC_VER) && (_MSC_VER >= 1600)
#pragma execution_character_set("utf-8")
#endif

#include <vector>
#include "linear_r3.h"
struct Posture
{
    Posture& operator=(const Posture& lhs)
    {
        alfa = lhs.alfa;
        beta = lhs.beta;
        gamma = lhs.gamma;
        return *this;
    }

    double alfa; /// alfa - 俯仰角
    double beta; /// beta - 横滚角
    double gamma; /// gamma - 偏航角
};

struct CameraParameter
{
    CameraParameter()
    {
        camera_pos_x = 0; //
        camera_pos_y = 0;
        camera_pos_z = 0;
    }

    CameraParameter& operator=(const CameraParameter& lhs)
    {
        stretch_angle_w = lhs.stretch_angle_w;
        stretch_angle_h = lhs.stretch_angle_h;
        pixel_x_number = lhs.pixel_x_number;
        pixel_y_number = lhs.pixel_y_number;
        camera_pos_x = lhs.camera_pos_x;
        camera_pos_y = lhs.camera_pos_y;
        camera_pos_z = lhs.camera_pos_z;
        cu = lhs.cu;
        cv = lhs.cv;
        return *this;
    }

    double stretch_angle_w; /// stretch_angle_w 横向张角(弧度)
    double stretch_angle_h; /// stretch_angle_h 纵向张角(弧度)
    int pixel_x_number; /// pixel_width 横向像元数
    int pixel_y_number; /// pixel_height 纵向像元数
    double camera_pos_x;
    double camera_pos_y;
    double camera_pos_z;

    double cu; // 光心坐标(横向)
    double cv; // 光心坐标(纵向)
};

class RelativeLocate
{
public:

    /// @brief construct RelativeLocate obj
    RelativeLocate() {}

    ~RelativeLocate() {}

    void Initialize(const Posture& posture, const CameraParameter& cammera_para);


    /// @brief 获得探测器在不同的姿态下的视线方向和两个轴线的方向
    ///
    /// @param axis
    /// axis[0]是宽度方向
    /// axis[1]是视线方向
    /// axis[2]是高度方向
    void GetCoordXYZ(std::vector<VectorR3>* axis);


    /// @brief 计算每个像素元到视点的方向向量
    ///
    /// @param pixel_x 待求像素位置坐标X
    /// @param pixel_y 待求像素位置坐标Y
    /// @param orientation 方向向量
    void GetOrientation(int pixel_x, int pixel_y, VectorR3* orientation);


    /// @brief 获取给定的像素位置对应于空间坐标系的坐标
    ///
    /// @param position
    /// @param pixel_x
    /// @param pixel_y
    /// @param geo_x
    /// @param geo_y
    /// @param geo_z
    void GetGeoCoordinate(int pixel_x, int pixel_y,
            double* geo_x, double* geo_y, double* geo_z);

    void GetPixelCoordinate(double geo_x, double geo_y, double geo_z,
            int* pixel_x, int* pixel_y);

private:
    Posture m_posture;
    CameraParameter m_camera_para;
    VectorR3 m_axis[3];
    double m_camera_w_length; /// 宽度方向像元总长度(assume 焦距是1个单位)
    double m_camera_h_length; /// 高度方向像元总长度(assume 焦距是1个单位)
    double m_camera_w_ratio;
    double m_camera_h_ratio;
    VectorR3 m_camera_position;
    double m_opt_center_u;
    double m_opt_center_v;
};


#endif  // RELATIVE_LOCATE_H
