#include <iostream>
#include "bird_perspective_mapping.h"

/*
 We are assuming the world coordinate frame center is at the camera,
 the ground plane is at height -h, the X-axis is going right,
 the Y-axis is going forward, the Z-axis is going up. The
 camera is looking forward with optical axis in direction of
 Y-axis, with possible pitch angle (above or below the Y-axis)
 and yaw angle (left or right).
 The camera coordinates have the same center as the world, but the Xc-axis goes right,
 the  Yc-axis goes down, and the Zc-axis (optical cxis) goes forward. The
 uv-plane of the image is such that u is horizontal going right, v is
 vertical going down.
 The image coordinates uv are such that the pixels are at half coordinates
 i.e. first pixel is (.5,.5) ...etc where the top-left point is (0,0) i.e.
 the tip of the first pixel is (0,0)
*/

BirdPerspectiveMapping::BirdPerspectiveMapping(const CameraPara& camera_para)
{
    Initialize(camera_para);
}

void BirdPerspectiveMapping::Initialize(const CameraPara& camera_para)
{
    m_camera_para = camera_para;
    // world to camera
    InitYawTransformMatrix();
    InitPitchTransformMatrix();
    // K Matrix
    InitShiftMatrix();
    // get projective matrix P=K*R
    GetGround2ImageMatrix();
    // image to ground matirx
    GetImage2GroundMatrix();
}

BirdPerspectiveMapping::~BirdPerspectiveMapping()
{
}

// yaw to matrix
void BirdPerspectiveMapping::InitYawTransformMatrix()
{
    // transform from world to camera coordinates
    //
    // rotation matrix for yaw
    m_yaw_matrix = cv::Mat::zeros(3, 3, CV_32FC1);

    m_yaw_matrix.at<float>(0, 0) = cos(m_camera_para.yaw);
    m_yaw_matrix.at<float>(0, 1) = -sin(m_camera_para.yaw);
    m_yaw_matrix.at<float>(0, 2) = 0;

    m_yaw_matrix.at<float>(1, 0) = sin(m_camera_para.yaw);
    m_yaw_matrix.at<float>(1, 1) = cos(m_camera_para.yaw);
    m_yaw_matrix.at<float>(1, 2) = 0;

    m_yaw_matrix.at<float>(2, 0) = 0;
    m_yaw_matrix.at<float>(2, 1) = 0;
    m_yaw_matrix.at<float>(2, 2) = 1;
//    std::cerr << "yaw_matrix: " << m_yaw_matrix << std::endl;
}

// pitch to matrix
void BirdPerspectiveMapping::InitPitchTransformMatrix()
{
    // rotation matrix for pitch
    m_pitch_matrix = cv::Mat::zeros(3, 3, CV_32FC1);

    m_pitch_matrix.at<float>(0, 0) = 1;
    m_pitch_matrix.at<float>(0, 1) = 0;
    m_pitch_matrix.at<float>(0, 2) = 0;

    m_pitch_matrix.at<float>(1, 0) = 0;
    m_pitch_matrix.at<float>(1, 1) = -sin(m_camera_para.pitch);
    m_pitch_matrix.at<float>(1, 2) = -cos(m_camera_para.pitch);

    m_pitch_matrix.at<float>(2, 0) = 0;
    m_pitch_matrix.at<float>(2, 1) = cos(m_camera_para.pitch);
    m_pitch_matrix.at<float>(2, 2) = -sin(m_camera_para.pitch);
//    std::cerr << "pitch_matrix: " << m_pitch_matrix << std::endl;
}

// �����趨��ǰpitch
void BirdPerspectiveMapping::SetPitchTransformMatrix(double pitch_new)
{
    // rotation matrix for pitch
    m_pitch_matrix = cv::Mat::zeros(3, 3, CV_32FC1);

    m_pitch_matrix.at<float>(0, 0) = 1;
    m_pitch_matrix.at<float>(0, 1) = 0;
    m_pitch_matrix.at<float>(0, 2) = 0;

    m_pitch_matrix.at<float>(1, 0) = 0;
    m_pitch_matrix.at<float>(1, 1) = -sin(pitch_new);
    m_pitch_matrix.at<float>(1, 2) = -cos(pitch_new);

    m_pitch_matrix.at<float>(2, 0) = 0;
    m_pitch_matrix.at<float>(2, 1) = cos(pitch_new);
    m_pitch_matrix.at<float>(2, 2) = -sin(pitch_new);

}

// K matrix (Intrinsic Matrix )
void BirdPerspectiveMapping::InitShiftMatrix()
{
    // transformation from (xc, yc) in camera coordinates
    // to (u,v) in image frame
    //
    // matrix to shift optical center and focal length
    m_shift_matrix = cv::Mat(3, 3, CV_32FC1);
    m_shift_matrix.at<float>(0, 0) = m_camera_para.fu;
    m_shift_matrix.at<float>(0, 1) = 0;
    m_shift_matrix.at<float>(0, 2) = m_camera_para.cu;

    m_shift_matrix.at<float>(1, 0) = 0;
    m_shift_matrix.at<float>(1, 1) = m_camera_para.fv;
    m_shift_matrix.at<float>(1, 2) = m_camera_para.cv;

    m_shift_matrix.at<float>(2, 0) = 0;
    m_shift_matrix.at<float>(2, 1) = 0;
    m_shift_matrix.at<float>(2, 2) = 1;
//    std::cerr << "shift_matrix: " << m_shift_matrix << std::endl;
}

void BirdPerspectiveMapping::GetGround2ImageMatrix()
{
    // can consider roll matrix
    m_ground2image_matrix = m_pitch_matrix * m_yaw_matrix;
    m_ground2image_matrix = m_shift_matrix * m_ground2image_matrix;
//    std::cerr << "ground2image  matrix: " << m_ground2image_matrix << std::endl;
}


// calc x_limits, y_limits
void BirdPerspectiveMapping::GetXYLimitsFromUV(IPMPara* ipm_para)
{
    // get size of input image
    float u = m_camera_para.image_width;
    float v = m_camera_para.image_height;

    // get the vanishing point
    cv::Point2f vp;
    GetVanishingPoint(&vp);
    vp.y = std::max(float(0), vp.y);

    // get extent of the image in the xfyf plane
    float eps = ipm_para->vp_portion * v;
    ipm_para->u_limits[0] = std::max(double(0), ipm_para->u_limits[0]);
    ipm_para->u_limits[1] = std::min(double(u-1), ipm_para->u_limits[1]);
    ipm_para->v_limits[0] = std::max(double(vp.y + eps), ipm_para->v_limits[0]);
    ipm_para->v_limits[1] = std::min(double(v-1), ipm_para->v_limits[1]);

    cv::Mat uv_limits_pts = cv::Mat::zeros(2, 4, CV_32FC1);
    uv_limits_pts.at<float>(0, 0) = vp.x;
    uv_limits_pts.at<float>(0, 1) = ipm_para->u_limits[1];
    uv_limits_pts.at<float>(0, 2) = ipm_para->u_limits[0];
    uv_limits_pts.at<float>(0, 3) = vp.x;

    uv_limits_pts.at<float>(1, 0) = ipm_para->v_limits[0];
    uv_limits_pts.at<float>(1, 1) = ipm_para->v_limits[0];
    uv_limits_pts.at<float>(1, 2) = ipm_para->v_limits[0];
    uv_limits_pts.at<float>(1, 3) = ipm_para->v_limits[1];

    // get these points on the ground plane
    cv::Mat xy_limits = cv::Mat::zeros(2, 4, CV_32FC1);
    TransformImage2Ground(uv_limits_pts, &xy_limits);

    cv::Mat row1 = xy_limits.row(0);
    cv::Mat row2 = xy_limits.row(1);
    cv::minMaxLoc(row1, &(ipm_para->x_limits[0]), &(ipm_para->x_limits[1]));
    cv::minMaxLoc(row2, &(ipm_para->y_limits[0]), &(ipm_para->y_limits[1]));
    ipm_para->x_scale = (ipm_para->x_limits[1] - ipm_para->x_limits[0]) / ipm_para->width;
    ipm_para->y_scale = (ipm_para->y_limits[1] - ipm_para->y_limits[0]) / ipm_para->height;
}

// from word coordinate comput ipm image width and height
void BirdPerspectiveMapping::GetUVLimitsFromXY(IPMPara* ipm_para)
{
    // iamge width height by scale(one pixel represent real word scale)
    ipm_para->width = static_cast<int>((ipm_para->x_limits[1] - ipm_para->x_limits[0]) / ipm_para->x_scale);
    ipm_para->height = static_cast<int>((ipm_para->y_limits[1] - ipm_para->y_limits[0]) / ipm_para->y_scale);
    // calc uv limits  2*4*1
    cv::Mat xy_limits_pts = cv::Mat::zeros(2, 4, CV_32FC1);
    xy_limits_pts.at<float>(0, 0) = ipm_para->x_limits[0];
    xy_limits_pts.at<float>(0, 1) = ipm_para->x_limits[1];
    xy_limits_pts.at<float>(0, 2) = ipm_para->x_limits[0];
    xy_limits_pts.at<float>(0, 3) = ipm_para->x_limits[1];

    xy_limits_pts.at<float>(1, 0) = ipm_para->y_limits[0];
    xy_limits_pts.at<float>(1, 1) = ipm_para->y_limits[0];
    xy_limits_pts.at<float>(1, 2) = ipm_para->y_limits[1];
    xy_limits_pts.at<float>(1, 3) = ipm_para->y_limits[1];

    cv::Mat uv_limits = cv::Mat::zeros(2, 4, CV_32FC1);
    TransformGround2Image(xy_limits_pts, &uv_limits);

    cv::Mat row1 = uv_limits.row(0);
    cv::Mat row2 = uv_limits.row(1);
    // returm max min pointer
    cv::minMaxLoc(row1, &(ipm_para->u_limits[0]), &(ipm_para->u_limits[1]));
    cv::minMaxLoc(row2, &(ipm_para->v_limits[0]), &(ipm_para->v_limits[1]));

    // confined  u v
    float u = m_camera_para.image_width;
    float v = m_camera_para.image_height;

    // get the vanishing point
    cv::Point2f vp;
    GetVanishingPoint(&vp);
    vp.y = std::max(float(0), vp.y);

    // 20211126...........
    // get extent of the image in the xfyf plane
    float eps = ipm_para->vp_portion * v;
    ipm_para->u_limits[0] = std::max(double(0), ipm_para->u_limits[0]);
    ipm_para->u_limits[1] = std::min(double(u-1), ipm_para->u_limits[1]);
    ipm_para->v_limits[0] = std::max(double(vp.y + eps), ipm_para->v_limits[0]);
    ipm_para->v_limits[1] = std::min(double(v-1), ipm_para->v_limits[1]);

    int i, j;
    float x, y;
    cv::Mat xy_grid = cv::Mat::zeros(2, ipm_para->width * ipm_para->height, CV_32FC1);
    for (i = 0, y = ipm_para->y_limits[1] - 0.5 * ipm_para->y_scale;
            i < ipm_para->height; ++i, y -= ipm_para->y_scale) {
            int base = i * ipm_para->width;
        for (j = 0, x = ipm_para->x_limits[0] + 0.5 * ipm_para->x_scale;
                j < ipm_para->width; ++j, x+= ipm_para->x_scale) {
            int offset = base + j;
            xy_grid.at<float>(0, offset) = x;
            xy_grid.at<float>(1, offset) = y;
        }
    }
    ipm_para->uv_grid = cv::Mat::zeros(2, ipm_para->width * ipm_para->height, CV_32FC1);
    TransformGround2Image(xy_grid, &ipm_para->uv_grid);
}

void BirdPerspectiveMapping::GetImage2GroundMatrix()
{
    float c1 = cos(m_camera_para.pitch);
    float s1 = sin(m_camera_para.pitch);
    float c2 = cos(m_camera_para.yaw);
    float s2 = sin(m_camera_para.yaw);
    // camera install height
    float h = m_camera_para.height; // unit: m

    m_image2ground_matrix = cv::Mat::zeros(4, 3, CV_32FC1);

    // homogeneous matrix seems correct
    // https://github.com/maitham/LaneDetectionAversion/blob/b05f8a3465745192a24096d9f63b810a177af521/FinalCodeLaneAversion/createFourPointsForIPM.cpp
    m_image2ground_matrix.at<float>(0, 0) = -h * c2 / m_camera_para.fu;
    m_image2ground_matrix.at<float>(0, 1) = h * s1 * s2 / m_camera_para.fv;
    m_image2ground_matrix.at<float>(0, 2) = (h * c2 * m_camera_para.cu / m_camera_para.fu) -
        (h * s1 * s2 * m_camera_para.cv / m_camera_para.fv) - h * c1 * s2;

    m_image2ground_matrix.at<float>(1, 0) = h * s2 / m_camera_para.fu;
    m_image2ground_matrix.at<float>(1, 1) = h * s1 * c2 / m_camera_para.fv;
    m_image2ground_matrix.at<float>(1, 2) = (-h * s2 * m_camera_para.cu / m_camera_para.fu) -
        (h * s1 * c2 * m_camera_para.cv / m_camera_para.fv) - h * c1 * c2;

    m_image2ground_matrix.at<float>(2, 0) = 0;
    m_image2ground_matrix.at<float>(2, 1) = h * c1 / m_camera_para.fv;
    m_image2ground_matrix.at<float>(2, 2) = (-h * c1 * m_camera_para.cv / m_camera_para.fv) + h * s1;

    m_image2ground_matrix.at<float>(3, 0) = 0;
    m_image2ground_matrix.at<float>(3, 1) = -c1 / m_camera_para.fv;
    m_image2ground_matrix.at<float>(3, 2) = (c1 * m_camera_para.cv / m_camera_para.fv) - s1;
}

void BirdPerspectiveMapping::TransformImage2Ground(const cv::Mat& in_points, cv::Mat* out_points)
{
    // add two rules to the input points
    cv::Mat in_points4 = cv::Mat::zeros(in_points.rows + 2, in_points.cols, in_points.type());

    // copy inpoints to first two rows
    cv::Mat in_points2 = in_points4.rowRange(0, 2);
    cv::Mat in_points3 = in_points4.rowRange(0, 3);
    cv::Mat in_pointsr3 = in_points4.row(2);
    cv::Mat in_pointsr4 = in_points4.row(3);
    in_pointsr3.setTo(cv::Scalar(1));
    in_points.copyTo(in_points2);
//    std::cerr << "image2ground matrix " << m_image2ground_matrix << std::endl;
//    std::cerr << "in_points3 " << in_points3 << std::endl;

    in_points4 = m_image2ground_matrix * in_points3;
//    std::cerr << "-------------------------------" << std::endl << std::endl;
//    std::cerr << "in_points4: " << in_points4 << std::endl;
//    std::cerr << "-------------------------------" << std::endl << std::endl;
    for(int i = 0; i < in_points.cols; ++i) {
        float div = in_pointsr4.at<float>(0, i);
        in_points4.at<float>(0, i) /= div;
        in_points4.at<float>(1, i) /= div;
    }
//    std::cerr << "in_points4: " << in_points4 << std::endl;
    in_points2.copyTo(*out_points);
}

void BirdPerspectiveMapping::GetVanishingPoint(cv::Point2f* vpt)
{
    // get the vp in world coordinates
    double vpp[] = {sin((double)(m_camera_para.yaw)) / cos((double)(m_camera_para.pitch)),
                  cos((double)(m_camera_para.yaw)) / cos((double)(m_camera_para.pitch)), 0};

    cv::Mat vp = cv::Mat(3, 1, CV_32FC1, vpp);
    vp = m_ground2image_matrix *vp;
    vpt->x = vp.at<float>(0, 0);
    vpt->y = vp.at<float>(1, 0);
}

// input 2*4  output 2*4
void BirdPerspectiveMapping::TransformGround2Image(const cv::Mat& in_points, cv::Mat* out_points)
{
    cv::Mat in_points3 = cv::Mat::zeros(in_points.rows + 1, in_points.cols, in_points.type());
    cv::Mat in_points2 = in_points3.rowRange(0, 2);
    cv::Mat in_pointsr3 = in_points3.row(2);
    in_pointsr3.setTo(cv::Scalar(-m_camera_para.height));

    in_points.copyTo(in_points2);
    in_points3 = m_ground2image_matrix * in_points3;

    for (int i = 0; i < in_points.cols;  ++i) {
        float div = in_pointsr3.at<float>(0, i);
        in_points3.at<float>(0, i) /= div;
        in_points3.at<float>(1, i) /= div;
    }
    in_points2.copyTo(*out_points);
}
