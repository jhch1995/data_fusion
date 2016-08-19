
#include "opencv2/opencv.hpp"
//#include "common/relative_locate/relative_locate.h"
#include "relative_locate.h"


int main(int argc, char *argv[])
{
    //
    RelativeLocate rel_locate;

    Posture posture;
    // 俯仰角
    posture.alfa = (1) * (0.2) * CV_PI / 180;
    // 横滚角
    posture.beta = 0.0 * CV_PI / 180;
    // 偏航角
    posture.gamma = (3.5)* CV_PI / 180;

    CameraParameter cam_para;
    cam_para.pixel_x_number = 1280; // 成像的横向像元数
    cam_para.pixel_y_number = 720; // 成像的纵向像元数
    cam_para.camera_pos_x = 0; // 相机位置(相机在地面的投影为坐标原点)
    cam_para.camera_pos_y = 0;
    cam_para.camera_pos_z = 1.29744; // 相机高度
    float fu = 1437.72915; // 归一化焦距
    float fv = 1437.72915; // 归一化焦距
    cam_para.cu = 640;
    cam_para.cv = 360;

    cam_para.stretch_angle_w = atan(cam_para.pixel_x_number / (2.0 * fu)) * 2;
    cam_para.stretch_angle_h = atan(cam_para.pixel_y_number / (2.0 * fv)) * 2;
    rel_locate.Initialize(posture, cam_para);

    cv::Mat image = cv::imread("test.jpg", 1);
    // IPM
    // 定义IPM的范围
    float x_start_offset = -3.25;
    float x_end_offset = 3.25;
    float x_resolution = 0.02;
    float y_start_offset = 6.0;
    float y_end_offset = 50.0;
    float y_resolution = 0.1;

    int ipm_height = static_cast<int>((y_end_offset - y_start_offset) / y_resolution);
    int ipm_width = static_cast<int>((x_end_offset - x_start_offset) / x_resolution);
    cv::Mat ipm_img(ipm_height, ipm_width, CV_8UC1);

    int row = ipm_height - 1;
    int col = 0;
    float y = 0;
    float x = 0;
    for (y = y_start_offset, row = ipm_height - 1;
            row >= 0;
            y += y_resolution, --row) {
        for (x = x_start_offset, col = 0;
                col < ipm_width;
                x += x_resolution, ++col) {
            int uu, vv;
            rel_locate.GetPixelCoordinate(x, y, 0, &uu, &vv);
            *(ipm_img.ptr<uint8_t>(row, col)) = image.ptr<uint8_t>(vv, uu)[0];
        }
    }

    cv::imwrite("ipm.jpg", ipm_img);

    return 0;
}
