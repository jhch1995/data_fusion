#include <vector>
#include "gflags/gflags.h"
//#include "gflags.h"
#include "bird_perspective_mapping.h"
#include "opencv2/opencv.hpp"

DEFINE_string(image_name, "./bird_perspective.png", "image_name");
DEFINE_double(fu, 309.4362, "fu");
DEFINE_double(fv, 344.2161, "fv");
DEFINE_double(cu, 317.9034, "cu");
DEFINE_double(cv, 256.5352, "cv");
DEFINE_double(camera_height, 2.1798, "camera height mm");
DEFINE_double(pitch, 14.0, "pitch angle (degree)");
DEFINE_double(yaw, -12.0, "yaw angle (degree)");
DEFINE_int32(image_width, 640, "image width");
DEFINE_int32(image_height, 480, "image height");
DEFINE_double(x_start_offset, -4.0, "x start offset");
DEFINE_double(x_end_offset, 4.0, "x start offset");
DEFINE_double(y_start_offset, 5.0, "y start offset");
DEFINE_double(y_end_offset, 55.0, "y end offset");
DEFINE_double(x_res, 0.02, "x resolution");
DEFINE_double(y_res, 0.1, "y resolution");

void LoadImage(cv::Mat* image)
{
    cv::Mat org_image = cv::imread(FLAGS_image_name, 1);
    cv::Mat channels[3];
    std::cerr << "org:" << int(org_image.at<uint8_t>(220, 100, 0)) << std::endl;
    cv::split(org_image, &channels[0]);
    image->create(org_image.rows, org_image.cols, CV_32FC1);
    std::cerr << "channel: " << int(channels[0].at<uint8_t>(220, 100)) << std::endl;
    channels[0].convertTo(*image, CV_32FC1);
    *image = *image * (1.0 / 255);
    std::cerr << "image: " << image->at<float>(220, 100) << std::endl;
}

int main(int argc, char *argv[])
{
    google::ParseCommandLineFlags(&argc, &argv, true);
    // parameter set up
    CameraPara camera_para;
    camera_para.fu = FLAGS_fu;
    camera_para.fv = FLAGS_fv;

    camera_para.cu = FLAGS_cu;
    camera_para.cv = FLAGS_cv;
    camera_para.height = FLAGS_camera_height; // mm

    camera_para.pitch = FLAGS_pitch * CV_PI / 180;
    camera_para.yaw = FLAGS_yaw * CV_PI / 180;

    camera_para.image_width = FLAGS_image_width;
    camera_para.image_height = FLAGS_image_height;

    BirdPerspectiveMapping bp_mapping(camera_para);

    // ipm para
    IPMPara ipm_para;
    ipm_para.x_limits[0] = FLAGS_x_start_offset;
    ipm_para.x_limits[1] = FLAGS_x_end_offset;
    ipm_para.y_limits[0] = FLAGS_y_start_offset;
    ipm_para.y_limits[1] = FLAGS_y_end_offset;
    ipm_para.x_scale = FLAGS_x_res;
    ipm_para.y_scale = FLAGS_y_res;

    bp_mapping.GetUVLimitsFromXY(&ipm_para);

    std::cerr << "u_limits_0 = " << ipm_para.u_limits[0]
        << ", u_limits_1 = " << ipm_para.u_limits[1]
        << ", v_limits_0 = " << ipm_para.v_limits[0]
        << ", v_limits_1 = " << ipm_para.v_limits[1]
        << ", ipm_height = " << ipm_para.height
        << ", ipm_width = " << ipm_para.width
        << std::endl;

    // ipm transform
    cv::Mat org_image;
    LoadImage(&org_image);
    cv::Mat ipm_image = cv::Mat::zeros(ipm_para.height, ipm_para.width, CV_32FC1);
    for (int i = 0; i < ipm_para.height; ++i) {
        int base = i * ipm_para.width;
        for (int j = 0; j < ipm_para.width; ++j) {
            int offset = base + j;
            float ui = ipm_para.uv_grid.at<float>(0, offset);
            float vi = ipm_para.uv_grid.at<float>(1, offset);
            if (ui < ipm_para.u_limits[0] || ui > ipm_para.u_limits[1] ||
                vi < ipm_para.v_limits[0] || vi > ipm_para.v_limits[1])
                continue;
            int x1 = int(ui), x2 = int(ui + 1);
            int y1 = int(vi), y2 = int(vi + 1);
            float x = ui - x1, y = vi - y1;
            float val = org_image.at<float>(y1, x1) * (1 - x) * (1-y) +
                org_image.at<float>(y1, x2) * x * (1-y) +
                org_image.at<float>(y2, x1) * (1-x) * y +
                org_image.at<float>(y2, x2) * x * y;
            ipm_image.at<float>(i, j) = static_cast<float>(val);
//            std::cerr << "ui = " << ui << ", vi = " << vi << ", i = " << i << ", j = " << j << ", val = " << val << std::endl;
        }
    }
    cv::imshow("ipm_cmp", ipm_image);
    cv::waitKey(-1);

    return 0;
}
