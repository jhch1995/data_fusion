#include "kalman_filter.h"

using namespace Eigen;  

namespace kf {
    
void KalmanFilter::Init( )
{
  
}


// KF function
void KalmanFilter::kf_update(const MatrixXd Xk_pre, const MatrixXd Pk_pre, 
                             const MatrixXd Q, const MatrixXd R, 
                             const MatrixXd F, const MatrixXd H, 
                             const MatrixXd Z,  
                             MatrixXd &Xk_new, MatrixXd &Pk_new )
{
    MatrixXd Xk_predict = F*Xk_pre;     // 1.状态一步预测   
    MatrixXd P_predict = F*Pk_pre*F.transpose() + Q;    // 2.预测误差协方差阵
    MatrixXd S = H*P_predict*H.transpose() + R;     // 信息协方差阵
    MatrixXd Kk = P_predict*H.transpose()*S.inverse(); // 3. 增益矩阵
    MatrixXd Z_predict = H*Xk_predict;
    MatrixXd Z_residual = Z - Z_predict;    // 残差
    Xk_new = Xk_predict + Kk*Z_residual;    // 4.状态估计
    Pk_new = P_predict - Kk*H*P_predict;    // 5.协方差估计
  
}


int KalmanFilter::LowpassFilter1D(const double y_pre, const double x_new, double dt, 
                                     const double filt_hz, double &y_new )
{
    double alpha = 0.0f, rc = 0.0f;

    if(filt_hz <= 0.0f || dt < 0.0f){
        y_new = x_new;
        return 0;
    }else{
        rc = 1.0f/(2.0*3.1415*filt_hz);
        alpha = dt/(dt + rc);
    }
    y_new = y_pre + alpha*(x_new - y_pre);
    return 1;
}


}
