#include "kalman_filter.h"

using namespace Eigen;  

namespace ttc {
    
void KalmanFilter::Init(const MatrixXd X0, const MatrixXd P0, const MatrixXd Q0, const MatrixXd R0)
{
    m_X0 = X0;
    m_P0 = P0;
    m_Q0 = Q0;
    m_R0 = R0;
        
    m_Xk = m_X0 ;
    m_Pk = m_P0;
    m_Q = m_Q0;
    m_R = m_R0;
}

void KalmanFilter::ResetKfState()
{
    m_Xk = m_X0 ;
    m_Pk = m_P0;
    m_Q = m_Q0;
    m_R = m_R0;
}

// KF function
void KalmanFilter::KfUpdate(const MatrixXd F, const MatrixXd H, const MatrixXd Z )
{
    MatrixXd Xk_predict = F*m_Xk;     // 1.状态一步预测   
    MatrixXd P_predict = F*m_Pk*F.transpose() + m_Q;    // 2.预测误差协方差阵
    MatrixXd S = H*P_predict*H.transpose() + m_R;     // 信息协方差阵
    MatrixXd Kk = P_predict*H.transpose()*S.inverse(); // 3. 增益矩阵
    MatrixXd Z_predict = H*Xk_predict;
    MatrixXd Z_residual = Z - Z_predict;    // 残差
    m_Xk = Xk_predict + Kk*Z_residual;    // 4.状态估计
    m_Pk = P_predict - Kk*H*P_predict;    // 5.协方差估计
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


void KalmanFilter::GetXk(MatrixXd &Xk)
{
    Xk = m_Xk;
}


}
