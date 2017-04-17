#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

// #if defined(_MSC_VER) && (_MSC_VER >= 1600)
// #pragma execution_character_set("utf-8")
// #endif

#include <iostream>  
#include <Eigen/Dense>  
using namespace Eigen;

namespace kf {
    
class KalmanFilter
{
public:

    /// @brief construct  obj
    KalmanFilter(MatrixXd X0, MatrixXd P0, MatrixXd Q0, MatrixXd R0) 
    {
        m_Xk = X0;
        m_Pk = P0;
        m_Q = Q0;
        m_R = R0;
    }
	
    ~KalmanFilter() {}

    void Init( );
    
    void KfUpdate( const MatrixXd F, const MatrixXd H, const MatrixXd z );
    
    void GetState(MatrixXd Xk);
    
    int LowpassFilter1D(const double y_pre, const double x_new, double dt, 
                                     const double filt_hz, double &y_new );

private:
    MatrixXd m_Xk;
    MatrixXd m_Pk;
    MatrixXd m_Q;
    MatrixXd m_R;
    

};


}

#endif  // KALMAN_FILTER_H
