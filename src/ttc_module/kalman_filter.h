#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

// #if defined(_MSC_VER) && (_MSC_VER >= 1600)
// #pragma execution_character_set("utf-8")
// #endif

#include <iostream>  
#include <Eigen/Dense>  
using namespace Eigen;

namespace ttc {
    
class KalmanFilter
{
public:

    /// @brief construct  obj
    KalmanFilter() 
    {
    }
	
    ~KalmanFilter() {}

    void Init(const MatrixXd X0, const MatrixXd P0, const MatrixXd Q0, const MatrixXd R0);
    
    // reset all the KF state(X,P,Q,R)
    void ResetKfState();
    
    void KfUpdate( const MatrixXd F, const MatrixXd H, const MatrixXd z );
    
    void GetXk(MatrixXd &Xk);
    
    static int LowpassFilter1D(const double y_pre, const double x_new, double dt, 
                                     const double filt_hz, double &y_new );

private:
    MatrixXd m_Xk, m_X0;
    MatrixXd m_Pk, m_P0;
    MatrixXd m_Q, m_Q0;
    MatrixXd m_R, m_R0;
    

};


}

#endif  // KALMAN_FILTER_H
