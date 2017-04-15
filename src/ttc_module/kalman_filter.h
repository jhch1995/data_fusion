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
    KalmanFilter() {}
	
    ~KalmanFilter() {}

    void Init( );
    
    static void kf_update(const MatrixXd Xk_pre, const MatrixXd Pk_pre, 
                             const MatrixXd Q, const MatrixXd R, 
                             const MatrixXd F, const MatrixXd H, const MatrixXd z,  
                             MatrixXd &Xk_new, MatrixXd &Pk_new );
    
    static int LowpassFilter1D(const double y_pre, const double x_new, double dt, 
                                     const double filt_hz, double &y_new );

private:

};


}

#endif  // KALMAN_FILTER_H
