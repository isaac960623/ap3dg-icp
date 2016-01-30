#ifndef __ICP__
#define __ICP__

#include <Eigen/Dense>
#include "ANN/ANN.h"

namespace N3dicp
{
    
    void applyICP(Eigen::MatrixXd& in_pFixed, Eigen::MatrixXd& in_qMoving, Eigen::MatrixXd& final_qMoving, int maxIter = 500, double err = 0.01);
}
#endif
