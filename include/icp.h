#ifndef __ICP__
#define __ICP__

#include <Eigen/Dense>
#include "ANN/ANN.h"

namespace N3dicp
{

    void applyICP(Eigen::MatrixXd& in_pFixed, Eigen::MatrixXd& in_qMoving, Eigen::MatrixXd& final_qMoving, int& out_numIter, double& out_finErr, int maxIter = 500, double errThresh = 0.0001, double BAD_P_FILTER = 0.8);

    void computeNormals(Eigen::MatrixXd& pEIG,Eigen::MatrixXd& pNormals, int kNN = 15);

    void subsample(Eigen::MatrixXd& pInit, Eigen::MatrixXd& pFinal, float subsamplingRate);

}
#endif
