#ifndef __CONVERTER__
#define __CONVERTER__

#include <Eigen/Dense>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include "ANN/ANN.h"
#include <stdio.h>
#include <stdlib.h>     /* malloc, free, rand */


namespace N3dicp
{

Eigen::Vector3d convertOMVecToEIGENVec (OpenMesh::Vec3d omCoord);

OpenMesh::Vec3d convertEIGENVecToOMVec (Eigen::Vector3d eCoord);

Eigen::VectorXd convertANNpointToEigenVec (ANNpoint in_annP, int dim);

ANNpoint convertEigenVecToANNpoint (Eigen::VectorXd in_eigP);

Eigen::MatrixXd convertANNarrayToEigenMat (ANNpointArray in_annArray, int dim, int nPts);

ANNpointArray convertEigenMatToANNarray (Eigen::MatrixXd in_eigArray);

}//namespace N3dicp
#endif
