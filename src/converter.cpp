#include "converter.h"

namespace N3dicp
{

    Eigen::Vector3d convertOMVecToEIGENVec (OpenMesh::Vec3d omCoord)
    {
        Eigen::Vector3d eCoord(omCoord[0],omCoord[1],omCoord[2]);
        return eCoord;
    }

    OpenMesh::Vec3d convertEIGENVecToOMVec (Eigen::Vector3d eCoord)
    {
        OpenMesh::Vec3d omCoord(eCoord[0],eCoord[1],eCoord[2]);
        return omCoord;
    }
    Eigen::VectorXd convertANNpointToEigenVec (ANNpoint in_annP, int dim)
    {
        Eigen::VectorXd out_eigP(dim);
        for( int i = 0; i < dim; i ++)
        {
            out_eigP[i] = in_annP[i];
        }
        return out_eigP;

    }
    ANNpoint convertEigenVecToANNpoint (Eigen::VectorXd in_eigP)
    {
        int dim = in_eigP.rows();
        // ANNpoint* out_annP = (ANNpoint)std::malloc(sizeof(double*)*3);
        ANNpoint out_annP = annAllocPt(dim);
        for( int i = 0; i < dim; i ++)
        {
            out_annP[i] = in_eigP[i];
        }
        return out_annP;
    }
    Eigen::MatrixXd convertANNarrayToEigenMat (ANNpointArray in_annArray, int dim, int nPts)
    {
        Eigen::MatrixXd out_eigArray(dim,nPts);
        for( int i = 0; i < nPts; i++ )
        {
            // ANNpoint thisAnnPoint = annAllocPt(dim);
            // thisAnnPoint = in_annArray[i];
            Eigen::VectorXd thisEigPoint = convertANNpointToEigenVec (in_annArray[i], dim);
            for( int d = 0; d < dim; d++)
            {
                out_eigArray(d,i) = thisEigPoint(d);
            }
        }
        return out_eigArray;


    }
    ANNpointArray convertEigenMatToANNarray (Eigen::MatrixXd in_eigArray)
    {
        int nPts = in_eigArray.cols();
        int dim = in_eigArray.rows();
        ANNpointArray out_annArray = annAllocPts(nPts, dim);
        for( int i = 0; i < nPts; i++ )
        {
            // std::cout << "pPoint " << i << std::endl;
            Eigen::VectorXd thisPoint = in_eigArray.col(i);
            out_annArray[i] = convertEigenVecToANNpoint (thisPoint);
        }
        return out_annArray;
    }
}//namespace N3dicp
