#include "icp.h"
#include <stdio.h>
#include "converter.h"
#include <iomanip>      // std::setprecision
namespace N3dicp
{
    // ************************************************************
    // Find knn correspondence
    // ************************************************************
    void findCorrespondences (Eigen::MatrixXd& in_pFixed, Eigen::MatrixXd& in_qMoving,Eigen::MatrixXd& out_pFixed, Eigen::MatrixXd& out_qMoving){
        int nPts = in_pFixed.cols(); // actual number of data points
        int nQueryPts = in_qMoving.cols();
        int dim = 3; // number of dimensions
        int k = 1; //number of near neighbors to find
        double eps = 0.0; // eps value for the kd search
        ANNpointArray dataPFixed; // fixed point set
        ANNpoint pointQMoving;// query point
        ANNidxArray nnIdx; // near neighbor indices
        ANNdistArray dists; // near neighbor distances
        ANNkd_tree* kdTree; // search structure

        int dimCorresp = k; // cols - closest k idx of the moving query point
        Eigen::MatrixXd correspArray(nQueryPts,dimCorresp);

        dataPFixed = convertEigenMatToANNarray(in_pFixed);
        pointQMoving = annAllocPt(dim);
        nnIdx = new ANNidx[k];
        dists = new ANNdist[k];
        kdTree = new ANNkd_tree( dataPFixed, nPts, dim);

        // search for the closest neighbour in the kdtree
        for (int i = 0; i < nQueryPts; i++)
        {
            Eigen::VectorXd thisPoint = in_qMoving.col(i);
            pointQMoving = convertEigenVecToANNpoint (thisPoint);

            kdTree->annkSearch(pointQMoving, k, nnIdx, dists, eps);
            // std::cout<< "*** distance = " << dists[0] << " and points idx = " << nnIdx[0] << std::endl;
            for( int kk = 0; kk < k; kk++)
            {
                correspArray(i,kk) = nnIdx[kk];
            }
            annDeallocPt(pointQMoving);
        }
        // deep copy! if you change in_q, out_q will not change
        out_qMoving = in_qMoving;
        // std::cout << "*** before in_q "<<in_qMoving.col(0) << std::endl;
        // std::cout << "*** before out_q "<<out_qMoving.col(0) << std::endl;
        // Eigen::Vector3d testq0(3);
        // testq0(0) = 0, testq0(1) = 0,testq0(2) = 0;
        // in_qMoving.col(0) = testq0;
        // std::cout << "*** after in_q "<<in_qMoving.col(0) << std::endl;
        // std::cout << "*** after out_q "<<out_qMoving.col(0) << std::endl;


        for (int i = 0; i < nQueryPts; i++)
        {
            int thisIdx = correspArray(i,0);
            out_pFixed.col(i) = in_pFixed.col(thisIdx);
        }

        // double checkQ = (in_qMoving.array() - out_qMoving.array()).sum();
        // std::cout << "checkQ = " << checkQ << std::endl;

        delete[] nnIdx;
        delete[] dists;
        delete kdTree;
        annDeallocPts(dataPFixed);
        annDeallocPt(pointQMoving);
        annClose(); // deallocate any shared memory used for the kd search

    }

    void getIcpIteration(Eigen::MatrixXd& in_pFixed, Eigen::MatrixXd& in_qMoving, Eigen::Matrix3d& rotationMatrix, Eigen::Vector3d& translationVec)
    {
        int numPts = in_qMoving.cols();

        Eigen::MatrixXd out_p(3,numPts);
        Eigen::MatrixXd out_q(3,numPts);
        findCorrespondences (in_pFixed, in_qMoving, out_p, out_q);

        // *** Subtract the center of the point clouds
        Eigen::Vector3d meanP(out_p.row(0).mean(),out_p.row(1).mean(),out_p.row(2).mean());
        Eigen::Vector3d meanQ(out_q.row(0).mean(),out_q.row(1).mean(),out_q.row(2).mean());
        // std::cout << "mean p" << meanP << std::endl;
        // std::cout << "mean q" << meanQ << std::endl;
        // std::cout << "before p0" << out_p.col(0) << std::endl;

        // // demeaned cloud points
        Eigen::MatrixXd diff_p = out_p - meanP.replicate(1,numPts);
        Eigen::MatrixXd diff_q = out_q - meanQ.replicate(1,numPts);

        // std::cout << "after p0" << diff_p.col(0) << std::endl;

        // // compute the covariance matrix
        Eigen::Matrix3d covMat(Eigen::Matrix3d::Zero());
        for( int i = 0; i < numPts; i++)
        {
            covMat += diff_p.col(i) * diff_q.col(i).transpose();
        }
        // covMat = covMat * (1/(numPts-1)); // normalize the covariance matrix

        // std::cout<< "*** covmat sum" << std::scientific << covMat << std::endl;
        covMat = (diff_p * diff_q.transpose()) * ((diff_q * diff_q.transpose()).inverse());
        // std::cout<< "*** covmat product" << std::scientific << covMat << std::endl;

        // std::cout << "outp: " << out_p.block<3,3>(0,0) << std::endl;
        // std::cout << "covMat " << std::setprecision(17) << covMat << std::endl;
        // std::cout<< std::scientific << covMat << std::endl;

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(covMat, Eigen::ComputeFullU | Eigen::ComputeFullV);

        // std::cout << "Its singular values are:" << std::endl << svd.singularValues() << std::endl;
        // std::cout << "Its left singular vectors are the columns of the full U matrix:" << std::endl << svd.matrixU() << std::endl;
        // std::cout << "Its right singular vectors are the columns of the full V matrix:" << std::endl << svd.matrixV() << std::endl;

        rotationMatrix = svd.matrixU() * svd.matrixV().transpose();

        translationVec = meanP - rotationMatrix * meanQ;

        // std::cout << "*** rotMat = " << rotationMatrix << "\n\n";
        // std::cout << "*** translVec = " << translationVec << "\n\n";

    }

    void applyICP(Eigen::MatrixXd& in_pFixed, Eigen::MatrixXd& in_qMoving, Eigen::MatrixXd& final_qMoving, int maxIter, double err)
    {
        int numPts = in_qMoving.cols();
        Eigen::Matrix3d rotMat(Eigen::Matrix3d::Identity());
        Eigen::Vector3d translVec(0,0,0);
        Eigen::MatrixXd updated_qMoving(in_qMoving);

        for( int i = 0; i <= maxIter; i++ )
        {
            getIcpIteration(in_pFixed, updated_qMoving, rotMat, translVec);

            updated_qMoving = rotMat * updated_qMoving + translVec.replicate(1,numPts);

            // compute error and check if it's smaller than the threshold!
        }

        final_qMoving = updated_qMoving;

    }

}// namespace N3dicp
