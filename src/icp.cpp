#include "icp.h"
#include <stdio.h>
#include <fstream>
#include <stdlib.h>     /* abs */
#include "converter.h"
#include <math.h>       /* floor */
#include <iomanip>      // std::setprecision
namespace N3dicp
{
    // ************************************************************
    // Find knn correspondence
    // ************************************************************
    void findCorrespondences (Eigen::MatrixXd& in_pFixed, Eigen::MatrixXd& in_qMoving,Eigen::MatrixXd& out_pFixed, Eigen::MatrixXd& out_qMoving, double BAD_P_FILTER){
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

        // 1 col for the closest k, 1 col for the corresponding distance
        Eigen::MatrixXd correspArray(nQueryPts,2);


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
            // closest neighbour
            correspArray(i,0) = nnIdx[0];
            // corresp distance
            correspArray(i,1) = dists[0];

            annDeallocPt(pointQMoving);
        }
        // deep copy! if you change in_q, out_q will not change
        // out_qMoving = in_qMoving;
        // std::cout << "*** before in_q "<<in_qMoving.col(0) << std::endl;
        // std::cout << "*** before out_q "<<out_qMoving.col(0) << std::endl;
        // Eigen::Vector3d testq0(3);
        // testq0(0) = 0, testq0(1) = 0,testq0(2) = 0;
        // in_qMoving.col(0) = testq0;
        // std::cout << "*** after in_q "<<in_qMoving.col(0) << std::endl;
        // std::cout << "*** after out_q "<<out_qMoving.col(0) << std::endl;
        double maxDist = correspArray.col(1).maxCoeff();
        // double minDist = correspArray.col(1).minCoeff();

        // std::cout << "*** max dist "<< maxDist << std::endl;
        // std::cout << "*** min dist "<< minDist << std::endl;
        double filterMaxDist = maxDist * BAD_P_FILTER;
        int outIdx = 0;
        for (int i = 0; i < nQueryPts; i++)
        {
            // int thisIdx = correspArray(i,0);
            // out_pFixed.col(i) = in_pFixed.col(thisIdx);

            if (correspArray(i,1) < filterMaxDist)
            {
                int thisIdx = correspArray(i,0);
                out_qMoving.col(outIdx) = in_qMoving.col(i);
                out_pFixed.col(outIdx) = in_pFixed.col(thisIdx);
                outIdx += 1;
            }

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

    void computeNormals(Eigen::MatrixXd& pEIG, Eigen::MatrixXd& pNormals, int kNN)
    {
        int nPts = pEIG.cols(); // actual number of data points
        // int k = 0; //num of NN to return
        int dim = 3; // number of dimensions
        double eps = 0.0; // eps value for the kd search
        ANNdist sqRad = 0.00005; // returns around 250 - 350 neighbours

        ANNpointArray pANN; // fixed point set
        ANNpoint queryP;// query point
        ANNidxArray nnIdx; // near neighbor indices
        ANNdistArray dists; // near neighbor distances
        ANNkd_tree* kdTree; // search structure

        pANN = convertEigenMatToANNarray(pEIG);
        queryP = annAllocPt(dim);
        nnIdx = new ANNidx[kNN];
        dists = new ANNdist[kNN];
        kdTree = new ANNkd_tree( pANN, nPts, dim);
        // search for the closest neighbour in the kdtree
        // nPts = 1;
        for (int i = 0; i < nPts; i++)
        {
            // std::cout << "*** i = " << i << std::endl;
            Eigen::VectorXd thisPoint = pEIG.col(i);
            queryP = convertEigenVecToANNpoint(thisPoint);
            // approx fixed-radius kNN search

            // how many points are in my sqRad search?
            int numNN = kdTree->annkFRSearch(queryP, sqRad,0);

            // std::cout << "*** num neighb = " << numNN <<  "\n";

            if (numNN > kNN)
            {
                kdTree->annkFRSearch(queryP, sqRad, kNN, nnIdx, dists, eps);

                // my local point cloud is
                Eigen::MatrixXd pLocal(3, kNN);
                for( int j = 0; j < kNN; j++ )
                {
                    pLocal.col(j) = pEIG.col(nnIdx[j]);
                }
                Eigen::Vector3d meanPLocal(pLocal.row(0).mean(),pLocal.row(1).mean(),pLocal.row(2).mean());
                Eigen::MatrixXd diff_p = pLocal - meanPLocal.replicate(1,kNN);

                // std::cout << "*** pLocal = " << pLocal <<  "\n";
                Eigen::Matrix3d qMat(Eigen::Matrix3d::Zero());
                for( int j = 0; j < kNN; j++ )
                {
                    qMat += pLocal.col(j) * pLocal.col(j).transpose();
                }
                // qMat = qMat * (1/kNN); // normalize the cov matrix

                // std::cout<< "*** my cov mat = " << qMat << std::endl;
                //
                Eigen::JacobiSVD<Eigen::MatrixXd> svd(qMat, Eigen::ComputeFullU | Eigen::ComputeFullV);

                // Eigen::EigenSolver<Eigen::Matrix3d> es(qMat);
                // std::cout << "The eigenvalues of A are:" << std::endl << es.eigenvalues() << std::endl;
                // std::cout << "The singular values of A are:" << std::endl << svd.singularValues() << std::endl;
                //
                // std::cout << "Its right singular vectors are the columns of the full V matrix:" << std::endl << svd.matrixV() << std::endl;

                // singular values are always sorted in decreasing order!
                pNormals.col(i) = svd.matrixV().col(2);
            } // else increase the search radius

        }
        delete[] nnIdx;
        delete[] dists;
        delete kdTree;
        annDeallocPts(pANN);
        annDeallocPt(queryP);
        annClose(); // deallocate any shared memory used for the kd search
    }

    void getIcpIteration(Eigen::MatrixXd& in_pFixed, Eigen::MatrixXd& in_qMoving, Eigen::Matrix3d& rotationMatrix, Eigen::Vector3d& translationVec, double& err, double BAD_P_FILTER)
    {
        int numPts = in_qMoving.cols();

        Eigen::MatrixXd out_p(Eigen::MatrixXd::Zero(3,numPts));
        Eigen::MatrixXd out_q(Eigen::MatrixXd::Zero(3,numPts));
        findCorrespondences (in_pFixed, in_qMoving, out_p, out_q, BAD_P_FILTER);

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
        // for( int i = 0; i < numPts; i++)
        // {
        //     covMat += diff_p.col(i) * diff_q.col(i).transpose();
        // }
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

        Eigen::MatrixXd errMat(rotationMatrix * diff_q + translationVec.replicate(1,numPts) - diff_p);
        err = 0;
        for ( int i = 0; i < numPts; i++)
        {
            err += errMat.col(i).squaredNorm();
        }
        // err = errMat.sum();
        // std::cout << "*** err = " << err << std::endl;
        // std::cout << "*** rotMat = " << rotationMatrix << "\n\n";
        // std::cout << "*** translVec = " << translationVec << "\n\n";

    }

    void applyICP(Eigen::MatrixXd& in_pFixed, Eigen::MatrixXd& in_qMoving, Eigen::MatrixXd& final_qMoving, int& out_numIter, double& out_finErr, int maxIter, double errThresh, double BAD_P_FILTER)
    {
        std::ofstream outputFile;
        outputFile.open ("icpErrors.txt");
        int numPts = in_qMoving.cols();
        Eigen::Matrix3d rotMat(Eigen::Matrix3d::Identity());
        Eigen::Vector3d translVec(0,0,0);
        Eigen::MatrixXd updated_qMoving(in_qMoving);
        double thisErr, prevErr;
        int i = 0; //curent iteration
        for( i = 0; i <= maxIter; i++ )
        {
            thisErr = 0;

            getIcpIteration(in_pFixed, updated_qMoving, rotMat, translVec, thisErr, BAD_P_FILTER);

            updated_qMoving = rotMat * updated_qMoving + translVec.replicate(1,numPts);

            double diffErr = std::abs(thisErr - prevErr);
            // compute error and check if it's smaller than the threshold!
            if (outputFile.is_open())
            {
                outputFile <<  thisErr << ",";
            }
            else std::cout << "Couldn't open output file.\n";
            if(diffErr < errThresh)
            {
                // std::cout << "*** ICP - num iterations = " << i << std::endl;
                break;
            }

            prevErr = thisErr;
        }

        final_qMoving = updated_qMoving;
        out_numIter = i;
        out_finErr = thisErr;

        std::cout << "*** ICP - num iterations = " << out_numIter << std::endl;
        std::cout << "*** ICP - final error = " << out_finErr << std::endl;

        if (outputFile.is_open())
        {
            outputFile.close();
        }

    }

    void subsample(Eigen::MatrixXd& pInit, Eigen::MatrixXd& pFinal, float subsamplingRate)
    {
        long numPoints = pInit.cols();
        long newNumPoints = (long)((subsamplingRate/100)*numPoints);
        // pFinal(3,newNumPoints);
        std::cout << "Total subsampled points: " << newNumPoints << std::endl;

        for(int i = 0; i < newNumPoints; i++)
        {
            // if (kInit < numPoints)
            // {
            long randIdx = std::rand() % numPoints;
            pFinal.col(i) = pInit.col(randIdx);
            // pFinal.col(i) = pInit.col(kInit);
            // kInit += stepIdx;
            // }
            // else break;

        }
    }

}// namespace N3dicp
