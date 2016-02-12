// compile with: gcc -I/usr/local/Cellar/glfw3/3.1.2/include -L/usr/local/Cellar/glfw3/3.1.2/lib/ -framework OpenGL -framework Cocoa -framework IOKit -framework CoreVideo -lglfw3 -o main -L/usr/lib/ -lstdc++ src/main.cpp
// standard headers
// basic file operations
#include <fstream>
#include <iostream>
#include <string>
#include "PlyMesh.h"
#include "utils.h"
#include "icp.h"
//#include <vector>
// OpenGL headers
//#include <GL/glew.h>
//#include <GLFW/glfw3.h>
#include <Eigen/Dense>
using namespace N3dicp;

#define TRANSF00_45 1
#define TRANSF00_90 2
#define TRANSF00_180 3
#define TRANSF00_270 4
#define TRANSF00_315 5
#define TRANSF_45 6



static const int MESH_NUMBER = 2;
static const char* BASE_PATH = "../src/plyMeshes/";
static const char* EXTENSION = ".ply";

static const char* MODEL_1_FILENAME = "../src/plyMeshes/_bun000.ply";
static const char* MODEL_2_FILENAME = "../src/plyMeshes/_bun045.ply";
static const char* MODEL_3_FILENAME = "../src/plyMeshes/_bun090.ply";
static const char* MODEL_4_FILENAME = "../src/plyMeshes/_bun180.ply";
static const char* MODEL_5_FILENAME = "../src/plyMeshes/_bun270.ply";
static const char* MODEL_6_FILENAME = "../src/plyMeshes/_bun315.ply";
static const char* MODEL_OUT1_FILENAME = "../src/plyMeshes/_bun000_init.ply";
static const char* MODEL_OUT2_INIT_FILENAME = "../src/plyMeshes/_bun045_init.ply";
static const char* MODEL_OUT2_FIN_FILENAME = "../src/plyMeshes/_bun045_fin.ply";
static const char* MODEL_OUT3_FILENAME = "../src/plyMeshes/_bun090_init.ply";
static const char* MODEL_OUT4_FILENAME = "../src/plyMeshes/_bun180_init.ply";
static const char* MODEL_OUT5_FILENAME = "../src/plyMeshes/_bun270_init.ply";
static const char* MODEL_OUT6_FILENAME = "../src/plyMeshes/_bun315_init.ply";
static const char* MODEL_OUT_FIN = "../src/plyMeshes/_bun_final.ply";
// ***************************************************************************
// SECTION 2
static const int S2_ICP_MAXITER = 30;
static const double S2_ICP_MAXERR = 0.0001;
static const double S2_ICP_BADPOINTSFILTER = 0.3;

// ***************************************************************************
// SECTION 3
static const int S3_NUM_ROTATIONS = 25;
static const int S3_MIN_ANGLE_DEGREES = -50;
static const int S3_MAX_ANGLE_DEGREES = 50;

static const int S3_ICP_MAXITER = 50;
static const double S3_ICP_MAXERR = 0.0001;
static const double S3_ICP_BADPOINTSFILTER = 0.7;
// ***************************************************************************
// SECTION 4
static const int S4_NUM_ITER = 1;
static const float S4_MIN_NOISE_STD = 0.0003;
static const float S4_MAX_NOISE_STD = 0.0003;

static const int S4_ICP_MAXITER = 50;
static const double S4_ICP_MAXERR = 0.0001;
static const double S4_ICP_BADPOINTSFILTER = 0.7;

// ***************************************************************************
// SECTION 5
static const int S5_NUM_ITER = 15;
// between 0 and 100
static const int S5_MIN_SAMPL = 10;
static const int S5_MAX_SAMPL = 90;

static const int S5_ICP_MAXITER = 50;
static const double S5_ICP_MAXERR = 0.0001;
static const double S5_ICP_BADPOINTSFILTER = 0.7;

// ***************************************************************************
// SECTION 6
static const int S6_ICP_MAXITER = 30;
static const double S6_ICP_MAXERR = 0.0001;
static const double S6_ICP_BADPOINTSFILTER = 0.4;

// ***************************************************************************
// SECTION 7
static const int S7_ICP_MAXITER = 30;
static const double S7_ICP_MAXERR = 0.0001;
static const double S7_ICP_BADPOINTSFILTER = 0.5;
static const double S7_ANGLE_THRESH = 5;

void SECTION2 () // basic ICP
{
    printMessage("TASK 2");

    printMessage("reading ply mesh 1 ...");
    PlyMesh mesh_00(MODEL_1_FILENAME);

    printMessage("reading ply mesh 2 ...");
    PlyMesh mesh_45(MODEL_2_FILENAME);

    // apply transformation - good initial alignment
    Eigen::Matrix4d initTransf = loadTransformation(TRANSF00_45);

    mesh_45.applyTransformation(initTransf);

    // ************************************************************
    // Get point clouds - fixed p, moving q

    // matrix of vertex location, dim: 3 x totalVert
    Eigen::MatrixXd p(3,mesh_00.getTotalVertices());
    mesh_00.getVecticesE(p);
    // std::cout << "Mat point 3" << q.block<3,1>(0,2) << std::endl;

    Eigen::MatrixXd q(3,mesh_45.getTotalVertices());
    mesh_45.getVecticesE(q);


    // ************************************************************
    int numPts = q.cols();
    Eigen::MatrixXd final_q(3,numPts);

    int out_numIter = 0;
    double out_finErr = 0.0;

    applyICP(p, q, final_q, out_numIter, out_finErr, S2_ICP_MAXITER, S2_ICP_MAXERR, S2_ICP_BADPOINTSFILTER);

    mesh_45.writeMesh(MODEL_OUT2_INIT_FILENAME);
    // ************************************************************
    // Save results
    mesh_45.setVerticesE(final_q);

    printMessage("writing ply meshes...");
    // mesh_00.writeMesh(MODEL_OUT1_FILENAME);
    mesh_45.writeMesh(MODEL_OUT2_FIN_FILENAME);

    printMessage("***");
}

// ***************************************************************************
void SECTION3() // ICP with rotation variation
{
    printMessage("TASK 3");
    // ************************************************************
    // INIT variables
    float rotX = 0;
    float rotY = 0;
    float rotZ = 0; // from 0 to 1
    Eigen::VectorXd anglesDegree;
    Eigen::VectorXd anglesRadians;
    std::ofstream outputFile;
    outputFile.open ("task3_results.txt");


    std::string filenameW("_M3rotated_");

    Eigen::Matrix3d rotationMat(Eigen::Matrix3d::Zero());
    Eigen::Matrix4d transformationMat(Eigen::Matrix4d::Zero());
    transformationMat(3,3) = 1;

    anglesDegree = Eigen::VectorXd::LinSpaced(S3_NUM_ROTATIONS, S3_MIN_ANGLE_DEGREES, S3_MAX_ANGLE_DEGREES);

    anglesRadians = (M_PI / 180 * anglesDegree.array()).matrix();

    // Read mesh M1 and init M3

    printMessage("reading ply mesh 1 ...");
    PlyMesh mesh_M1(MODEL_1_FILENAME);

    // center the mesh
    int numPts = mesh_M1.getTotalVertices();
    Eigen::MatrixXd p(3,numPts);
    mesh_M1.getVecticesE(p);
    Eigen::Vector3d meanP(p.row(0).mean(),p.row(1).mean(),p.row(2).mean());
    p = p - meanP.replicate(1,numPts);
    mesh_M1.setVerticesE(p);
    std::string m1Filename = BASE_PATH + std::string("_M1_centered_") + EXTENSION;
    mesh_M1.writeMesh(m1Filename);


    // ************************************************************
    // Generate rotations
    for( int i = 0; i < S3_NUM_ROTATIONS; i ++)
    {
        std::cout << "\nT3 - iteration= " << i << "\n";
        std::cout << "angle = " << anglesDegree[i] << "\n";

        // createRotationMatrix(anglesRadians[i], rotY, rotZ, rotationMat);
        // createRotationMatrix(rotX, anglesRadians[i], rotZ, rotationMat);
        createRotationMatrix(rotX, rotY, anglesRadians[i], rotationMat);

        transformationMat.block<3,3>(0,0) = rotationMat;//.data();
        // std::cout << "*** transfMat idx " << i << " :\n" << transformationMat << "\n\n";
        // ************************************************************
        // Apply transformation to the mesh and run ICP
        PlyMesh mesh_M3(mesh_M1); // this creates a new object - why? OpenMesh magic
        mesh_M3.applyTransformation(transformationMat);


        // ************************************************************
        // Get point clouds - fixed p, moving q
        Eigen::MatrixXd q(3,numPts);
        mesh_M3.getVecticesE(q);
        // ************************************************************
        Eigen::MatrixXd final_q(3,numPts);


        int out_numIter = 0;
        double out_finErr = 0.0;

        applyICP(p, q, final_q, out_numIter, out_finErr, S3_ICP_MAXITER, S3_ICP_MAXERR, S3_ICP_BADPOINTSFILTER);

        // save iterations and errors in a file somewhere
        if (outputFile.is_open())
        {
            outputFile <<  anglesDegree[i] << "," << out_numIter << "," << out_finErr << "\n";
        }
        else std::cout << "Couldn't open output file.\n";
        // ************************************************************
        // Save results
        mesh_M3.setVerticesE(final_q);

        // ************************************************************
        // Save iteration
        std::string iterationNo(std::to_string(i));
        std::string m3Filename = BASE_PATH + filenameW + iterationNo + EXTENSION;
        // std::string filen1("_M1_aftercopy");
        // std::string m1Filename = BASE_PATH + filen1 + iterationNo + EXTENSION;
        // mesh_M1.writeMesh(m1Filename);
        mesh_M3.writeMesh(m3Filename);

    }
    if (outputFile.is_open())
    {
        outputFile.close();
    }

}
// ***************************************************************************
void SECTION4() // ICP with add random noise
{
    printMessage("TASK 4");
    // ************************************************************
    // INIT
    int i = 1;
    std::string filenameW("_M1noisy_");
    // double noiseStd = 0.001;
    Eigen::VectorXd stdVect;
    stdVect = Eigen::VectorXd::LinSpaced(S4_NUM_ITER, S4_MIN_NOISE_STD, S4_MAX_NOISE_STD);
    std::ofstream outputFile;
    outputFile.open ("task4_results.txt");


    printMessage("reading ply mesh 1 ...");
    PlyMesh mesh_M1(MODEL_1_FILENAME);
    printMessage("reading ply mesh 2 ...");
    PlyMesh mesh_M2(MODEL_2_FILENAME);

    // apply transformation - good initial alignment
    Eigen::Matrix4d initTransf = loadTransformation(TRANSF00_45);
    mesh_M2.applyTransformation(initTransf);


    int numPts = mesh_M1.getTotalVertices();
    Eigen::MatrixXd p(3,numPts);
    mesh_M1.getVecticesE(p);


    for (int i = 0; i < S4_NUM_ITER; i++  )
    {
        std::cout << "\nT4 - iteration= " << i << "\n";
        std::cout << "noiseStd = " << stdVect(i) << "\n";
        // ************************************************************
        PlyMesh mesh_M3(mesh_M2);
        mesh_M3.addNoise(stdVect(i));

        // ************************************************************
        // run ICP analysis
        // ************************************************************
        // Get point clouds - fixed p, moving q

        Eigen::MatrixXd q(3,numPts);
        mesh_M3.getVecticesE(q);
        // ************************************************************
        Eigen::MatrixXd final_q(3,numPts);
        int out_numIter = 0;
        double out_finErr = 0.0;
        applyICP(p, q, final_q, out_numIter, out_finErr, S4_ICP_MAXITER, S4_ICP_MAXERR, S4_ICP_BADPOINTSFILTER);
        // save iterations and errors in a file somewhere
        if (outputFile.is_open())
        {
            outputFile <<  stdVect[i] << "," << out_numIter << "," << out_finErr << "\n";
        }
        else std::cout << "Couldn't open output file.\n";
        // ************************************************************
        // Save results
        mesh_M3.setVerticesE(final_q);

        // ************************************************************
        // Save results
        std::string iterationNo(std::to_string(i));
        std::string m1Filename = BASE_PATH + filenameW + iterationNo + EXTENSION;
        mesh_M3.writeMesh(m1Filename);
    }

    if (outputFile.is_open())
    {
        outputFile.close();
    }
}
// ***************************************************************************
void SECTION5() // ICP with subsampling
{
    printMessage("TASK 5");
    // ************************************************************
    // INIT
    std::string filenameW ("_M2subsampled_");
    std::ofstream outputFile;
    outputFile.open ("task5_results.txt");
    Eigen::VectorXd samplingVect;
    samplingVect = Eigen::VectorXd::LinSpaced(S5_NUM_ITER, S5_MIN_SAMPL, S5_MAX_SAMPL);
    // float subsamplingRate = 30; // % - between 0 and 100
    printMessage("reading ply mesh 1 ...");
    PlyMesh mesh_M1(MODEL_1_FILENAME);
    printMessage("reading ply mesh 2 ...");
    PlyMesh mesh_M2(MODEL_2_FILENAME);


    // apply transformation - good initial alignment
    Eigen::Matrix4d initTransf = loadTransformation(TRANSF00_45);
    mesh_M2.applyTransformation(initTransf);


    Eigen::MatrixXd p(3,mesh_M1.getTotalVertices());
    mesh_M1.getVecticesE(p);

    Eigen::MatrixXd q(3,mesh_M2.getTotalVertices());
    mesh_M2.getVecticesE(q);

    // ************************************************************
    // ICP magic
    int numPts = q.cols();

    for (int i = 0; i < S5_NUM_ITER; i++  )
    {
        std::cout << "\nT5 - iteration= " << i << "\n";
        std::cout << "sampling rate = " << samplingVect(i) << "\n";

        long newNumPoints = (long)((samplingVect(i)/100)*numPts);
        Eigen::MatrixXd qSubsampled(3,newNumPoints);
        subsample(q, qSubsampled, samplingVect(i));

        // ************************************************************
        // run ICP
        // ************************************************************
        Eigen::MatrixXd final_q(3,newNumPoints);
        int out_numIter = 0;
        double out_finErr = 0.0;
        applyICP(p, qSubsampled, final_q, out_numIter, out_finErr, S5_ICP_MAXITER, S5_ICP_MAXERR, S5_ICP_BADPOINTSFILTER);
        // save iterations and errors in a file somewhere
        if (outputFile.is_open())
        {
            outputFile <<  samplingVect[i] << "," << out_numIter << "," << out_finErr << "\n";
        }
        else std::cout << "Couldn't open output file.\n";
        // Eigen::MatrixXd final_q(qSubsampled);
        // ************************************************************
        // Save results
        PlyMesh mesh_M2_ss(final_q);
        std::string iterationNo(std::to_string(i));
        std::string m1Filename = BASE_PATH + filenameW + iterationNo + EXTENSION;
        mesh_M2_ss.writeMesh(m1Filename);
    }

    if (outputFile.is_open())
    {
        outputFile.close();
    }
}
// ***************************************************************************
void SECTION6() // multiple body registration - bad
{
    printMessage("TASK 6");
    int out_numIter = 0;
    double out_finErr = 0.0;
    double newNumPoints;
    printMessage("reading ply mesh 1 ...");
    PlyMesh mesh_00(MODEL_1_FILENAME);
    printMessage("reading ply mesh 2 ...");
    PlyMesh mesh_45(MODEL_2_FILENAME);

    // apply transformation - good initial alignment
    Eigen::Matrix4d initTransf = loadTransformation(TRANSF00_45);
    mesh_45.applyTransformation(initTransf);

    // ************************************************************
    // Get point clouds - fixed p, moving q

    // matrix of vertex location, dim: 3 x totalVert
    Eigen::MatrixXd p(3,mesh_00.getTotalVertices());
    mesh_00.getVecticesE(p);
    // std::cout << "Mat point 3" << q.block<3,1>(0,2) << std::endl;

    Eigen::MatrixXd q(3,mesh_45.getTotalVertices());
    mesh_45.getVecticesE(q);
    // ************************************************************
    int numPts = q.cols();
    Eigen::MatrixXd final_q(3,numPts);


    applyICP(p, q, final_q, out_numIter, out_finErr, S6_ICP_MAXITER, S6_ICP_MAXERR, S6_ICP_BADPOINTSFILTER);

    Eigen::MatrixXd mergedPC1(p.rows(), p.cols() + final_q.cols());
    mergedPC1 << p, final_q;
    newNumPoints = (50.0/100.0) * (double)mergedPC1.cols();
    // int32_t cols_ss = (int32_t)newNumPoints;
    // std::cout<< cols_ss << '\n';
    Eigen::MatrixXd mergedPC1_ss(3,(int)newNumPoints);
    // std::cout<< mergedPC1.rows() << " and cols " << mergedPC1.cols() <<'\n';
    subsample(mergedPC1, mergedPC1_ss, 50);
    mergedPC1.resize(0,0);
    final_q.resize(0,0);


    // printMessage("reading ply mesh 3 ...");
    PlyMesh mesh_90(MODEL_6_FILENAME);

    Eigen::MatrixXd p90(3,mesh_90.getTotalVertices());
    mesh_90.getVecticesE(p90);
    numPts = p90.cols();
    Eigen::MatrixXd final_p90(3,numPts);
    applyICP(mergedPC1_ss, p90, final_p90, out_numIter, out_finErr, S6_ICP_MAXITER, S6_ICP_MAXERR, S6_ICP_BADPOINTSFILTER);

    Eigen::MatrixXd mergedPC2(mergedPC1_ss.rows(), mergedPC1_ss.cols() + final_p90.cols());
    mergedPC2 << mergedPC1_ss, final_p90;
    newNumPoints = (50.0/100.0)*(double)mergedPC2.cols();
    Eigen::MatrixXd mergedPC2_ss(3,(int)newNumPoints);
    subsample(mergedPC2, mergedPC2_ss, 50);
    mergedPC2.resize(0,0);
    mergedPC1.resize(0,0);

    printMessage("writing final ply mesh...");
    PlyMesh mesh_fin(mergedPC2_ss);
    mesh_fin.writeMesh(MODEL_OUT_FIN);
    printMessage("***");

}
void SECTION6_1() // multiple body registration - closer
{
    printMessage("TASK 6.1");
    int out_numIter = 0;
    double out_finErr = 0.0;
    int numPts;
    printMessage("reading ply mesh 1 ...");
    PlyMesh mesh_00(MODEL_1_FILENAME);
    printMessage("reading ply mesh 2 ...");
    PlyMesh mesh_45(MODEL_2_FILENAME);
    printMessage("reading ply mesh 3 ...");
    PlyMesh mesh_90(MODEL_3_FILENAME);
    printMessage("reading ply mesh 4 ...");
    PlyMesh mesh_180(MODEL_4_FILENAME);
    printMessage("reading ply mesh 5 ...");
    PlyMesh mesh_270(MODEL_5_FILENAME);
    printMessage("reading ply mesh 6 ...");
    PlyMesh mesh_315(MODEL_6_FILENAME);



    // mesh_00.writeMesh(MODEL_OUT1_FILENAME);

    Eigen::Matrix4d initTransf = loadTransformation(TRANSF00_45);
    mesh_45.applyTransformation(initTransf);

    initTransf = loadTransformation(TRANSF00_90);
    mesh_90.applyTransformation(initTransf);
    // mesh_90.writeMesh(MODEL_OUT3_FILENAME);

    initTransf = loadTransformation(TRANSF00_180);
    mesh_180.applyTransformation(initTransf);
    // mesh_180.writeMesh(MODEL_OUT4_FILENAME);

    initTransf = loadTransformation(TRANSF00_180);
    mesh_270.applyTransformation(initTransf);
    initTransf = loadTransformation(TRANSF00_90);
    mesh_270.applyTransformation(initTransf);
    // mesh_270.writeMesh(MODEL_OUT5_FILENAME);

    initTransf = loadTransformation(TRANSF00_180);
    mesh_315.applyTransformation(initTransf);
    initTransf = loadTransformation(TRANSF00_90);
    mesh_315.applyTransformation(initTransf);
    initTransf = loadTransformation(TRANSF00_45);
    mesh_315.applyTransformation(initTransf);
    // mesh_315.writeMesh(MODEL_OUT6_FILENAME);

    // ************************************************************
    // ICP 00 - 45

    Eigen::MatrixXd p00(3,mesh_00.getTotalVertices());
    mesh_00.getVecticesE(p00);
    Eigen::MatrixXd p45(3,mesh_45.getTotalVertices());
    mesh_45.getVecticesE(p45);
    numPts = p45.cols();
    Eigen::MatrixXd final_p45(3,numPts);
    applyICP(p00, p45, final_p45, out_numIter, out_finErr, S6_ICP_MAXITER, S6_ICP_MAXERR, S6_ICP_BADPOINTSFILTER);
    mesh_45.setVerticesE(final_p45);

    // Eigen::MatrixXd mergedPC1(p00.rows(), p00.cols() + final_p45.cols());
    // mergedPC1 << p00, final_p45;
    // numPts = (int)((30.0/100.0) * (double)mergedPC1.cols());
    // Eigen::MatrixXd mergedPC1_ss(3,numPts);
    // subsample(mergedPC1, mergedPC1_ss, 30);
    //clean up everything we don't need
    // p00.resize(0,0);
    // p45.resize(0,0);
    // mergedPC1.resize(0,0);
    // final_p45.resize(0,0);

    // ************************************************************
    // ICP 45 - 90

    Eigen::MatrixXd p90(3,mesh_90.getTotalVertices());
    mesh_90.getVecticesE(p90);
    numPts = p90.cols();

    // numPts = (int)((50.0/100.0) * (double)p90.cols());
    // Eigen::MatrixXd p90_ss(3,numPts);
    // subsample(p90, p90_ss, 50);

    Eigen::MatrixXd final_p90(3,numPts);
    applyICP(p45, p90, final_p90, out_numIter, out_finErr, S6_ICP_MAXITER, S6_ICP_MAXERR, S6_ICP_BADPOINTSFILTER);

    mesh_90.setVerticesE(final_p90);
    // PlyMesh newMesh_90(final_p90);
    // final_p45.resize(0,0);
    // p45.resize(0,0);
    // p90.resize(0,0);


    // ************************************************************
    // ICP 90 - 180

    Eigen::MatrixXd p180(3,mesh_180.getTotalVertices());
    mesh_180.getVecticesE(p180);
    numPts = p180.cols();

    // numPts = (int)((50.0/100.0) * (double)p90.cols());
    // Eigen::MatrixXd p90_ss(3,numPts);
    // subsample(p90, p90_ss, 50);

    Eigen::MatrixXd final_p180(3,numPts);
    applyICP(p90, p180, final_p180, out_numIter, out_finErr, S6_ICP_MAXITER, S6_ICP_MAXERR, S6_ICP_BADPOINTSFILTER);

    mesh_180.setVerticesE(final_p180);

    // ************************************************************
    // ICP 180 - 270

    Eigen::MatrixXd p270(3,mesh_270.getTotalVertices());
    mesh_270.getVecticesE(p270);
    numPts = p270.cols();

    // numPts = (int)((50.0/100.0) * (double)p90.cols());
    // Eigen::MatrixXd p90_ss(3,numPts);
    // subsample(p90, p90_ss, 50);

    Eigen::MatrixXd final_p270(3,numPts);
    applyICP(p180, p270, final_p270, out_numIter, out_finErr, S6_ICP_MAXITER, S6_ICP_MAXERR, S6_ICP_BADPOINTSFILTER);

    mesh_270.setVerticesE(final_p270);

    // ************************************************************
    // ICP 180 - 270

    Eigen::MatrixXd p315(3,mesh_315.getTotalVertices());
    mesh_315.getVecticesE(p315);
    numPts = p315.cols();

    // numPts = (int)((50.0/100.0) * (double)p90.cols());
    // Eigen::MatrixXd p90_ss(3,numPts);
    // subsample(p90, p90_ss, 50);

    Eigen::MatrixXd final_p315(3,numPts);
    applyICP(p270, p315, final_p315, out_numIter, out_finErr, S6_ICP_MAXITER, S6_ICP_MAXERR, S6_ICP_BADPOINTSFILTER);

    mesh_315.setVerticesE(final_p315);

    // ************************************************************

    mesh_45.writeMesh(MODEL_OUT2_INIT_FILENAME);
    mesh_90.writeMesh(MODEL_OUT3_FILENAME);
    mesh_180.writeMesh(MODEL_OUT4_FILENAME);
    mesh_270.writeMesh(MODEL_OUT5_FILENAME);
    mesh_315.writeMesh(MODEL_OUT6_FILENAME);



    printMessage("***");
}

void SECTION6_N() // multiple body registration with normals
{
    printMessage("TASK 6.1");
    int out_numIter = 0;
    double out_finErr = 0.0;
    int numPts;
    int kNN=10;
    printMessage("reading ply mesh 1 ...");
    PlyMesh mesh_00(MODEL_1_FILENAME);
    printMessage("reading ply mesh 2 ...");
    PlyMesh mesh_45(MODEL_2_FILENAME);
    printMessage("reading ply mesh 3 ...");
    PlyMesh mesh_90(MODEL_3_FILENAME);
    printMessage("reading ply mesh 4 ...");
    PlyMesh mesh_180(MODEL_4_FILENAME);
    printMessage("reading ply mesh 5 ...");
    PlyMesh mesh_270(MODEL_5_FILENAME);
    printMessage("reading ply mesh 6 ...");
    PlyMesh mesh_315(MODEL_6_FILENAME);



    // mesh_00.writeMesh(MODEL_OUT1_FILENAME);

    Eigen::Matrix4d initTransf = loadTransformation(TRANSF00_45);
    mesh_45.applyTransformation(initTransf);

    initTransf = loadTransformation(TRANSF00_90);
    mesh_90.applyTransformation(initTransf);
    // mesh_90.writeMesh(MODEL_OUT3_FILENAME);

    initTransf = loadTransformation(TRANSF00_180);
    mesh_180.applyTransformation(initTransf);
    // mesh_180.writeMesh(MODEL_OUT4_FILENAME);

    initTransf = loadTransformation(TRANSF00_270);
    mesh_270.applyTransformation(initTransf);
    // initTransf = loadTransformation(TRANSF00_90);
    // mesh_270.applyTransformation(initTransf);
    // mesh_270.writeMesh(MODEL_OUT5_FILENAME);

    initTransf = loadTransformation(TRANSF00_315);
    mesh_315.applyTransformation(initTransf);
    // initTransf = loadTransformation(TRANSF00_90);
    // mesh_315.applyTransformation(initTransf);
    // initTransf = loadTransformation(TRANSF00_45);
    // mesh_315.applyTransformation(initTransf);
    // mesh_315.writeMesh(MODEL_OUT6_FILENAME);

    // ************************************************************
    // compute normals for all the meshes
    Eigen::MatrixXd p00(3,mesh_00.getTotalVertices());
    mesh_00.getVecticesE(p00);
    Eigen::MatrixXd p45(3,mesh_45.getTotalVertices());
    mesh_45.getVecticesE(p45);
    Eigen::MatrixXd p90(3,mesh_90.getTotalVertices());
    mesh_90.getVecticesE(p90);
    Eigen::MatrixXd p180(3,mesh_180.getTotalVertices());
    mesh_180.getVecticesE(p180);
    Eigen::MatrixXd p270(3,mesh_270.getTotalVertices());
    mesh_270.getVecticesE(p270);
    Eigen::MatrixXd p315(3,mesh_315.getTotalVertices());
    mesh_315.getVecticesE(p315);

    printMessage ("Compute normals...");

    Eigen::MatrixXd p00Normals(3,p00.cols());
    computeNormals(p00, p00Normals, kNN);
    mesh_00.setNormals(p00Normals);

    Eigen::MatrixXd p45Normals(3,p45.cols());
    computeNormals(p45, p45Normals, kNN);
    mesh_45.setNormals(p45Normals);

    Eigen::MatrixXd p90Normals(3,p90.cols());
    computeNormals(p90, p90Normals, kNN);
    mesh_90.setNormals(p90Normals);

    Eigen::MatrixXd p180Normals(3,p180.cols());
    computeNormals(p180, p180Normals, kNN);
    mesh_180.setNormals(p180Normals);

    Eigen::MatrixXd p270Normals(3,p270.cols());
    computeNormals(p270, p270Normals, kNN);
    mesh_270.setNormals(p270Normals);

    Eigen::MatrixXd p315Normals(3,p315.cols());
    computeNormals(p315, p315Normals, kNN);
    mesh_315.setNormals(p315Normals);

    // ************************************************************
    // ICP 00 - 45
    numPts = p45.cols();
    Eigen::MatrixXd final_p45(3,numPts);
    applyICP(p00, p45, p00Normals, p45Normals, final_p45, out_numIter, out_finErr, S6_ICP_MAXITER, S6_ICP_MAXERR, 0.7, S7_ANGLE_THRESH);
    mesh_45.setVerticesE(final_p45);

    // ************************************************************
    // ICP 45 - 90
    numPts = p90.cols();
    Eigen::MatrixXd final_p90(3,numPts);
    applyICP(p45, p90, p45Normals, p90Normals, final_p90, out_numIter, out_finErr, S6_ICP_MAXITER, S6_ICP_MAXERR, 0.3, S7_ANGLE_THRESH);
    mesh_90.setVerticesE(final_p90);


    // ************************************************************
    // ICP 90 - 180
    numPts = p180.cols();
    Eigen::MatrixXd final_p180(3,numPts);
    applyICP(p90, p180, p90Normals, p180Normals, final_p180, out_numIter, out_finErr, S6_ICP_MAXITER, S6_ICP_MAXERR, S6_ICP_BADPOINTSFILTER, S7_ANGLE_THRESH);

    mesh_180.setVerticesE(final_p180);

    // ************************************************************
    // ICP 180 - 270
    numPts = p270.cols();
    Eigen::MatrixXd final_p270(3,numPts);
    applyICP(p180, p270, p180Normals, p270Normals, final_p270, out_numIter, out_finErr, S6_ICP_MAXITER, S6_ICP_MAXERR, S6_ICP_BADPOINTSFILTER, S7_ANGLE_THRESH);
    mesh_270.setVerticesE(final_p270);

    // ************************************************************
    // ICP 270 - 315
    numPts = p315.cols();
    Eigen::MatrixXd final_p315(3,numPts);
    applyICP(p270, p315, p270Normals, p315Normals, final_p315, out_numIter, out_finErr, S6_ICP_MAXITER, S6_ICP_MAXERR, S6_ICP_BADPOINTSFILTER, S7_ANGLE_THRESH);
    mesh_315.setVerticesE(final_p315);

    // ************************************************************
    // save final transformations
    mesh_00.writeMesh(MODEL_OUT1_FILENAME);
    mesh_45.writeMesh(MODEL_OUT2_INIT_FILENAME);
    mesh_90.writeMesh(MODEL_OUT3_FILENAME);
    mesh_180.writeMesh(MODEL_OUT4_FILENAME);
    mesh_270.writeMesh(MODEL_OUT5_FILENAME);
    mesh_315.writeMesh(MODEL_OUT6_FILENAME);

    printMessage("***");
}
// ***************************************************************************
void SECTION7() // ICP with normals
{
    printMessage("TASK 7");
    // ************************************************************
    // INIT
    std::string filenameW1 ("_M1normals_");
    std::string filenameW2 ("_M2normals_");

    int kNN = 10;
    Eigen::Matrix4d initTransf;
    printMessage("reading ply mesh 1 ...");
    PlyMesh mesh_M1(MODEL_1_FILENAME);
    printMessage("reading ply mesh 2 ...");
    PlyMesh mesh_M2(MODEL_2_FILENAME);

    //
    // initTransf = loadTransformation(TRANSF00_45);
    // mesh_M1.applyTransformation(initTransf);

    initTransf = loadTransformation(TRANSF00_45);
    mesh_M2.applyTransformation(initTransf);

    // std::string m1FilenameBefore = BASE_PATH + filenameW + "before_" + EXTENSION;
    // mesh_M1.writeMesh(m1FilenameBefore);

    // ************************************************************
    Eigen::MatrixXd p(3,mesh_M1.getTotalVertices());
    mesh_M1.getVecticesE(p);
    Eigen::MatrixXd pNormals(3,p.cols());
    computeNormals(p, pNormals, kNN);
    // set normals on the mesh
    mesh_M1.setNormals(pNormals);

    // ************************************************************
    Eigen::MatrixXd q(3,mesh_M2.getTotalVertices());
    mesh_M2.getVecticesE(q);
    Eigen::MatrixXd qNormals(3,q.cols());
    computeNormals(q, qNormals, kNN);
    // set normals on the mesh
    mesh_M2.setNormals(qNormals);

    // ************************************************************
    // // ICP with normals!
    int numPts = q.cols();
    Eigen::MatrixXd final_q(3,numPts);

    int out_numIter = 0;
    double out_finErr = 0.0;

    applyICP(p, q, pNormals, qNormals, final_q, out_numIter,out_finErr,S7_ICP_MAXITER, S7_ICP_MAXERR, S7_ICP_BADPOINTSFILTER, S7_ANGLE_THRESH);
    // applyICP(p, q, final_q, out_numIter,out_finErr,S7_ICP_MAXITER, S7_ICP_MAXERR, S7_ICP_BADPOINTSFILTER);

    mesh_M2.setVerticesE(final_q);

    // ************************************************************
    // Save results
    std::string m1Filename = BASE_PATH + filenameW1 + EXTENSION;
    std::string m2Filename = BASE_PATH + filenameW2 + EXTENSION;

    mesh_M1.writeMesh(m1Filename);
    mesh_M2.writeMesh(m2Filename);

    printMessage("***");
}

// ***************************************************************************
int main(int argc, const char * argv[])
{
    SECTION2(); // basic ICP implementation
    // SECTION3(); // vary rotation
    // SECTION4(); // add Gaussian noise
    // SECTION5(); // subsampling
    // SECTION6(); // multibody - trial 1
    // SECTION6_1(); // multibody - trial 2
    // SECTION6_N(); // multibody - trial 3
    // SECTION7(); // normals

    return 0;
}
