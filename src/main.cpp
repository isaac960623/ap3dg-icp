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
static const char* MODEL_OUT3_FIN_FILENAME = "../src/plyMeshes/_bun090_fin.ply";
static const char* MODEL_OUT4_FILENAME = "../src/plyMeshes/_bun180_init.ply";
static const char* MODEL_OUT5_FILENAME = "../src/plyMeshes/_bun270_init.ply";
static const char* MODEL_OUT6_FILENAME = "../src/plyMeshes/_bun315_init.ply";
// ***************************************************************************
// SECTION 2
static const int S2_ICP_MAXITER = 50;
static const double S2_ICP_MAXERR = 0.0001;
static const double S2_ICP_BADPOINTSFILTER = 0.5;


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
// static const int S5_SAMPLING_RATE= 30;
// between 0 and 100
static const int S5_MIN_SAMPL = 10;
static const int S5_MAX_SAMPL = 90;

static const int S5_ICP_MAXITER = 50;
static const double S5_ICP_MAXERR = 0.0001;
static const double S5_ICP_BADPOINTSFILTER = 0.7;

void SECTION2 () // basic ICP
{
    printMessage("TASK 2");

    printMessage("reading ply mesh 1 ...");
    PlyMesh mesh_00(MODEL_1_FILENAME);

    printMessage("reading ply mesh 2 ...");
    PlyMesh mesh_45(MODEL_2_FILENAME);
    // printMessage("reading ply mesh 3 ...");
    // PlyMesh mesh_90(MODEL_3_FILENAME);
    // printMessage("reading ply mesh 4 ...");
    // PlyMesh mesh_180(MODEL_4_FILENAME);
    // printMessage("reading ply mesh 5 ...");
    // PlyMesh mesh_270(MODEL_5_FILENAME);
    // printMessage("reading ply mesh 6 ...");
    // PlyMesh mesh_315(MODEL_6_FILENAME);

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
    // mesh_90.writeMesh(MODEL_OUT3_FILENAME);
    // mesh_180.writeMesh(MODEL_OUT4_FILENAME);
    // mesh_270.writeMesh(MODEL_OUT5_FILENAME);
    // mesh_315.writeMesh(MODEL_OUT6_FILENAME);

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
void section6() // multiple body registration
{

}
// ***************************************************************************
void SECTION7() // ICP with normals
{
    printMessage("TASK 7");
    // ************************************************************
    // INIT
    std::string filenameW ("_M1normals_");
    int kNN = 10;
    printMessage("reading ply mesh 1 ...");
    PlyMesh mesh_M1(MODEL_1_FILENAME);

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
    // ICP with normals! (???)




    // ************************************************************
    // Save results
    std::string m1Filename = BASE_PATH + filenameW + EXTENSION;
    mesh_M1.writeMesh(m1Filename);
}

// ***************************************************************************
int main(int argc, const char * argv[])
{
    // SECTION2 ();
    // SECTION3();
    // SECTION4();
    SECTION5();
    // SECTION6();
    // SECTION7();
    // all headers should be defined in the h files, not the corresponding cpp - in the guards




    return 0;
}
