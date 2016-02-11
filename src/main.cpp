// compile with: gcc -I/usr/local/Cellar/glfw3/3.1.2/include -L/usr/local/Cellar/glfw3/3.1.2/lib/ -framework OpenGL -framework Cocoa -framework IOKit -framework CoreVideo -lglfw3 -o main -L/usr/lib/ -lstdc++ src/main.cpp
// standard headers
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
static const int S2_ICP_MAXITER = 20;
static const double S2_ICP_MAXERR = 0.2;


// ***************************************************************************
// SECTION 3
static const int S3_NUM_ROTATIONS = 5;
static const int S3_MIN_ANGLE_DEGREES = 5;
static const int S3_MAX_ANGLE_DEGREES = 30;

// ***************************************************************************
// SECTION 4



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
    applyICP(p, q, final_q, S2_ICP_MAXITER, S2_ICP_MAXERR);

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

    std::string filenameW("_M3rotated_");

    Eigen::Matrix3d rotationMat(Eigen::Matrix3d::Zero());
    Eigen::Matrix4d transformationMat(Eigen::Matrix4d::Zero());
    transformationMat(3,3) = 1;

    anglesDegree = Eigen::VectorXd::LinSpaced(S3_NUM_ROTATIONS, S3_MIN_ANGLE_DEGREES, S3_MAX_ANGLE_DEGREES);

    anglesRadians = (M_PI / 180 * anglesDegree.array()).matrix();

    // Read mesh M1 and init M3

    printMessage("reading ply mesh 1 ...");
    PlyMesh mesh_M1(MODEL_1_FILENAME);

    // ************************************************************
    // Generate rotations
    for( int i = 0; i < S3_NUM_ROTATIONS; i ++)
    {
        std::cout << "*** angle = " << anglesRadians[i] << "\n";
        createRotationMatrixQuat (anglesRadians[i], rotY, rotZ, rotationMat);

        transformationMat.block<3,3>(0,0) = rotationMat;//.data();
        std::cout << "*** transfMat idx " << i << " :\n" << transformationMat << "\n\n";
        // ************************************************************
        // Apply transformation to the mesh and run ICP
        PlyMesh mesh_M3(mesh_M1); // this creates a new object - why? OpenMesh magic
        mesh_M3.applyTransformation(transformationMat);

        // ************************************************************
        // Save iteration
        std::string iterationNo(std::to_string(i));
        std::string m3Filename = BASE_PATH + filenameW + iterationNo + EXTENSION;
        // std::string filen1("_M1_aftercopy");
        // std::string m1Filename = BASE_PATH + filen1 + iterationNo + EXTENSION;
        // mesh_M1.writeMesh(m1Filename);
        mesh_M3.writeMesh(m3Filename);

        // run ICP between M1 and M3
    }
}
void SECTION4() // ICP with add random noise
{
    printMessage("TASK 4");
    // ************************************************************
    // INIT
    int i = 1;
    std::string filenameW("_M1noisy_");
    double noiseStd = 0.001;
    int kNN = 15;
    printMessage("reading ply mesh 1 ...");
    PlyMesh mesh_M1(MODEL_1_FILENAME);
    PlyMesh mesh_M2(mesh_M1);

    // ************************************************************
    mesh_M2.addNoise(noiseStd);

    // ************************************************************
    // run ICP analysis

    // ************************************************************
    // Save results
    std::string iterationNo(std::to_string(i));
    std::string m1Filename = BASE_PATH + filenameW + iterationNo + EXTENSION;
    mesh_M2.writeMesh(m1Filename);




}
void SECTION5() // ICP with subsampling
{
    printMessage("TASK 5");
    // ************************************************************
    // INIT
    std::string filenameW ("_M2subsampled_");
    float subsamplingRate = 30; // % - between 0 and 100
    printMessage("reading ply mesh 1 ...");
    PlyMesh mesh_M1(MODEL_1_FILENAME);
    PlyMesh mesh_M2(MODEL_2_FILENAME);


    Eigen::MatrixXd p(3,mesh_M2.getTotalVertices());
    mesh_M2.getVecticesE(p);

    long numPoints = p.cols();
    long newNumPoints = (long)((subsamplingRate/100)*numPoints);
    Eigen::MatrixXd pSubsampled(3,newNumPoints);
    subsample(p, pSubsampled, subsamplingRate);

    PlyMesh mesh_M2_ss(pSubsampled);

    // ************************************************************
    // ICP magic


    // ************************************************************
    // Save results
    std::string m1Filename = BASE_PATH + filenameW + EXTENSION;
    mesh_M2_ss.writeMesh(m1Filename);
}
void section6() // multiple body registration
{

}
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

int main(int argc, const char * argv[])
{
    SECTION2 ();
    // SECTION3();
    // SECTION4();
    // SECTION5();
    // SECTION6();
    // SECTION7();
    // all headers should be defined in the h files, not the corresponding cpp - in the guards




    return 0;
}
