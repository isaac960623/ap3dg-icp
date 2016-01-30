// compile with: gcc -I/usr/local/Cellar/glfw3/3.1.2/include -L/usr/local/Cellar/glfw3/3.1.2/lib/ -framework OpenGL -framework Cocoa -framework IOKit -framework CoreVideo -lglfw3 -o main -L/usr/lib/ -lstdc++ src/main.cpp
// standard headers
#include <iostream>
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


static const int MESH_NUMBER = 2;
static const char* MODEL_1_FILENAME = "../src/_bun000.ply";
static const char* MODEL_2_FILENAME = "../src/_bun045.ply";
static const char* MODEL_3_FILENAME = "../src/_bun090.ply";
static const char* MODEL_4_FILENAME = "../src/_bun180.ply";
static const char* MODEL_5_FILENAME = "../src/_bun270.ply";
static const char* MODEL_6_FILENAME = "../src/_bun315.ply";
static const char* MODEL_OUT1_FILENAME = "../src/_bun000_init.ply";
static const char* MODEL_OUT2_INIT_FILENAME = "../src/_bun045_init.ply";
static const char* MODEL_OUT2_FIN_FILENAME = "../src/_bun045_fin.ply";
static const char* MODEL_OUT3_FILENAME = "../src/_bun090_init.ply";
static const char* MODEL_OUT4_FILENAME = "../src/_bun180_init.ply";
static const char* MODEL_OUT5_FILENAME = "../src/_bun270_init.ply";
static const char* MODEL_OUT6_FILENAME = "../src/_bun315_init.ply";

// ***************************************************************************
static const int ICP_MAXITER = 5;
static const double ICP_MAXERR = 0.005;



void TASK2 (){
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
    applyICP(p, q, final_q,ICP_MAXITER, ICP_MAXERR);

    mesh_45.writeMesh(MODEL_OUT2_INIT_FILENAME);
    // ************************************************************
    // Save results
    mesh_45.setVerticesE(final_q);

    // printMessage("writing ply meshes...");
    // mesh_00.writeMesh(MODEL_OUT1_FILENAME);
    mesh_45.writeMesh(MODEL_OUT2_FIN_FILENAME);
    // mesh_90.writeMesh(MODEL_OUT3_FILENAME);
    // mesh_180.writeMesh(MODEL_OUT4_FILENAME);
    // mesh_270.writeMesh(MODEL_OUT5_FILENAME);
    // mesh_315.writeMesh(MODEL_OUT6_FILENAME);

    printMessage("***");
}

int main(int argc, const char * argv[])
{
    TASK2 ();

    return 0;
}
