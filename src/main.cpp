// compile with: gcc -I/usr/local/Cellar/glfw3/3.1.2/include -L/usr/local/Cellar/glfw3/3.1.2/lib/ -framework OpenGL -framework Cocoa -framework IOKit -framework CoreVideo -lglfw3 -o main -L/usr/lib/ -lstdc++ src/main.cpp
// standard headers
#include <iostream>
#include "utils.h"
//#include <vector>
// OpenGL headers
//#include <GL/glew.h>
//#include <GLFW/glfw3.h>
#include "PlyMesh.h"
using namespace N3dicp;

static const int MESH_NUMBER = 1;
static const char* MODEL_1_FILENAME = "../src/_bun000.ply";
static const char* MODEL_OUT_FILENAME = "../src/_bun000_out.ply";

int main(int argc, const char * argv[])
{

    // insert code here...
    printMessage("***");

    printMessage("reading ply mesh ...");
    PlyMesh mesh(MODEL_1_FILENAME);
    
    printMessage("writing ply mesh ...");
    mesh.writeMesh(MODEL_OUT_FILENAME);

    printMessage("***");
    return 0;
}
