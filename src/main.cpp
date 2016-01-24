// compile with: gcc -I/usr/local/Cellar/glfw3/3.1.2/include -L/usr/local/Cellar/glfw3/3.1.2/lib/ -framework OpenGL -framework Cocoa -framework IOKit -framework CoreVideo -lglfw3 -o main -L/usr/lib/ -lstdc++ src/main.cpp
// standard headers
#include <iostream>
#include <stdio.h>
// OpenGL headers
//#include <GL/glew.h>
#include <GLFW/glfw3.h>
// -------------------- OpenMesh
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

typedef OpenMesh::TriMesh_ArrayKernelT<>  MyMesh;
// #include <OpenGL/gl.h>
// #include <OpenGl/glu.h>


int main(int argc, const char * argv[]) {
    // insert code here...
    std::cout << "My first OpenGL victory!\n";
    
    
    GLFWwindow* window;
    
    /* Initialize the library */
    if (!glfwInit())
        return -1;
    
    /* Create a windowed mode window and its OpenGL context */
    window = glfwCreateWindow(640, 480, "Window to my soul", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        return -1;
    }
    
    /* Make the window's context current */
    glfwMakeContextCurrent(window);
    
    /* Loop until the user closes the window */
    while (!glfwWindowShouldClose(window))
    {
        /* Render here */

        // http://stackoverflow.com/questions/27678819/crazy-flashing-window-opengl-glfw
        // stop the flickering - it's basically updating from uninitialized 'framebuffer' memory, which the GL implementation has probably used for other purposes, like backing store, textures
        glClear(GL_COLOR_BUFFER_BIT);
        
         // Swap front and back buffers 
        glfwSwapBuffers(window);
        
        /* Poll for and process events */
        glfwPollEvents();
    }
    
    glfwTerminate();
    
    
    
    
    return 0;
}
