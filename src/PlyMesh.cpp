#include <iostream>
#include "PlyMesh.h"

namespace N3dicp
{
    PlyMesh::PlyMesh(std::string filename){
        if ( ! OpenMesh::IO::read_mesh(m_mesh, filename) )
        {
            std::cerr << "Error: Cannot read mesh from " << filename << std::endl;
        }
    }

    bool PlyMesh::writeMesh(std::string filename){
        if ( ! OpenMesh::IO::write_mesh(m_mesh, filename) )
        {
            std::cerr << "Error: cannot write mesh to " << filename << std::endl;
            return 1;
        }
        return 0;
    }
}//namespace N3dicp
