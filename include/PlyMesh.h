#ifndef __PLY_CLASS__
#define __PLY_CLASS__

#include <iostream>
#include <vector>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

namespace N3dicp
{
    struct MyTraits: public OpenMesh::DefaultTraits
    {
        typedef OpenMesh::Vec3d Point;
        VertexAttributes( OpenMesh::Attributes::Normal |
            OpenMesh::Attributes::Color );
            FaceAttributes( OpenMesh::Attributes::Normal );
    };

    class PlyMesh
    {
    public:
        PlyMesh(std::string filename);
        bool writeMesh(std::string filename);

    private:
        typedef OpenMesh::TriMesh_ArrayKernelT<MyTraits>  MyMesh;
        MyMesh m_mesh;

    };
}//namespace N3dicp
#endif
