#ifndef __PLY_CLASS__
#define __PLY_CLASS__

#include <Eigen/Dense>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>

namespace N3dicp
{
    struct MyTraits: public OpenMesh::DefaultTraits
    {
        typedef OpenMesh::Vec3d Point;
        typedef OpenMesh::Vec3d Normal;
        VertexAttributes( OpenMesh::Attributes::Normal |
            OpenMesh::Attributes::Color );
        FaceAttributes( OpenMesh::Attributes::Normal | OpenMesh::Attributes::Color );
    };
    typedef OpenMesh::TriMesh_ArrayKernelT<MyTraits>  MyMesh;

    class PlyMesh
    {
    public:
        static int meshCount;
        PlyMesh(std::string filename);
        ~PlyMesh();
        bool writeMesh(std::string filename);
        void applyTransformation(Eigen::Matrix4d transfMat);
        // demean vertices
        //set colour
        void setColour(OpenMesh::Vec3uc colour);
        // get vertices
        void getVecticesE(Eigen::MatrixXd& outVertMat);
        // set vertices
        void setVerticesE(Eigen::MatrixXd newVertPos);
        uint32_t getTotalVertices() { return m_vertNum;}

    private:
        MyMesh m_mesh;
        uint32_t m_vertNum;
    };

}//namespace N3dicp
#endif
