#ifndef __PLY_CLASS__
#define __PLY_CLASS__

#include <Eigen/Dense>
#include "ANN/ANN.h"
#include "icp.h"
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
        // PlyMesh();
        PlyMesh(std::string filename);
        PlyMesh(Eigen::MatrixXd& pPointCloud);
        PlyMesh(const PlyMesh&);
        ~PlyMesh();
        PlyMesh& operator= (const PlyMesh &pSrc);
        bool writeMesh(std::string filename);
        void setNormals(Eigen::MatrixXd& pNormals);
        void applyTransformation(Eigen::Matrix4d transfMat);
        void addNoise(double noiseStd);
        void setColour(OpenMesh::Vec3uc colour);
        void getVecticesE(Eigen::MatrixXd& outVertMat);
        void setVerticesE(Eigen::MatrixXd newVertPos);
        void subsample(float degree = 0.5);
        uint32_t getTotalVertices() { return m_vertNum;}

    private:
        MyMesh m_mesh;
        uint32_t m_vertNum;
    };

}//namespace N3dicp
#endif
