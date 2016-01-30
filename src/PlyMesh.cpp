#include <iostream>
#include  <Eigen/Dense>
#include "PlyMesh.h"
#include "converter.h"
#include "utils.h"


namespace N3dicp
{
    int PlyMesh::meshCount = 0;
    // void PlyMesh::setColour(OpenMesh::Vec3uc colour);
    PlyMesh::PlyMesh(std::string filename){
        if ( ! OpenMesh::IO::read_mesh(m_mesh, filename) )
        {
            std::cerr << "Error: Cannot read mesh from " << filename << std::endl;
        }
        m_vertNum = 0;
        for (MyMesh::VertexIter v_it = m_mesh.vertices_begin();
        v_it != m_mesh.vertices_end(); ++v_it)
        {
            m_vertNum += 1;
        }
        meshCount++;
        OpenMesh::Vec3uc myColour(0,0,0);
        getColourFromList(meshCount,myColour);
        this->setColour(myColour);
    }
    PlyMesh::~PlyMesh()
    {}
        #define MESHOPT( msg, tf ) \
        std::cout << "  " << msg << ": " << ((tf)?"yes\n":"no\n")

        bool PlyMesh::writeMesh(std::string filename){
            OpenMesh::IO::Options wopt;
            wopt += OpenMesh::IO::Options::VertexColor;
            // wopt += OpenMesh::IO::Options::FaceColor;

            // MESHOPT("vertex normals", m_mesh.has_vertex_normals());
            // MESHOPT("vertex colors", m_mesh.has_vertex_colors());
            // MESHOPT("face normals", m_mesh.has_face_normals());
            // MESHOPT("face colors", m_mesh.has_face_colors());

            if ( ! OpenMesh::IO::write_mesh(m_mesh, filename, wopt) )
            {
                std::cerr << "Error: cannot write mesh to " << filename << std::endl;
                return 1;
            }
            return 0;
        }
        void PlyMesh::applyTransformation(Eigen::Matrix4d transfMat)
        {
            for (MyMesh::VertexIter v_it = m_mesh.vertices_begin();
            v_it != m_mesh.vertices_end(); ++v_it)
            {
                // apply the transformation to all the vertices
                Eigen::Vector4d thisCoord = convertOMVecToEIGENVec(m_mesh.point(*v_it )).homogeneous() ;
                Eigen::Vector4d updCoord = transfMat * thisCoord;

                OpenMesh::Vec3d updVertex(updCoord[0],updCoord[1],updCoord[2]);
                m_mesh.set_point( *v_it, updVertex);
            }
        }
        void PlyMesh::setColour(OpenMesh::Vec3uc colour)
        {
            for (MyMesh::VertexIter v_it = m_mesh.vertices_begin();
            v_it != m_mesh.vertices_end(); ++v_it)
            {
                m_mesh.set_color(*v_it,colour);
            }
        }
        void PlyMesh::getVecticesE(Eigen::MatrixXd& outVertMat)
        {
            uint32_t colIdx = 0;

            for (MyMesh::VertexIter v_it = m_mesh.vertices_begin();
            v_it != m_mesh.vertices_end(); ++v_it)
            {
                OpenMesh::Vec3d thisOMVert = m_mesh.point(*v_it);
                Eigen::Vector3d thisEVert = convertOMVecToEIGENVec(thisOMVert);
                // vectors are represented as matrices in Eigen
                // .col = block operation for matrices and arrays
                outVertMat.col(colIdx) = thisEVert;
                colIdx += 1;
            }
        }
        void PlyMesh::setVerticesE(Eigen::MatrixXd newVertPos)
        {
            uint32_t colIdx = 0;
            for (MyMesh::VertexIter v_it = m_mesh.vertices_begin();
            v_it != m_mesh.vertices_end(); ++v_it)
            {
                Eigen::Vector3d thisEVert = newVertPos.col(colIdx);

                OpenMesh::Vec3d thisOMVert = convertEIGENVecToOMVec(thisEVert);
                m_mesh.set_point( *v_it, thisOMVert);
                colIdx += 1;
            }

        }
    }//namespace N3dicp
