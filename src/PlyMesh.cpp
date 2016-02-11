#include <iostream>
#include  <Eigen/Dense>
#include "PlyMesh.h"
#include "converter.h"
#include "utils.h"
#include <random>


namespace N3dicp
{
    int PlyMesh::meshCount = 0;
    // ***************************************************************************
    PlyMesh::PlyMesh(std::string filename){
        if ( ! OpenMesh::IO::read_mesh(m_mesh, filename) )
        {
            std::cerr << "Error: Cannot read mesh from " << filename << std::endl;
        }
        m_mesh.release_vertex_normals();
        m_mesh.release_face_normals();

        if(m_mesh.has_vertex_normals())
        {
            printMessage("READ - YES - normals");
            //     // delete any mesh normals!
            //     m_mesh.release_vertex_normals();
        }
        else
        {
            printMessage("READ - NO - normals");
        }
        m_vertNum = 0;
        for (MyMesh::VertexIter v_it = m_mesh.vertices_begin();
        v_it != m_mesh.vertices_end(); ++v_it)
        {
            m_vertNum += 1;
        }
        meshCount++;
        std::cout << "Explicit constr - meshCount = " << meshCount << "\n";

        OpenMesh::Vec3uc myColour(0,0,0);
        getColourFromList(meshCount,myColour);
        this->setColour(myColour);
    }
    PlyMesh::PlyMesh(Eigen::MatrixXd& pPointCloud)
    {
        m_vertNum = pPointCloud.cols();
        // MyMesh::VertexHandle* vhandle = new MyMesh::VertexHandle[m_vertNum];

        for (int i = 0; i < m_vertNum; i++)
        {
            OpenMesh::Vec3d thisVert = convertEIGENVecToOMVec(pPointCloud.col(i));
            m_mesh.add_vertex(thisVert);
        }

        // delete[] vhandle;

        meshCount++;
        std::cout << "Explicit constr - meshCount = " << meshCount << "\n";
        OpenMesh::Vec3uc myColour(0,0,0);
        getColourFromList(meshCount,myColour);
        this->setColour(myColour);
    }
    // ***************************************************************************
    PlyMesh::PlyMesh(const PlyMesh& pSrc)
    {
        m_mesh = pSrc.m_mesh;
        m_vertNum = pSrc.m_vertNum;

        meshCount++;
        std::cout << "Copy constr - meshCount = " << meshCount << "\n";
        OpenMesh::Vec3uc myColour(0,0,0);
        getColourFromList(meshCount,myColour);
        this->setColour(myColour);

    }
    // ***************************************************************************
    PlyMesh& PlyMesh::operator= (const PlyMesh& pSrc)
    {
        // check for self - assignment
        if (this == &pSrc)
        return *this;

        m_mesh = pSrc.m_mesh;
        m_vertNum = pSrc.m_vertNum;

        meshCount++;
        std::cout << "= operator - meshCount = " << meshCount << "\n";

        OpenMesh::Vec3uc myColour(0,0,0);
        getColourFromList(meshCount,myColour);
        this->setColour(myColour);
        // return the existing object
        return *this;
    }
    // ***************************************************************************
    PlyMesh::~PlyMesh()
    {

    }
    // ***************************************************************************
    // #define MESHOPT( msg, tf ) \
    // std::cout << "  " << msg << ": " << ((tf)?"yes\n":"no\n")
    bool PlyMesh::writeMesh(std::string filename){
        OpenMesh::IO::Options wopt;
        wopt += OpenMesh::IO::Options::VertexColor;
        if (m_mesh.has_vertex_normals())
        {
            wopt += OpenMesh::IO::Options::VertexNormal;
            // wopt += OpenMesh::IO::Options::FaceColor;
        }


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
    // ***************************************************************************
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
    // ***************************************************************************
    void PlyMesh::setNormals(Eigen::MatrixXd& pNormals)
    {
        m_mesh.request_vertex_normals();

        uint32_t colIdx = 0;
        for (MyMesh::VertexIter v_it = m_mesh.vertices_begin();
        v_it != m_mesh.vertices_end(); ++v_it)
        {
            Eigen::Vector3d thisNormal = pNormals.col(colIdx);
            thisNormal.normalize();
            // Eigen::Vector3d thisNormal(1,1,1);
            // if(colIdx < 10000)
            //     thisNormal.col(0) = Eigen::Vector3d::Zero();

            OpenMesh::Vec3d thisOMNormal = convertEIGENVecToOMVec(thisNormal);
            // m_mesh.set_point( *v_it, thisOMVert);
            m_mesh.set_normal( *v_it, thisOMNormal);
            // TO DO: set vertex colour based on normal direction
            colIdx += 1;
            //change direction of normals! some of them are inverted
        }
        if(m_mesh.has_vertex_normals())
        {
            // delete any mesh normals!
            printMessage("SET N - YES - normals");
        }
        else
        {
            printMessage("SET N - NO - normals");
        }
    }
    // ***************************************************************************
    void PlyMesh::addNoise(double noiseStd)
    {
        double noiseMean = 0.0;
        std::default_random_engine generator;
        std::normal_distribution<double> gaussNoise(noiseMean,noiseStd);
        double thisRandomNoise;
        // add random noise to each vertex
        for (MyMesh::VertexIter v_it = m_mesh.vertices_begin();
        v_it != m_mesh.vertices_end(); ++v_it)
        {
            thisRandomNoise = gaussNoise(generator);
            OpenMesh::Vec3d thisVNoise(thisRandomNoise,thisRandomNoise,thisRandomNoise);

            OpenMesh::Vec3d thisOMVert = m_mesh.point(*v_it);
            thisOMVert = thisOMVert + thisVNoise;
            m_mesh.set_point( *v_it, thisOMVert);
        }
    }
    // ***************************************************************************
    void PlyMesh::setColour(OpenMesh::Vec3uc colour)
    {
        for (MyMesh::VertexIter v_it = m_mesh.vertices_begin();
        v_it != m_mesh.vertices_end(); ++v_it)
        {
            m_mesh.set_color(*v_it,colour);
        }
    }
    // ***************************************************************************
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
    // ***************************************************************************
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
    // ***************************************************************************
    // void PlyMesh::subsample(float degree)
    // {
    // m_mesh.request_face_status();
    // m_mesh.request_edge_status();
    // m_mesh.request_vertex_status();
    // for (MyMesh::FaceIter v_it = m_mesh.faces_sbegin();
    // v_it != m_mesh.faces_end(); ++v_it)
    // {
    //     m_mesh.delete_face(*v_it, true);
    // }
    // update each vertex with the location of the face center + delete the other vertices
    // update m_mesh & m_vertNum
    // }
}//namespace N3dicp
