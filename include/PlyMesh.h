#ifndef __PLY_CLASS__
#define __PLY_CLASS__

#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
typedef OpenMesh::TriMesh_ArrayKernelT<>  MyMesh;


class PlyMesh
{
public:
	PlyMesh();
	PlyMesh readMesh(std::string filename);
	void writeMesh(std::string filename, PlyMesh& my_mesh);


private:
	
};


#endif


