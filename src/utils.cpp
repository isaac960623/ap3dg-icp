#include "utils.h"
#include <iostream>

namespace N3dicp
{
    void printMessage(std::string text)
    {
        std::cout<< text << std::endl;
    }
    Eigen::Matrix4d loadTransformation(int transfNo)
    {
        switch (transfNo) {
            case 1:
            Eigen::Matrix4d initTransf;
            initTransf << 0.859356, 0.000000, 0.511377, -0.053997,
            0.000000, 1.000000, 0.000000, 0.003653,
            -0.511377, 0.000000, 0.859356, -0.005487,
            0.000000, 0.000000, 0.000000, 1.000000;

            // initTransf << 0.824503, 0.008312, 0.565796, -0.053328,
            // -0.010281, 0.999947, 0.000292, -0.000592,
            // -0.565764, -0.006057, 0.824545, -0.010384,
            // 0.000000, 0.000000, 0.000000, 1.000000;
            //
            return initTransf;


        }
        return Eigen::Matrix4d::Identity();
    }
    void getColourFromList(int idx,OpenMesh::Vec3uc& outColour)
    {
        OpenMesh::Vec3uc purple(92,75,81);
        OpenMesh::Vec3uc lightBlue(140,190,178);
        OpenMesh::Vec3uc lightYellow(242,235,191);
        OpenMesh::Vec3uc lightOrange(243,181,98);
        OpenMesh::Vec3uc lightPink(240,96,96);
        OpenMesh::Vec3uc otherBlue(95,172,190);

        if (idx == 1 ) outColour = purple;
        if (idx == 2 ) outColour = lightBlue;
        if (idx == 3 ) outColour = lightYellow;
        if (idx == 4 ) outColour = lightOrange;
        if (idx == 5 ) outColour = lightPink;
        if (idx == 6 ) outColour = otherBlue;
    }

}//namespace N3dicp
