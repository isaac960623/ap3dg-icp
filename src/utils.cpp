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
        Eigen::Matrix4d initTransf;
        switch (transfNo) {
            case 1:
            initTransf << 0.816588, 0.011159, 0.577113, -0.056024,
            -0.017766, 0.999825, 0.005807, -0.001257,
            -0.576947, -0.014995, 0.816644, -0.009324,
            0.000000, 0.000000, 0.000000, 1.000000;

            // case 2:
            // initTransf << -0.027151, -0.023013, 0.999366, 0.002352,
            // 0.010032, 0.999678, 0.023293, -0.001107,
            // -0.999581, 0.010658, -0.026911, -0.001844,
            // 0.000000, 0.000000, 0.000000, 1.000000;

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

        outColour = otherBlue;
        if (idx == 1 ) outColour = purple;
        if (idx == 2 ) outColour = lightBlue;
        if (idx == 3 ) outColour = lightYellow;
        if (idx == 4 ) outColour = lightOrange;
        if (idx == 5 ) outColour = lightPink;
        if (idx == 6 ) outColour = otherBlue;
    }
    void createRotationMatrix (float rotX, float rotY, float rotZ, Eigen::Matrix3d& rotMat)
    {
        // good tutorial: http://www.gamasutra.com/view/feature/131686/rotating_objects_using_quaternions.php
        //v´ = q v q-1 (where v = [0, v]), q = quaternion, v = arbitrary vector
        rotMat = Eigen::AngleAxisd(rotX*M_PI, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(rotY*M_PI,  Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(rotZ*M_PI, Eigen::Vector3d::UnitZ());
        // std::cout << rotMat << std::endl << "is unitary: " << rotMat.isUnitary() << std::endl;
    }

}//namespace N3dicp
