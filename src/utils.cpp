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
        Eigen::Matrix4d initTransf (Eigen::Matrix4d::Identity());

        Eigen::Matrix4d initTransf45_0 (Eigen::Matrix4d::Identity());
        Eigen::Matrix4d initTransf90_0 (Eigen::Matrix4d::Identity());
        Eigen::Matrix4d initTransf180_0 (Eigen::Matrix4d::Identity());

        initTransf45_0 << 0.707013, 0.000000, 0.707200, 0.000000,
        0.000000, 1.000000, 0.000000, 0.000000,
        -0.707200, 0.000000, 0.707013, 0.000000,
        0.000000, 0.000000, 0.000000, 1.000000;

        initTransf90_0 << -0.009378, 0.000000, 0.999956, 0.000000,
        0.000000, 1.000000, 0.000000, 0.000000,
        -0.999956, 0.000000, -0.009378, 0.000000,
        0.000000, 0.000000, 0.000000, 1.000000;

        initTransf180_0 << -0.999929, 0.000000, -0.011915, 0.000000,
        0.000000, 1.000000, 0.000000, 0.000000,
        0.011915, 0.000000, -0.999929, 0.000000,
        0.000000, 0.000000, 0.000000, 1.000000;


        if(transfNo == 1) { // 00 - 45
            initTransf << 0.696072, 0.000000, 0.717972, -0.049942,
            0.000000, 1.000000, 0.000000, 0.000000,
            -0.717972, 0.000000, 0.696072, 0.000000,
            0.000000, 0.000000, 0.000000, 1.000000;
        }
        if(transfNo == 2) { // 90
            initTransf = initTransf90_0;
        }
        if(transfNo == 3) {
            initTransf = initTransf180_0;
        }
        if(transfNo == 4) {
            initTransf = initTransf180_0 * initTransf90_0;
        }
        if(transfNo == 5) {
            initTransf = initTransf45_0 * initTransf180_0 * initTransf90_0;
        }
        if(transfNo == 6) { // 45
            initTransf = initTransf45_0;
        }
        return initTransf;

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
