#ifndef HANDEYECALIBRATION_UTILS_H
#define HANDEYECALIBRATION_UTILS_H

#include<Eigen/Core>
#include"type.h"
#include <theia/theia.h>
#include "handeyetransformation.h"
using namespace theia;

Eigen::Matrix3d skew(Eigen::Vector3d u);

template<typename T>
Eigen::Matrix<T,4,4> Rt2hom(Eigen::Matrix<T, 3, 3> R, Eigen::Matrix<T, 3, 1> t);

Pose Rt2hom(Eigen::Matrix3d R, Eigen::Vector3d t);

void SetCameraPosesFromHandPoses(const Poses& handposes,
                                 HandEyeTransformation* handeyetrans, Reconstruction* reconstruction);

void SetCameraPoseFromHandPose(View *view, Pose handpose, HandEyeTransformation* handeyetrans);

template<typename T> void pose2array(const Eigen::Matrix<T,4,4> &, T *ar);

double L2Norm(Eigen::MatrixXd m);

#endif // HANDEYECALIBRATION_UTILS_H
