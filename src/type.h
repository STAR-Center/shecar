#ifndef TYPE_H
#define TYPE_H

#include<Eigen/Core>

typedef Eigen::Matrix4d Pose;
//typedef std::pair<Eigen::Matrix3d, Eigen::Vector3d> Pose;
typedef std::vector<Pose> Poses;

#endif // TYPE_H
