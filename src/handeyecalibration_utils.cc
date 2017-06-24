#include "handeyecalibration_utils.h"
#include<ceres/ceres.h>
#include<Eigen/SVD>

using Eigen::Map;

Eigen::Matrix3d skew(Eigen::Vector3d u)
{
    Eigen::Matrix3d u_hat = Eigen::MatrixXd::Zero(3,3);
    u_hat(0,1) = u(2);
    u_hat(1,0) = -u(2);
    u_hat(0,2) = -u(1);
    u_hat(2,0) = u(1);
    u_hat(1,2) = u(0);
    u_hat(2,1) = -u(0);

    return u_hat;
}

template<typename T>
Eigen::Matrix<T,4,4> Rt2hom(Eigen::Matrix<T, 3, 3> R, Eigen::Matrix<T, 3, 1> t)
{
    Eigen::Matrix<T,4,4> trans_hom = Eigen::Matrix<T,4,4>::Identity(4,4);
    trans_hom.topLeftCorner(3,3) = R;
    trans_hom.topRightCorner(3,1) = t;

    return trans_hom;
}

Pose Rt2hom(Eigen::Matrix3d R, Eigen::Vector3d t)
{
    Pose trans_hom = Pose::Identity(4,4);
    trans_hom.topLeftCorner(3,3) = R;
    trans_hom.topRightCorner(3,1) = t;

    return trans_hom;
}

void SetCameraPosesFromHandPoses(const Poses& handposes,
                                 HandEyeTransformation* handeyetrans, Reconstruction* reconstruction)
{

    Eigen::Matrix3d handeyerotation = handeyetrans->GetHandEyeRotationAsRotationMatrix();
    Eigen::Vector3d handeyetranslation = handeyetrans->GetHandEyeTranslation();

    for (int i=0; i<handposes.size(); i++)
    {
        View* view = reconstruction->MutableView(i);
        if (view == nullptr)
        {
            LOG(WARNING) << "Trying to set the pose of View " << i
                         << " which does not exist in the reconstruction.";
            continue;
        }
        //    CHECK(!view->IsEstimated()) << "Cannot set the pose of a view that has "
        //                                   "already been estimated. View Id "
        //                                << i;
        //Pose camerapose = handeyetrans*handposes[i];

        Eigen::Matrix3d handorientation = handposes[i].topLeftCorner(3,3);
        Eigen::Vector3d handeyeposition = handposes[i].topRightCorner(3,1);

        view->MutableCamera()->SetPosition(handeyeposition-handorientation*handeyerotation.transpose()*handeyetranslation);
        view->MutableCamera()->SetOrientationFromRotationMatrix(handeyerotation*handorientation.transpose());
        //    view->MutableCamera()->SetOrientationFromRotationMatrix(handorientation*handeyerotation.transpose());
        view->SetEstimated(true);
    }
}

void SetCameraPoseFromHandPose(View* view, Pose handpose,
                               HandEyeTransformation* handeyetrans)
{

    if (view == nullptr)
    {
        LOG(WARNING) << "Trying to set the pose of View which does not exist";
        return;
    }
    //    CHECK(!view->IsEstimated()) << "Cannot set the pose of a view that has "
    //                                   "already been estimated.";

    Eigen::Matrix3d handeyerotation = handeyetrans->GetHandEyeRotationAsRotationMatrix();
    Eigen::Vector3d handeyetranslation = handeyetrans->GetHandEyeTranslation();
    Eigen::Matrix3d handorientation = handpose.topLeftCorner(3,3);
    Eigen::Vector3d handeyeposition = handpose.topRightCorner(3,1);

    view->MutableCamera()->SetPosition(handeyeposition-handorientation*handeyerotation.transpose()*handeyetranslation);
    view->MutableCamera()->SetOrientationFromRotationMatrix(handeyerotation*handorientation.transpose());
    view->SetEstimated(true);
}

template<typename T>
void pose2array(const Eigen::Matrix<T, 4, 4> &pose, T *ar)
{
    ceres::RotationMatrixToAngleAxis(
        ceres::ColumnMajorAdapter3x3(pose.topLeftCorner(3,3).data()),
        ar );
    Map<Eigen::Matrix<T,3,1>>(ar + 3) = pose.topRightCorner(3,1);
}

double L2Norm(Eigen::MatrixXd m)
{
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(m);
    return (svd.singularValues())(0);
}














