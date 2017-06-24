#ifndef AXXBESTIMATOR_H
#define AXXBESTIMATOR_H
#include "../type.h"
#include <theia/theia.h>
#include"axxbsvdsolver.h"
#include "../handeyecalibration_utils.h"

using namespace theia;

// Our "data".
struct MotionPair
{
    Pose A;
    Pose B;
    MotionPair() {}
    MotionPair(Pose a,Pose b):A(a),B(b) {}
};

// Our "model".
//struct Transformation {
//  Pose trans;
//};

// Estimator class.
class AXXBEstimator: public Estimator<MotionPair, Pose>
{
public:
    AXXBEstimator() {}

    ~AXXBEstimator() {}
    // Number of transformation pairs needed to estimate a hand-eye transformation.
    double SampleSize() const
    {
        return 2;
    }

    // Estimate hand-eye transformation from ten pairs of hand and eye motions.
    bool EstimateModel(const std::vector<MotionPair>& data,
                       std::vector<Pose>* models) const
    {
        Poses A,B;
        for(MotionPair AB : data)
        {
            A.push_back(AB.A);
            B.push_back(AB.B);
        }

        AXXBSVDSolver axxbsolver = AXXBSVDSolver(A,B);
        Pose x = axxbsolver.SolveX();
        models->push_back(x);
        return true;
    }

    // Calculate the error.
    double Error(const MotionPair& motionpair, const Pose& trans) const
    {
        // AX=XB, so A=XBX-1
        Pose A_predict = trans*motionpair.B*trans.inverse();
        double error;
        // rotation error, using the rotation matrix difference
        //    Eigen::Matrix3d tmp = A_predict.topLeftCorner(3,3)-motionpair.A.topLeftCorner(3,3);
        //    Eigen::JacobiSVD<Eigen::MatrixXd> svd( tmp);
        //    error = (svd.singularValues())(0);
        error = L2Norm(A_predict.topLeftCorner(3,3)-motionpair.A.topLeftCorner(3,3));
        // as there is an unknown scale factor, so the error is
        // min(||lambda*A_translation-A_translation_predict||^2)
        // = ||A_translation||^2*lambda^2-2*dot(A_translation,A_translation_predict)*lambda+||A_translation_predict||^2
        // this is a quadratic function to lambda, its minimal is
        // ||A_translation_predict||^2 -  dot(A_translation,A_translation_predict)^2/||A_translation||^2
        Eigen::Vector3d A_translation_predict = A_predict.topRightCorner(3,1);
        Eigen::Vector3d A_translation = motionpair.A.topRightCorner(3,1);
        double a = A_translation.squaredNorm();
        double b = A_translation_predict.dot(A_translation);
        double c = A_translation_predict.squaredNorm();
        error += 0.1*std::sqrt(c - b*b/a);
        return error;
    }
};

#endif // AXXBESTIMATOR_H
