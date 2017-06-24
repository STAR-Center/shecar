#ifndef AXXBSOLVER_H
#define AXXBSOLVER_H

#include<Eigen/Core>
#include"../type.h"

//used for hand eye calibration

class AXXBSolver
{
public:
    AXXBSolver();
    AXXBSolver(const Poses A, const Poses B):A_(A),B_(B) {}

    virtual Pose SolveX()=0;

    Poses A_;
    Poses B_;
};

#endif // AXXBSOLVER_H
