#ifndef AXXBSVDSOLVER_H
#define AXXBSVDSOLVER_H

#include"axxbsolver.h"
class AXXBSVDSolver:public AXXBSolver
{
public:
    AXXBSVDSolver(const Poses A, const Poses B):AXXBSolver(A,B) {}
    Pose SolveX();
};

#endif // AXXBSVDSOLVER_H
