#ifndef BSPLINE_H
#define BSPLINE_H

#include "gltypes.h"
#include <vector>
#include "vec3.h"

using namespace gsl;


class BSpline
{
public:
    BSpline();

    unsigned int getIntervalFromTime(float x, std::vector<float> &knots);
    Vec3 evaluate (unsigned int k, unsigned int degree, unsigned int i, float x, std::vector<float> &knots, std::vector<gsl::Vec3>& controlPoints);
};

#endif // BSPLINE_H
