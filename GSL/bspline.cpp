#include "bspline.h"

BSpline::BSpline()
{

}

unsigned int BSpline::getIntervalFromTime(float x, std::vector<float> &knots)
{
    for(unsigned int i = 0; i < knots.size(); i++)
    {
        if(x >= knots[i] && x < knots[i+1])
        {
            return i;
        }
    }
}

Vec3 BSpline::evaluate(unsigned int k, unsigned int degree, unsigned int i, float x, std::vector<float> &knots, std::vector<Vec3> &controlPoints)
{
    if(k == 0)
    {
        return controlPoints[i];
    }
    else
    {
        float alpha = (x-knots[i])/(knots[i + degree + 1 - k] - knots[i]);
        return(evaluate(k-1, degree, i-1, x, knots, controlPoints)*(1-alpha)+evaluate(k-1, degree, i, x, knots, controlPoints)*alpha);
    }
}
