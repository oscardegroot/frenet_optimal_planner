#ifndef POLYNOMIALS_H_
#define POLYNOMIALS_H_

#include "Eigen/Dense"

using Eigen::VectorXd;

namespace fop
{

// Evaluate a polynomial.
double polyeval(const VectorXd &coeffs, double x);
// Fit a polynomial.
VectorXd polyfit(const VectorXd &xvals, const VectorXd &yvals, int order);

} // end of namespace fop

#endif // POLYNOMIALS_H_