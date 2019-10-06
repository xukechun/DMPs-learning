/*********************************************************************
DMP created in 2019.09
 *********************************************************************/

#ifndef LINEAR_APPROX_H_
#define LINEAR_APPROX_H_

#include "dmp/function_approx.h"
#include <iostream>

namespace dmp{

typedef std::pair<double, double> pt_pair;

      
/// Class for function approximation by recording points and linearly interpolating
class LinearApprox : public FunctionApprox{
public:
	LinearApprox();
        LinearApprox(std::vector<double> X, std::vector<double> Y);
	virtual ~LinearApprox();

	/**\brief Evaluate the function approximator at point x
	 * \param x The point at which to evaluate
	 * \return The scalar value of the function at x
	 */
	virtual double evalAt(double x);

	/**\brief Computes the least squares weights given a set of data points
	 * \param X A vector of the domain values of the points
	 * \param Y A vector of the target values of the points
	 */
	virtual void leastSquaresWeights(double *X, double *Y, int n_pts);

        
private:
 
        std::vector<pt_pair> points;

};

}

#endif /* LINEAR_APPROX_H_ */
