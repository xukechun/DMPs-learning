/*********************************************************************
DMP created in 2019.09
 *********************************************************************/


#ifndef FUNCTION_APPROX_H_
#define FUNCTION_APPROX_H_

#include "ros/ros.h"


namespace dmp{

/// Interface for univariate linear function approximation
class FunctionApprox {
public:
	FunctionApprox(){};
	virtual ~FunctionApprox(){};

	/**\brief Evaluate the function approximator at point x
	 * \param x The point at which to evaluate
	 * \return The scalar value of the function at x
	 */
	virtual double evalAt(double x) = 0;

	/**\brief Computes the least squares weights given a set of data points
	 * \param X A vector of the domain values of the points
	 * \param Y A vector of the target values of the points
	 */
	virtual void leastSquaresWeights(double *X, double *Y, int n_pts) = 0;

	/**\brief Returns the number of basis functions
	 * \return The number of basis functions used by the approximator
	 */
	int getNumBases(){return n_bases;}

	/**\brief Returns the current weight vector
	 * \return The current weight vector
	 */
	std::vector<double> getWeights(){return weights;}

protected:
	int n_bases;					//The number of bases in the approximator
	std::vector<double> weights;	//The weight vector
};

}

#endif /* FUNCTION_APPROX_H_ */
