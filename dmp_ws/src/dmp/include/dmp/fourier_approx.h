/*********************************************************************
DMP created in 2019.09
 *********************************************************************/

#ifndef FOURIER_APPROX_H_
#define FOURIER_APPROX_H_

#include "dmp/function_approx.h"
#include <iostream>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/LU>

namespace dmp{

const double PI = 3.14159265359;

/// Class for linear function approximation with the univariate Fourier basis
class FourierApprox : public FunctionApprox{
public:
	FourierApprox(int order);
	FourierApprox(const std::vector<double> &w);
	virtual ~FourierApprox();

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
	/**\brief Calculate the Fourier basis features at point x
	 * \param x The point at which to get features
	 */
	void calcFeatures(double x);

	/**\brief Calculate the Moore-Penrose pseudoinverse of a matrix using SVD
	 * \param mat The matrix to pseudoinvert
	 * \return The pseudoinverted matrix
	 */
	Eigen::MatrixXd pseudoinverse(Eigen::MatrixXd mat);

	double *features;  //Storage for a set of features

};

}

#endif /* FOURIER_APPROX_H_ */
