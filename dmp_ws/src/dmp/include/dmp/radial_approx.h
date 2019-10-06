/*********************************************************************
DMP created in 2019.09
 *********************************************************************/

#ifndef RADIAL_APPROX_H_
#define RADIAL_APPROX_H_

#include "dmp/function_approx.h"
#include <iostream>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/LU>

namespace dmp{

/// Class for linear function approximation with the univariate Radial basis
class RadialApprox : public FunctionApprox{
public:
	RadialApprox(int num_bases, double base_width, double alpha);
	RadialApprox(const std::vector<double> &w, double base_width, double alpha);
	virtual ~RadialApprox();

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
	/**\brief Calculate the Radial basis features at point x
	 * \param x The point at which to get features
	 */
	void calcFeatures(double x);

	/**\brief Calculate the Moore-Penrose pseudoinverse of a matrix using SVD
	 * \param mat The matrix to pseudoinvert
	 * \return The pseudoinverted matrix
	 */
	Eigen::MatrixXd pseudoinverse(Eigen::MatrixXd mat);

	double *features;  //Storage for a set of features
	double *centers;   //Centers of RBFs
	double *widths;    //Widths of RBFs

};

}

#endif /* RADIAL_APPROX_H_ */
