/*********************************************************************
inverse_kinematics created in 2019.09
**********************************************************************/
#ifndef INVERSE_KINEMATICS_H_
#define INVERSE_KINEMATICS_H_

#include <math.h>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/LU>

namespace inverse_kinematics
{
	const double PI_ = 3.14159265359;

	//angle_generator
	double* angle_generator(Eigen::MatrixXd4 goal);

	//trail generator




}