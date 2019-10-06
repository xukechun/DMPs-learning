/*********************************************************************
inverse_kinematics created in 2019.09
**********************************************************************/
#include "inverse_kinematics.h"
#include<stdio.h>
using namespace Eigen;
using namespace std;

namespace inverse_kinematics
{
	const double a[6] = [0,-0.42500,-0.39225,0,0,0];
	const double d[6] = [0.089159,0,0,0.10915,0.09465,0.8230];

	const alpha = [PI_/2,0,0,PI_/2,-PI_/2,0]; //UR5模型

	//尚缺少角度筛选机制
	double* angle_generator(Eigen::MatrixXd4 goal)
	{
		double nx = goal(0,0);
		double ny = goal(1,0);
		double nz = goal(2,0);
		double ox = goal(0,1);
		double oy = goal(1,1);
		double oz = goal(2,1);
		double ax = goal(0,2);
		double ay = goal(1,2);
		double az = goal(2,2);
		double px = goal(0,3);
		double py = goal(1,3);
		double pz = goal(2,3);

		//求解theta1
		Eigen::Vector2d theta1;
		u1 = d[3]/sqrt((d[5]*ay-py)*(d[5]*ay-py)+(px-d[5]*ax)*(px-d[5]*ax));
        theta1(0) = acos(u1)+atan2(px-d[5]*ax, d[5]*ay-py);
        theta1(1) = -acos(u1)+atan2(px-d[5]*ax, d[5]*ay-py);
		
		//求解theta5
		Eigen::Vector2d theta5;
		theta5(0) = acos(sin(theta1(0))*ax-cos(theta1(0))*ay);
		theta5(1) = -acos(sin(theta1(0))*ax-cos(theta1(0))*ay);

		//求解theta6
		
		Eigen::Vector1d theta6;
		theta6(0) = atan2(-sin(theta1(0))*ox+cos(theta1(0))*oy,sin(theta1(0))*nx-cos(theta1(0))*ny);

		//求解theta3
		Eigen::Vector2d theta3;
		double m = d[4]*((cos(theta1(0))*nx+sin(theta1(0))*ny)*sin(theta6(0))+(cos(theta1(0))*ox+sin(theta1(0))*oy)*cos(theta6(0)))-d[5]*(cos(theta1(0))*ax+sin(theta1(0))*ay)+cos(theta1(0))*px+sin(theta1(0))*py;
		double n = d[4]*(nz*sin(theta6(0))+oz*cos(theta6(0)))-d[5]*az+pz-d[0];
		theta3(0) = acos((m*m+n*n-a[1]*a[1]-a[2]*a[2])/(2*a[1]*a[2]));
		theta3(1) = -acos((m*m+n*n-a[1]*a[1]-a[2]*a[2])/(2*a[1]*a[2]));

		//求解theta2
		//去解!!!
		Eigen::Vector1d theta2;
		theta2(0) = atan2(n*(a[1]+a[2]*cos(theta3(0)))-m*a[2]*sin(theta3(0)), m*(a[1]+a[2]*cos(theta3(0)))+n*a[2]*sin(theta3(0)));

		//求解theta4
		Eigen::Vector1d theta4;
		double u2 = (nz*cos(theta6(0))-oz*sin(theta6(0)))*cos(theta5(0));
		double u3 = ((cos(theta1(0))*nx+sin(theta1(0))*ny)*cos(theta6(0))-(cos(theta1(0))*ox+sin(theta1(0))*oy)*sin(theta6(0)))*cos(theta5(0))-(cos(theta1(0))*ax+sin(theta1(0))*ay)*sin(theta5(0));
		theta4(0) = atan2(u2,u3) - theta2(0) - theta3(0);
	}
}