/*********************************************************************
DMP created in 2019.09
**********************************************************************/
#ifndef DMP_H_
#define DMP_H_

#include "ros/ros.h"
#include "dmp/LearnDMPFromDemo.h"
#include "dmp/GetDMPPlan.h"
#include "dmp/SetActiveDMP.h"
#include "dmp/radial_approx.h"
#include "dmp/fourier_approx.h"
#include "dmp/linear_approx.h"
#include <math.h>

namespace dmp{

//四个轨迹同时在RVIZ中显示
//逆运动学和运动控制
double calcPhase(const double curr_time, const double tau);

void learnFromDemo(const DMPTraj &demo,
				   const std::vector<double> &k_gains,
				   const std::vector<double> &d_gains,
				   const int &num_bases,
				   std::vector<DMPData> &dmp_list);

void generatePlan(const std::vector<DMPData> &dmp_list,
				  const std::vector<double> &x_0,
				  const std::vector<double> &x_dot_0,
				  const double &t_0,
				  const std::vector<double> &goal,
				  const std::vector<double> &goal_thresh,
				  const double &seg_length,
				  const double &tau,
				  const double &total_dt,
				  const int &integrate_iter,
				  DMPTraj &plan,
				  uint8_t &at_goal);

}
#endif /* DMP_H_ */
