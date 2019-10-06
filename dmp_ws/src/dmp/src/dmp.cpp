/*********************************************************************
DMP created in 2019.09
 *********************************************************************/

#include "dmp/dmp.h"
using namespace std;

namespace dmp{

#define MAX_PLAN_LENGTH 1000

double alpha = -log(0.01); //Ensures 99% phase convergence at t=tau

/**
 * @brief Calculate an exp-decaying 1 to 0 phase based on time and the time scaling constant tau
 * @param[in] curr_time The current time in seconds from the start of DMP execution / trajectory
 * @param[in] tau The DMP time scaling constant
 * @return A zero to one phase
 */
double calcPhase(double curr_time, double tau)
{
	return exp(-(alpha/tau)*curr_time);
}


/**
 * @brief Given a single demo trajectory, produces a multi-dim DMP
 * @param[in] demo An n-dim demonstration trajectory
 * @param[in] k_gains A proportional gain for each demo dimension
 * @param[in] d_gains A vector of differential gains for each demo dimension
 * @param[in] num_bases The number of basis functions to use for the fxn approx (i.e. the order of the Fourier series)
 * @param[out] dmp_list An n-dim list of DMPs that are all linked by a single canonical (phase) system
 */

//给定待模仿的轨迹，三个维数k,d参数，用于傅里叶近似的基函数个数，ｎ维的DMP_DATA
void learnFromDemo(const DMPTraj &demo,
				   const vector<double> &k_gains,
				   const vector<double> &d_gains,
				   const int &num_bases,
				   vector<DMPData> &dmp_list)
{
	//Determine traj length and dim
	int n_pts = demo.points.size();
	if(n_pts < 1){
		ROS_ERROR("Empty trajectory passed to learn_dmp_from_demo service!");
		return;
	}

	int dims = demo.points[0].positions.size();
	double tau = demo.times[n_pts-1];//总的执行时间

	double *x_demo = new double[n_pts];
	double *v_demo = new double[n_pts];
	double *v_dot_demo = new double[n_pts];
	double *f_domain = new double[n_pts];
	double *f_targets = new double[n_pts];
	FunctionApprox *f_approx = new LinearApprox();

	//Compute the DMP weights for each DOF separately
	for(int d=0; d<dims; d++){
		double curr_k = k_gains[d];
		double curr_d = d_gains[d];
		double x_0 = demo.points[0].positions[d];
		double goal = demo.points[n_pts-1].positions[d];
		x_demo[0] = demo.points[0].positions[d];
		v_demo[0] = 0;
		v_dot_demo[0] = 0;

		//Calculate the demonstration v and v dot by assuming constant acceleration over a time period
		for(int i=1; i<n_pts; i++){
			x_demo[i] = demo.points[i].positions[d];
			double dx = x_demo[i] - x_demo[i-1];
			double dt = demo.times[i] - demo.times[i-1];
			v_demo[i] = dx/dt;
			v_dot_demo[i] = (v_demo[i] - v_demo[i-1]) / dt;
		}

		//Calculate the target pairs so we can solve for the weights
		for(int i=0; i<n_pts; i++){
			double phase = calcPhase(demo.times[i],tau);
			f_domain[i] = demo.times[i]/tau;  //Scaled time is cleaner than phase for spacing reasons
			f_targets[i] = ((tau*tau*v_dot_demo[i] + curr_d*tau*v_demo[i]) / curr_k) - (goal-x_demo[i]) + ((goal-x_0)*phase);
			f_targets[i] /= phase; // Do this instead of having fxn approx scale its output based on phase
			
			//Convergence to target position and target speed at the same time
			//f_targets[i] = (tau*v_dot_demo[i]+(phase - 1)*(curr_k*(goal - x_demo[i]) - curr_d*tau*(goal - v_demo[i])))/(goal - x_0);
		}

		//Solve for weights
		f_approx->leastSquaresWeights(f_domain, f_targets, n_pts);

		//Create the DMP structures
		DMPData *curr_dmp = new DMPData();
		curr_dmp->weights = f_approx->getWeights();
		curr_dmp->k_gain = curr_k;
		curr_dmp->d_gain = curr_d;
        for(int i=0; i<n_pts; i++){
            curr_dmp->f_domain.push_back(f_domain[i]); 
            curr_dmp->f_targets.push_back(f_targets[i]);
        }
		dmp_list.push_back(*curr_dmp);
	}

	delete[] x_demo;
	delete[] v_demo;
	delete[] v_dot_demo;
	delete[] f_domain;
	delete[] f_targets;
	delete f_approx;
}



/**
 * @brief Use the current active multi-dim DMP to create a plan starting from x_0 toward a goal
 * @param[in] dmp_list An n-dim list of DMPs that are all linked by a single canonical (phase) system
 * @param[in] x_0 The (n-dim) starting state for planning
 * @param[in] x_dot_0 The (n-dim) starting instantaneous change in state for planning
 * @param[in] t_0 The time in seconds at which to begin the planning segment. Should only be nonzero when doing a partial segment plan that does not start at beginning of DMP
 * @param[in] goal The (n-dim) goal point for planning
 * @param[in] goal_thresh Planning will continue until system is within the specified threshold of goal in each dimension
 * @param[in] seg_length The length of the requested plan segment in seconds. Set to -1 if plan until goal is desired.
 * @param[in] tau The time scaling constant (in this implementation, it is the desired length of the TOTAL (not just this segment) DMP execution in seconds)
 * @param[in] total_dt The desired time resolution of the plan
 * @param[in] integrate_iter The number of loops used when numerically integrating accelerations
 * @param[out] plan An n-dim plan starting from x_0
 * @param[out] at_goal True if the final time is greater than tau AND the planned position is within goal_thresh of the goal
 */
void generatePlan(const vector<DMPData> &dmp_list,
				  const vector<double> &x_0,
				  const vector<double> &x_dot_0,
				  const double &t_0,
				  const vector<double> &goal,
				  const vector<double> &goal_thresh,
				  const double &seg_length,
				  const double &tau,
				  const double &total_dt,
				  const int &integrate_iter,
				  DMPTraj &plan,
				  uint8_t &at_goal)
{
	plan.points.clear();
	plan.times.clear();
	at_goal = false;

	int dims = dmp_list.size();
	int n_pts = 0;
	double dt = total_dt / integrate_iter;
	//时间同步

	vector<double> *x_vecs, *x_dot_vecs;
	vector<double> t_vec;
	x_vecs = new vector<double>[dims];
	x_dot_vecs = new vector<double>[dims];
	FunctionApprox **f = new FunctionApprox*[dims];

	for(int i=0; i<dims; i++)
		f[i] = new LinearApprox(dmp_list[i].f_domain, dmp_list[i].f_targets);
	
	double t = 0;
	double f_eval;

	//Plan for at least tau seconds.  After that, plan until goal_thresh is satisfied.
	//Cut off if plan exceeds MAX_PLAN_LENGTH seconds, in case of overshoot / oscillation
	//Only plan for seg_length seconds if specified
	bool seg_end = false;
	while(((t+t_0) < tau || (!at_goal && t<MAX_PLAN_LENGTH)) && !seg_end){
		//Check if we've planned to the segment end yet
		if(seg_length > 0){
			if (t > seg_length) seg_end = true;
		}

		//Plan in each dimension
		for(int i=0; i<dims; i++){
            double x,v;
            if(n_pts==0){
                x = x_0[i]; //initial position
                v = x_dot_0[i]; //initial velocities
            }
            else{			
                x = x_vecs[i][n_pts-1];
			    v = x_dot_vecs[i][n_pts-1] * tau;
            }

			//Numerically integrate to get new x and v
			for(int iter=0; iter<integrate_iter; iter++)
			{
				//Compute the phase and the log of the phase to assist with some numerical issues
				//Then, evaluate the function approximator at the log of the phase
				double s = calcPhase((t+t_0) + (dt*iter), tau);
				double log_s = (t+t_0)/tau;
				if(log_s >= 1.0){
					f_eval = 0;
				}
				else{
					f_eval = f[i]->evalAt(log_s) * s;
				}
				
				//Update v dot and x dot based on DMP differential equations
				double v_dot = (dmp_list[i].k_gain*((goal[i]-x) - (goal[i]-x_0[i])*s + f_eval) - dmp_list[i].d_gain*v) / tau;
				double x_dot = v/tau;

				//Update state variables
				v += v_dot * dt;
				x += x_dot * dt;
			}

			//Add current state to the plan
			x_vecs[i].push_back(x);
			x_dot_vecs[i].push_back(v/tau);
		}
		t += total_dt;
		t_vec.push_back(t);
		n_pts++;

		//If plan is at least minimum length, check to see if we are close enough to goal
		if((t+t_0) >= tau){
			at_goal = true;
			for(int i=0; i<dims; i++){
				if(goal_thresh[i] > 0){
					if(fabs(x_vecs[i][n_pts-1] - goal[i]) > goal_thresh[i])
						at_goal = false;
				}
			}
		}
	}
	ROS_INFO("n_pts: %d",n_pts);

	//Create a plan from the generated trajectories

	plan.points.resize(n_pts+1);
	for(int j=0; j<=n_pts; j++){
		plan.points[j].positions.resize(dims);
		plan.points[j].velocities.resize(dims);
	}
	for(int i=0; i<dims; i++){
		for(int j=0; j<=n_pts; j++){
			plan.points[j].positions[i] = x_vecs[i][j];
			plan.points[j].velocities[i] = x_dot_vecs[i][j];
		}
	}
	plan.times = t_vec;
	ROS_INFO("planning time: %lf",plan.times[2]);

	//Clean up
	for(int i=0; i<dims; i++){
		delete f[i];
	}
	delete[] f;
	delete[] x_vecs;
	delete[] x_dot_vecs;
}

}

