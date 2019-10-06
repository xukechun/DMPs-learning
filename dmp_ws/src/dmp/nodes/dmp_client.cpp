#include "dmp/dmp.h"
#include <geometry_msgs/Point.h>
using namespace dmp;

//client object
dmp::LearnDMPFromDemo mydemo;
dmp::GetDMPPlan myplan;
dmp::SetActiveDMP myactive_dmp;

void Set_mydemo()
{
	ROS_INFO("mydemo is begin.");
	//Initialize all the points
	double p1[3] = {0,0,0};
	double p2[3] = {1,1,1};
	double p3[3] = {2,3,4};
	double v1[3] = {0,0,0};
	double v2[3] = {0,0,0};
	double v3[3] = {0,0,0};
	double time[3] = {0,1,2};

	DMPPoint pnt1;
	pnt1.positions.insert(pnt1.positions.begin(),p1+1,p1+3);
	pnt1.velocities.insert(pnt1.velocities.begin(),v1+1,v1+3);
	ROS_INFO("pnt1 is ready.");	

	DMPPoint pnt2;
	pnt2.positions.insert(pnt2.positions.begin(),p2+1,p2+3);
	pnt2.velocities.insert(pnt2.velocities.begin(),v2+1,v2+3);
	ROS_INFO("pnt2 is ready.");	

	DMPPoint pnt3;
	pnt3.positions.insert(pnt3.positions.begin(),p3+1,p3+3);
	pnt3.velocities.insert(pnt3.velocities.begin(),v3+1,v3+3);
	ROS_INFO("pnt3 is ready.");	

	mydemo.request.demo.points.push_back(pnt1);
	mydemo.request.demo.points.push_back(pnt2);
	mydemo.request.demo.points.push_back(pnt3);	

	mydemo.request.demo.times.insert(mydemo.request.demo.times.begin(),time+1, time+3);
	ROS_INFO("demo is ready.");	

	//coefficient of elasticity and coefficient of resistance of each DOF
	mydemo.request.k_gains.push_back(1);
	mydemo.request.k_gains.push_back(1);
	mydemo.request.k_gains.push_back(1);
	mydemo.request.d_gains.push_back(1);
	mydemo.request.d_gains.push_back(1);
	mydemo.request.d_gains.push_back(1);
	ROS_INFO("kd is ready.");	

	//this para is not used
	mydemo.request.num_bases = 1;
	ROS_INFO("all is ready.");

}

void Set_myplan()
{
	myplan.request.x_0 = mydemo.request.demo.points[0].positions;
	myplan.request.x_dot_0 = mydemo.request.demo.points[0].velocities;
	myplan.request.t_0 = 0;
	int pnt = mydemo.request.demo.points.size();
	myplan.request.goal = mydemo.request.demo.points[pnt-1].positions;
	myplan.request.goal_thresh.push_back(0.01);
	myplan.request.goal_thresh.push_back(0.01);
	myplan.request.goal_thresh.push_back(0.01);
	myplan.request.tau = mydemo.response.tau;
	myplan.request.seg_length = myplan.request.tau;
	myplan.request.dt = mydemo.response.tau/(pnt-1);
	myplan.request.integrate_iter = 1;
}

void Set_myactive_dmp()
{
	myactive_dmp.request.dmp_list = mydemo.response.dmp_list;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "dmp_client");
	ros::NodeHandle n;

	ros::ServiceClient client1 = n.serviceClient<dmp::LearnDMPFromDemo>("learn_dmp_from_demo");
	ros::ServiceClient client2 = n.serviceClient<dmp::GetDMPPlan>("get_dmp_plan");
	ros::ServiceClient client3 = n.serviceClient<dmp::SetActiveDMP>("set_active_dmp");
	ROS_INFO("DMP clients are ready.");

	Set_mydemo();
	ROS_INFO("mydemo is ready.");
	if(client1.call(mydemo))
	{
		Set_myplan();
		ROS_INFO("myplan is ready.");
	}
	else
	{
    	ROS_ERROR("Failed to call service1");
    	return 1;		
	}
	if(client2.call(myplan))
	{	
		Set_myactive_dmp();
		ROS_INFO("myactive_dmp is ready.");
	}
	else
	{
		ROS_ERROR("Failed to call service2");
    	return 1;

	}

	ROS_INFO("DMP client has pub the data.");
	
	if (client3.call(myactive_dmp))
    {
    	ROS_INFO("if success: %d ", myactive_dmp.response.success);
    	ROS_INFO("if at_goal: %d ", myplan.response.at_goal);
    	ROS_INFO("the first point: %lf %lf %lf", myplan.response.plan.points[1].positions[0],myplan.response.plan.points[1].positions[1],myplan.response.plan.points[1].positions[2]);
    }
    else
    {
    	ROS_ERROR("Failed to call service3");
    	return 1;
    }

    ros::spin();
    
	return 0;
}

