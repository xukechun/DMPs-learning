#include "dmp/dmp.h"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <cmath>
#include <fstream>
#include <sstream>
#include <string>

using namespace std;
using namespace dmp;

std::vector<DMPData> active_dmp_list;
DMPTraj raw_plan;

//client object
dmp::LearnDMPFromDemo mydemo;
dmp::GetDMPPlan myplan;
dmp::SetActiveDMP myactive_dmp;

void Set_mydemo()
{
	ROS_INFO("mydemo is begin.");

	//Initialize all the points
	std::vector<DMPPoint> pnts;
	int max_index;
	int index = 0;
	string temp0,temp1;

	//left_elbow
	ifstream mydata1,mydata0;
	mydata1.open("/home/xu/dmp_ws/src/dmp/data/data3/right_hand.txt");
	//mydata0.open("/home/xu/dmp_ws/src/dmp/data1/right_shoulder.txt");
	max_index = 171;
	if(!mydata1.is_open())
	{
		ROS_INFO("fail to open the data!");
	}
	while(getline(mydata1,temp1))
	{
		if(index>=0 && index <max_index)
		{
			DMPPoint p;
			double a[3] = {50,50,50};
			istringstream iss0,iss1;
			//iss0.str(temp0);
			iss1.str(temp1);
			string s0,s1;
			int i = 0;
			while(iss1 >> s1)
			{
				//double x0 = atof(s0.c_str());
				double x1 = atof(s1.c_str());
				a[i] = x1;
				cout << x1 << endl;
				i++;
			}
			p.positions.insert(p.positions.begin(),a,a+3);
			if(index == 0)
			{
				double v[3] = {0};
				p.velocities.insert(p.velocities.begin(),v,v+3);
			}
			else
			{
				double v[3] = {0};
				p.velocities.insert(p.velocities.begin(),v,v+3);				
			}
			pnts.push_back(p);
			//ROS_INFO("%d pnts over!",index + 1);
		}
		index++;
	}
	//ROS_INFO("load pnts over!");
	for(int i=0; i<max_index; i++)
	{
		mydemo.request.demo.points.push_back(pnts[i]);
	}
	//ROS_INFO("load pnts over again!");
	//total time is 10s
	std::vector<double> time;
	for(int i=0; i<max_index; i++)
	{
		time.push_back(i*171/(max_index-1));
	}
	

	mydemo.request.demo.times.insert(mydemo.request.demo.times.begin(),time.begin(),time.begin()+max_index);
	//ROS_INFO("dims of my demo: %d",mydemo.request.demo.points[0].positions.size());
	ROS_INFO("demo points : %d",mydemo.request.demo.points.size());
	//ROS_INFO("demo is ready.");	

	//coefficient of elasticity and coefficient of resistance of each DOF
	//should be modified!!!!!
	mydemo.request.k_gains.push_back(10);
	mydemo.request.k_gains.push_back(10);
	mydemo.request.k_gains.push_back(10);
	mydemo.request.d_gains.push_back(10);
	mydemo.request.d_gains.push_back(10);
	mydemo.request.d_gains.push_back(10);
	//ROS_INFO("kd is ready.");	

	//this para is not used
	mydemo.request.num_bases = 1;
	//ROS_INFO("all is ready.");

	mydemo.response.dmp_list = active_dmp_list;

}

void Set_myplan()
{
	myplan.request.x_0 = mydemo.request.demo.points[0].positions;
	myplan.request.x_dot_0 = mydemo.request.demo.points[0].velocities;
	myplan.request.t_0 = 0;
	int pnt = mydemo.request.demo.points.size();
	myplan.request.goal = mydemo.request.demo.points[pnt-1].positions;
	myplan.request.goal_thresh.push_back(10);
	myplan.request.goal_thresh.push_back(10);
	myplan.request.goal_thresh.push_back(10);
	myplan.request.tau = mydemo.response.tau;
	myplan.request.seg_length = myplan.request.tau;
	myplan.request.dt = mydemo.response.tau/(pnt-1);
	myplan.request.integrate_iter = 1;

	myplan.response.at_goal = false;
	myplan.response.plan = raw_plan;
}

void Set_myactive_dmp()
{
	myactive_dmp.request.dmp_list = mydemo.response.dmp_list;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "dmps");
	ros::NodeHandle n;

	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    ros::Rate r(30);
	
	//四条轨迹修改目标函数为Set_mydemo(i),i为路径字符串
	Set_mydemo();

	ROS_INFO("learnFromDemo now begin");
	learnFromDemo(mydemo.request.demo, mydemo.request.k_gains, mydemo.request.d_gains, mydemo.request.num_bases, mydemo.response.dmp_list);
	mydemo.response.tau = mydemo.request.demo.times[mydemo.request.demo.times.size()-1];
	ROS_INFO("learnFromDemo now ready");	

	Set_myplan();

	ROS_INFO("planCallback now begin");
	generatePlan(mydemo.response.dmp_list, myplan.request.x_0, myplan.request.x_dot_0, myplan.request.t_0, myplan.request.goal, myplan.request.goal_thresh,
			     myplan.request.seg_length, myplan.request.tau, myplan.request.dt, myplan.request.integrate_iter, myplan.response.plan, myplan.response.at_goal);
	ROS_INFO("planCallback now ready");	
	
	SetActiveDMP();
	
	//ROS_INFO("SetActiveDMP now begin");
	active_dmp_list = myactive_dmp.request.dmp_list;
	myactive_dmp.response.success = true;
	//ROS_INFO("SetActiveDMP now ready");

	//ROS_INFO("if_success: %d", myactive_dmp.response.success);
	ROS_INFO("if_at_goal: %d", myplan.response.at_goal);
	ROS_INFO("the size of points: %d", myplan.response.plan.points.size());
	//ROS_INFO("the first point: (%lf %lf %lf)",myplan.response.plan.points[0].positions[0],myplan.response.plan.points[0].positions[1],myplan.response.plan.points[0].positions[2]);
    

    while (ros::ok())
    {

    //创建一个 visualization_msgs/Marker消息，并且初始化所有共享的数据。消息成员默认为0，仅仅设置位姿成员w。
    visualization_msgs::Marker points, line_strip;
    points.header.frame_id = line_strip.header.frame_id = "/my_frame";
    points.header.stamp = line_strip.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = "dmps";
    points.action = line_strip.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;


    //分配三个不同的id到三个markers。points_and_lines名称空间的使用确保彼此不会相互冲突。
    points.id = 0;
    line_strip.id = 1;
   


    //设置marker类型到 POINTS, LINE_STRIP 和 LINE_LIST
    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    


    // scale成员对于这些marker类型是不同的,POINTS marker分别使用x和y作为宽和高，然而LINE_STRIP和LINE_LIST marker仅仅使用x，定义为线的宽度。单位是米。
    points.scale.x = 10;
    points.scale.y = 10;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 10;

    // 点为绿色
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip 是蓝色
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    for(int i=0; i<myplan.response.plan.points.size(); i++)
    {	
    	geometry_msgs::Point p;
    	//cout << "(" << myplan.response.plan.points[i].positions[0]<< ","<<myplan.response.plan.points[i].positions[1] << ","<< myplan.response.plan.points[i].positions[2] << ")" <<endl;
    	p.x = myplan.response.plan.points[i].positions[0]-2500;
    	p.y = myplan.response.plan.points[i].positions[1]-2000;
    	p.z = myplan.response.plan.points[i].positions[2]-3000;    	
    	// p.x = mydemo.request.demo.points[i].positions[0]-2500;
    	// p.y = mydemo.request.demo.points[i].positions[1]-2000;
    	// p.z = mydemo.request.demo.points[i].positions[2]-3000;
    	//ROS_WARN("the points: (%lf,%lf,%lf)",p.x,p.y,p.z);

    	points.points.push_back(p);
        line_strip.points.push_back(p);

    }

    //发布各个markers
    marker_pub.publish(points);
    marker_pub.publish(line_strip);

    r.sleep();

  }

	ros::spin();

	return 0;
}

