#include "dmp/dmp.h"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <cmath>

using namespace dmp;

std::vector<DMPData> active_dmp_list;

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
	ROS_INFO("demo points : %d",mydemo.request.demo.points.size());
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

	mydemo.response.dmp_list = active_dmp_list;

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

	myplan.response.at_goal = false;
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
	generatePlan(active_dmp_list, myplan.request.x_0, myplan.request.x_dot_0, myplan.request.t_0, myplan.request.goal, myplan.request.goal_thresh,
			     myplan.request.seg_length, myplan.request.tau, myplan.request.dt, myplan.request.integrate_iter, myplan.response.plan, myplan.response.at_goal);
	ROS_INFO("planCallback now ready");	
	
	SetActiveDMP();
	
	ROS_INFO("SetActiveDMP now begin");
	active_dmp_list = myactive_dmp.request.dmp_list;
	myactive_dmp.response.success = true;
	ROS_INFO("SetActiveDMP now ready");

	ROS_INFO("if_success: %d", myactive_dmp.response.success);
	ROS_INFO("if_at_goal: %d", myplan.response.at_goal);
	ROS_INFO("the size of points: %d", myplan.response.plan.points.size());
    
    while (ros::ok())
    {

    //创建一个 visualization_msgs/Marker消息，并且初始化所有共享的数据。消息成员默认为0，仅仅设置位姿成员w。
    visualization_msgs::Marker points, line_strip, line_list;
    points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/my_frame";
    points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
    points.ns = line_strip.ns = line_list.ns = "dmps";
    points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;


    //分配三个不同的id到三个markers。points_and_lines名称空间的使用确保彼此不会相互冲突。
    points.id = 0;
    line_strip.id = 1;
    line_list.id = 2;


    //设置marker类型到 POINTS, LINE_STRIP 和 LINE_LIST
    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_list.type = visualization_msgs::Marker::LINE_LIST;


    // scale成员对于这些marker类型是不同的,POINTS marker分别使用x和y作为宽和高，然而LINE_STRIP和LINE_LIST marker仅仅使用x，定义为线的宽度。单位是米。
    points.scale.x = 0.05;
    points.scale.y = 0.05;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.05;

    // 点为绿色
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // Line strip 是蓝色
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    for(int i=0; i<myplan.response.plan.points.size(); i++)
    {	
    	geometry_msgs::Point p;
    	p.x = myplan.response.plan.points[i].positions[0];
    	p.y = myplan.response.plan.points[i].positions[1];
    	p.z = myplan.response.plan.points[i].positions[2];

    	points.points.push_back(p);
        line_strip.points.push_back(p);

    }

    //发布各个markers
    marker_pub.publish(points);
    marker_pub.publish(line_strip);
    marker_pub.publish(line_list);

    r.sleep();

  }

	ros::spin();

	return 0;
}

