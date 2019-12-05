#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "geometry_msgs/PoseStamped.h"
#include "iostream"
#include "unistd.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <costmap_2d/costmap_2d_ros.h>
#include <navfn/navfn_ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

using namespace std;

nav_msgs:: OccupancyGrid grid_map;
geometry_msgs::PoseStamped startPose;
geometry_msgs::PoseStamped goalPose;
vector<geometry_msgs::PoseStamped> plan;
navfn::NavfnROS navfn_planner;

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
void callback(const nav_msgs::Odometry::ConstPtr & msg);


void callback(const nav_msgs::Odometry::ConstPtr & msg) {
	startPose.pose = (*msg).pose.pose;
}

void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	goalPose.pose = (*msg).pose;

	startPose.header.frame_id = "map";
	startPose.header.stamp = ros::Time::now();

	goalPose.header.frame_id = "map";
    goalPose.header.stamp = ros::Time::now();

	bool chk = navfn_planner.makePlan(startPose, goalPose, plan);
	cout<<chk<<endl;
	navfn_planner.publishPlan(plan, 0, 0, 255, 1);
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
	grid_map = (*msg);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "Sound_comm");

	ros::NodeHandle n;
	
	tf2_ros::Buffer buffer;
	tf2_ros::TransformListener tfl(buffer);
	ros::Duration timeout(10.0);
	costmap_2d::Costmap2DROS costmap("custom_map", buffer);
	//costmap_2d::LayeredCostmap layers("map", false, true);

	//costmap.getCostmap();
    costmap.start();

	
    navfn_planner.initialize("my_navfn_planner", &costmap);

	ros::Subscriber odomSub = n.subscribe("/custom_odom", 100, callback);
	ros::Subscriber goalSub = n.subscribe("/move_base_simple/goal", 100, goalCallback);
	//ros::Subscriber mapSub = n.subscribe("/rtabmap/grid_map", 100, mapCallback);

	ros::spin();
	return 0;

}