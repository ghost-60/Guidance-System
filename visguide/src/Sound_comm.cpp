#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "geometry_msgs/PoseStamped.h"
#include "iostream"
#include "unistd.h"
using namespace std;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	cout<<"checkpoint"<<endl;
	
	MoveBaseClient ac("move_base", true);
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.pose = (*msg).pose;
	goal.target_pose.header.frame_id = (*msg).header.frame_id;
	
	for (;;) {
    usleep(5000000);
    //cout<<"checkpoint2"<<endl;
		goal.target_pose.header.stamp = ros::Time::now();
		ac.sendGoal(goal);
    //cout<<"checkpoint3"<<endl;
  	//ac.waitForResult();
    //cout<<"checkpoint3"<<endl;
		cout<<"Publishing goal again"<<endl;
	}
}
int main(int argc, char** argv){
  ros::init(argc, argv, "Nav_goals");

  ros::NodeHandle n;
  ros::Subscriber GoalSub = n.subscribe("/move_base_simple/goal", 100, callback);
  ros::Rate loop_rate(10);
	
  ros::spin();
  return 0;
}