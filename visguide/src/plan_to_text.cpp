#include "ros/ros.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include <message_filters/sync_policies/approximate_time.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include <string>
#include <vector>
#include <math.h>
#include "iostream"
using namespace std;
#define PI 3.14

class new_pos {
    public:
    double x, y, z;
    double ox, oy, oz, ow;
};

double getSlope(double x1, double x2, double y1, double y2);
int sample_points(vector<new_pos> &gPath, double cur_theta);
void callback(const nav_msgs::Path::ConstPtr & msgPath);
void odomCallback(const nav_msgs::Odometry::ConstPtr & msgOdom);
double getDist(double x1, double y1, double x2, double y2);
void dis_direct(vector<new_pos> &gPath);
void angle_correction();
void text_to_speech(vector<string> msg);


//nav_msgs::Path gPlan;
int counter = 0;
new_pos init_source;

double getSlope(double x1, double x2, double y1, double y2) {
    double slope = (y2 - y1) / (x2 - x1);
    return slope;
}
int odom_flag = 0;

void odomCallback(const nav_msgs::Odometry::ConstPtr & msgOdom) {
    init_source.x = (*msgOdom).pose.pose.position.x;init_source.y = (*msgOdom).pose.pose.position.y;init_source.z = (*msgOdom).pose.pose.position.z;
    init_source.ox = (*msgOdom).pose.pose.orientation.x;init_source.oy = (*msgOdom).pose.pose.orientation.y;init_source.oz = (*msgOdom).pose.pose.orientation.z;
    init_source.ow = (*msgOdom).pose.pose.orientation.w;
}

void callback(const nav_msgs::Path::ConstPtr & msgPath){
    odom_flag = 1;
    new_pos cur_pose(init_source);
    //cout<<(*msgPath)[0].pose.orientation<<endl;
    //cout<<cur_pose.x<<"  "<<cur_pose.y<<"  "<<cur_pose.z<<endl;
    //cout<<cur_pose.ox<<"  "<<cur_pose.oy<<"  "<<cur_pose.oz<<"   "<<cur_pose.ow<<endl;
    double cur_theta = acos(cur_pose.ow);
    cout<<"Current orientation:  "<<(cur_theta * 360.0 / PI)<<endl;
    double cur_x =  cur_pose.ox;
    double cur_y = cur_pose.oy;
    /*
    //cout<<"Current pose:  "<<cur_x<<"  "<<cur_y<<endl;
    double cur_slope = (cur_x, 0, cur_y, 0);
    //cout<<"Current slope:  "<<cur_slope<<endl;
    cout<<"------------"<<endl;
    */
    counter++;
    int i = 0;
    vector<new_pos> gPath;
    new_pos point;
	for(std::vector<geometry_msgs::PoseStamped>::const_iterator it= msgPath->poses.begin(); it!= msgPath->poses.end();  ++it) {
        //cout<<"Position  "<<(*it).pose.position.x<<"   "<<(*it).pose.position.y<<"   "<<(*it).pose.position.z<<endl;
        
        //cout<<"CHECK:  "<<(*it).pose.orientation<<endl;
        //break;
        

        //Cloning for easier reference
        point.x = (*it).pose.position.x; point.y = (*it).pose.position.y; point.z = (*it).pose.position.z;
        point.ox = (*it).pose.orientation.x; point.oy = (*it).pose.orientation.y; point.oz = (*it).pose.orientation.z;
        point.ow = (*it).pose.orientation.w;
        gPath.push_back(point);
        ++i;
    }
    
    sample_points(gPath, cur_theta);
    cout<<"Total :"<<i<<endl;
    //cout<<msgPath.poses[0].pose<<endl;
    cout<<"-------------"<<endl;
    
   odom_flag = 0;
}



int sample_points(vector<new_pos> &gPath, double cur_theta) {
    int l = gPath.size();
    vector<new_pos> newgPath(3);
    //Parameters
    double threshold = 1;
    int p_prev_i = 0;
    int prev_i = 100;
    
    for(int i = 600;i < l; i += 200) {
        //cout<<"Points:  "<<gPath[i].x<<"  "<<gPath[i].y<<endl;
        double slope3 = getSlope(gPath[i].x, gPath[p_prev_i].x, gPath[i].y, gPath[p_prev_i].y);
        double slope2 = getSlope(gPath[i].x, gPath[prev_i].x, gPath[i].y, gPath[prev_i].y);
        double slope1 = getSlope(gPath[prev_i].x, gPath[p_prev_i].x, gPath[prev_i].y, gPath[p_prev_i].y);
        if(abs(slope2 - slope1) <= threshold) {
            prev_i = i;
        } else {
            //cout<<"Points:  "<<gPath[p_prev_i].x<<"  "<<gPath[p_prev_i].y<<"  "<<gPath[prev_i].x<<"  "<<gPath[prev_i].y<<"  "<<gPath[i].x<<"  "<<gPath[i].y<<endl;
            //p_prev_i = prev_i;
            //prev_i = i;
            //cout<<"Slope:  "<<slope1<<"  "<<slope2<<"  "<<slope3<<endl;
            //cout<<"I:  "<<i<<endl;
            newgPath[0] = gPath[p_prev_i];
            newgPath[1] = gPath[prev_i];
            newgPath[2] = gPath[i];
            double new_theta = atan(slope1);
            double angle_diff = (new_theta - cur_theta) * 180.0 / PI;
            cout<<"Angle correction: "<<angle_diff<<endl;
            dis_direct(newgPath);
            return 0;
        }
    }
    double slope_st = getSlope(gPath[0].x, gPath[l - 1].x, gPath[0].y, gPath[l - 1].y);
    double new_theta = atan(slope_st);
    double angle_diff = (new_theta - cur_theta) * 180.0 / PI;
    cout<<"Angle correction: "<<angle_diff<<endl;

    double d1 = getDist(gPath[0].x, gPath[l - 1].x, gPath[0].y, gPath[l - 1].y);
    string text_msg = "Go forward " + to_string(int(round(d1))) + " meters";
    cout<<text_msg<<endl;
    //text_to_speech(text_msg);
    return 0;
}
double getDist(double x1, double x2, double y1, double y2) {
    double dist = sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
    return dist;
}
void dis_direct(vector<new_pos> &gPath) {
    //Distance to go
    double d1 = getDist(gPath[0].x, gPath[1].x, gPath[0].y, gPath[1].y);
    double d2 = getDist(gPath[2].x, gPath[1].x, gPath[2].y, gPath[1].y);
    double d3 = getDist(gPath[0].x, gPath[2].x, gPath[0].y, gPath[2].y);
    //cout<<gPath[0].x<<"  "<<gPath[1].x<<"  "<<gPath[2].x<<"  "<<gPath[0].y<<"  "<<gPath[1].y<<"  "<<gPath[2].y<<endl;
    //cout<<"Distance:  "<<d1<<"  "<<d2<<"  "<<d3<<endl;
    //Angle to rotate
    double alpha = acos((d2*d2 + d3*d3 - d1*d1) / (2 * d2 * d3)); alpha = alpha * 180.0 / PI;
    double beta = acos((d3*d3 + d1*d1 - d2*d2) / (2 * d3 * d1)); beta = beta * 180.0 / PI;
    //cout<<endl<<"Angles:  "<<alpha<<"  "<<beta<<endl;
    double angle_to_rotate = (alpha + beta);
    //d1 = d1 + d2 * cos(angle_to_rotate);

    //Direction to rotate
    string direction;
    double slope01 = (gPath[1].y - gPath[0].y) / (gPath[1].x - gPath[0].x);
    double slope12 = (gPath[2].y - gPath[0].y) / (gPath[2].x - gPath[0].x);
    if(slope01 >= slope12) {
        direction = "right";
    } else {
        direction = "left";
        //angle_to_rotate = 180 - angle_to_rotate;
    }

    //Generate message
    string text_msg = "Go forward " + to_string(int(round(d1))) + " units and turn " + to_string(int(angle_to_rotate)) + " degrees to " + direction;
    cout<<text_msg<<endl;

    //Get voice command
    //text_to_speech(text_msg);
}

void angle_correction() {

}
void text_to_speech(string text_msg) {

}

int main(int argc, char **argv){
	
	ros::init(argc, argv, "Text_Comm");
    ros::NodeHandle n;
	cout<<"checkpoint1"<<endl;
	/*message_filters::Subscriber<nav_msgs::Path> PathSub(n, "/move_base/NavfnROS/plan", 10);
    message_filters::Subscriber<nav_msgs::Odometry> OdomSub(n, "/visguide/zed_node/odom", 10);
	message_filters::TimeSynchronizer<nav_msgs::Path, nav_msgs::Odometry> sync(PathSub, OdomSub, 30);
	sync.registerCallback(boost::bind(&callback, _1, _2));
    //PathSub.registerCallback(callback);
	*/
    ros::Subscriber PathSub = n.subscribe("/move_base/NavfnROS/plan", 100, callback);
    ros::Subscriber OdomSub = n.subscribe("/visguide/zed_node/odom", 100, odomCallback);
   	ros::Rate loop_rate(10);
	
	ros::spin();
	return 0;
}



