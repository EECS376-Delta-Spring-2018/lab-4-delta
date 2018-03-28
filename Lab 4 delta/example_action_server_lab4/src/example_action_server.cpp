// example_action_server: 2nd version, includes "cancel" and "feedback"
// expects client to give an integer corresponding to a timer count, in seconds
// server counts up to this value, provides feedback, and can be cancelled any time
// re-use the existing action message, although not all fields are needed
// use request "input" field for timer setting input, 
// value of "fdbk" will be set to the current time (count-down value)
// "output" field will contain the final value when the server completes the goal request

#include<ros/ros.h>
#include <actionlib/server/simple_action_server.h>
//the following #include refers to the "action" message defined for this package
// The action message can be found in: .../example_action_server/action/demo.action
// Automated header generation creates multiple headers for message I/O
// These are referred to by the root name (demo) and appended name (Action)
#include <example_action_server_lab4/demoAction.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>
#include <math.h>
#include <std_msgs/Bool.h> // boolean message


using namespace std;


int g_count = 0;
bool g_count_failure = false;
bool g_alarm = false;
bool g_goal_active=false;

const double g_move_speed = 0.1;// set forward speed to this value, e.g. 1m/s
const double g_spin_speed = 0.1; // set yaw rate to this value, e.g. 1 rad/s
const double g_sample_dt = 0.01;
const double g_dist_tol = 0.01; // 1cm
double robot_heading; //robot's actual heading (not 0!!!)

geometry_msgs::Twist g_twist_cmd;
ros::Publisher g_twist_commander; //global publisher object
geometry_msgs::Pose g_current_pose; // not really true--should get this from odom 

// here are a few useful utility functions:
double sgn(double x);
double min_spin(double spin_angle);
double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion);
geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi);

void do_halt();
void do_move(double distance);
void do_spin(double spin_ang);

class ExampleActionServer {
private:

    ros::NodeHandle nh_;  // we'll need a node handle; get one upon instantiation

    // this class will own a "SimpleActionServer" called "as_".
    // it will communicate using messages defined in example_action_server/action/demo.action
    // the type "demoAction" is auto-generated from our name "demo" and generic name "Action"
    actionlib::SimpleActionServer<example_action_server_lab4::demoAction> as_;
    
    // here are some message types to communicate with our client(s)
    example_action_server_lab4::demoGoal goal_; // goal message, received from client
    example_action_server_lab4::demoResult result_; // put results here, to be sent back to the client when done w/ goal
    example_action_server_lab4::demoFeedback feedback_; // for feedback 
    //  use: as_.publishFeedback(feedback_); to send incremental feedback to the client
    int countdown_val_;


public:
    ExampleActionServer(); //define the body of the constructor outside of class definition

    ~ExampleActionServer(void) {
    }
    // Action Interface
    void executeCB(const actionlib::SimpleActionServer<example_action_server_lab4::demoAction>::GoalConstPtr& goal);
};

//implementation of the constructor:
// member initialization list describes how to initialize member as_
// member as_ will get instantiated with specified node-handle, name by which this server will be known,
//  a pointer to the function to be executed upon receipt of a goal.
//  
// Syntax of naming the function to be invoked: get a pointer to the function, called executeCB, 
// which is a member method of our class exampleActionServer.  
// Since this is a class method, we need to tell boost::bind that it is a class member,
// using the "this" keyword.  the _1 argument says that our executeCB function takes one argument
// The final argument,  "false", says don't start the server yet.  (We'll do this in the constructor)

ExampleActionServer::ExampleActionServer() :
   as_(nh_, "path_action", boost::bind(&ExampleActionServer::executeCB, this, _1),false) 
// in the above initialization, we name the server "example_action"
//  clients will need to refer to this name to connect with this server
{
    ROS_INFO("in constructor of exampleActionServer...");
    // do any other desired initializations here...specific to your implementation

    as_.start(); //start the server running
}

void get_yaw_and_dist(geometry_msgs::Pose current_pose, geometry_msgs::Pose goal_pose,double &dist, double &heading) {
 
 //dist = 0.0; //FALSE!!
 double x_s, y_s, x_g, y_g;
 
 x_s = current_pose.position.x;
 y_s = current_pose.position.y;
 x_g = goal_pose.position.x;
 y_g = goal_pose.position.y;
 double dx, dy;
 dx = x_g - x_s;
 dy = y_g - y_s;
 dist = sqrt((dx*dx)+(dy*dy)); //calculate desired travel distance
 if (dist < g_dist_tol) { //too small of a motion, so just set the heading from goal heading
   heading = convertPlanarQuat2Phi(goal_pose.orientation); 
 }
 else if (dy == 0 && dx < 0){ //if dy=0 & dx<0, then heading should be 3.14, not zero
	heading = 3.14 - robot_heading; 
	robot_heading = 3.14;
 }
 else {
    heading = (atan2(dy,dx) - robot_heading); //subtract robot's actual heading from goal heading
    robot_heading = atan2(dy,dx); //set robot's actual heading to goal heading
 }

}

//executeCB implementation: this is a member method that will get registered with the action server
// argument type is very long.  Meaning:
// actionlib is the package for action servers
// SimpleActionServer is a templated class in this package (defined in the "actionlib" ROS package)
// <example_action_server_lab4::demoAction> customizes the simple action server to use our own "action" message 
// defined in our package, "example_action_server", in the subdirectory "action", called "demo.action"
// The name "demo" is prepended to other message types created automatically during compilation.
// e.g.,  "demoAction" is auto-generated from (our) base name "demo" and generic name "Action"
void ExampleActionServer::executeCB(const actionlib::SimpleActionServer<example_action_server_lab4::demoAction>::GoalConstPtr& goal) {
    ROS_INFO("callback activated");
    double yaw_desired, yaw_current, travel_distance, spin_angle;
    nav_msgs::Path paths;
    geometry_msgs::Pose pose_desired;
    paths = goal->input;
    int npts = paths.poses.size();
    ROS_INFO("received path request with %d poses",npts);  
    int move = 0;
    //do work here: this is where your interesting code goes
    //ros::Rate timer(1.0); // 1Hz timer
    //implement a simple timer, which counts down from provided countdown_val to 0, in seconds
    for (int i=0;i<npts;i++) { //visit each subgoal
        // odd notation: drill down, access vector element, drill some more to get pose
        pose_desired = paths.poses[i].pose; //get next pose from vector of poses
        
        //WRITE THIS FNC: compute desired heading and travel distance based on current and desired poses
        get_yaw_and_dist(g_current_pose, pose_desired,travel_distance, yaw_desired);
        ROS_INFO("pose %d: desired yaw = %f; desired (x,y) = (%f,%f)",i,yaw_desired,pose_desired.position.x,pose_desired.position.y); 
        ROS_INFO("current (x,y) = (%f, %f)",g_current_pose.position.x,g_current_pose.position.y);
        ROS_INFO("travel distance = %f",travel_distance);         
        
        
        // a quaternion is overkill for navigation in a plane; really only need a heading angle
        // this yaw is measured CCW from x-axis
        // GET RID OF NEXT LINE AFTER FIXING get_yaw_and_dist()
        //yaw_desired = convertPlanarQuat2Phi(pose_desired.orientation); //from i'th desired pose
        
        ROS_INFO("pose %d: desired yaw = %f",i,yaw_desired);        
        //yaw_current = convertPlanarQuat2Phi(g_current_pose.orientation); //our current yaw--should use a sensor
        spin_angle = yaw_desired - yaw_current; // spin this much
        spin_angle = min_spin(spin_angle);// but what if this angle is > pi?  then go the other way
        do_spin(spin_angle); // carry out this incremental action
        // we will just assume that this action was successful--really should have sensor feedback here
        g_current_pose.orientation = pose_desired.orientation; // assumes got to desired orientation precisely
        
        //FIX THE NEXT LINE, BASED ON get_yaw_and_dist()
        do_move(travel_distance);  // move forward desired travel distance
        if(as_.isPreemptRequested() || travel_distance == 0 )
        {
          ROS_WARN("goal cancelled!");
          //g_move_speed=0.0;
          result_.output = -1;
          as_.setAborted(result_); // tell the client we have given up on this goal; send the result message as well
          return; // done with callback
 		}
        
        }
        result_.output = 0; //value should be zero, if completed countdown
        as_.setSucceeded(result_); // return the "result" message to client, along with "success" status
       
}

//signum function: strip off and return the sign of the argument
double sgn(double x) { if (x>0.0) {return 1.0; }
    else if (x<0.0) {return -1.0;}
    else {return 0.0;}
}

//a function to consider periodicity and find min delta angle
double min_spin(double spin_angle) {
        while (spin_angle>M_PI) {
            spin_angle -= 2.0*M_PI;}
        while (spin_angle< -M_PI) {
            spin_angle += 2.0*M_PI;}
         return spin_angle;   
}            

// a useful conversion function: from quaternion to yaw
double convertPlanarQuat2Phi(geometry_msgs::Quaternion quaternion) {
    double quat_z = quaternion.z;
    double quat_w = quaternion.w;
    double phi = 2.0 * atan2(quat_z, quat_w); // cheap conversion from quaternion to heading for planar motion
    return phi;
}

//and the other direction:
geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

// a few action functions:
//a function to reorient by a specified angle (in radians), then halt
void do_spin(double spin_ang) {
    ros::Rate loop_timer(1/g_sample_dt);
    double timer=0.0;
    double final_time = fabs(spin_ang)/g_spin_speed;
    g_twist_cmd.angular.z= sgn(spin_ang)*g_spin_speed;
    while(timer<final_time) {
          g_twist_commander.publish(g_twist_cmd);
          timer+=g_sample_dt;
          loop_timer.sleep(); 
          }  
    do_halt(); 
}
void alarmCb(const std_msgs::Bool& alarm_msg) {
    g_alarm = alarm_msg.data;
    if (g_alarm) {
	 do_halt();
     ROS_INFO("alarm!!! %d:", g_alarm);
     //g_goal_active=false;
	}
	g_alarm = false;
}
//a function to move forward by a specified distance (in meters), then halt
void do_move(double distance) { // always assumes robot is already oriented properly
                                // but allow for negative distance to mean move backwards
    ros::Rate loop_timer(1/g_sample_dt);
    double timer=0.0;
    double final_time = fabs(distance)/g_move_speed;
    g_twist_cmd.angular.z = 0.0; //stop spinning
    g_twist_cmd.linear.x = sgn(distance)*g_move_speed;
    while(timer<final_time) {
          g_twist_commander.publish(g_twist_cmd);
          timer+=g_sample_dt;
          loop_timer.sleep(); 
          }  
    do_halt();
}

void do_halt() {
    ros::Rate loop_timer(1/g_sample_dt);   
    g_twist_cmd.angular.z= 0.0;
    g_twist_cmd.linear.x=0.0;
    for (int i=0;i<10;i++) {
          g_twist_commander.publish(g_twist_cmd);
          loop_timer.sleep(); 
          }   
}

void do_inits(ros::NodeHandle &n) {
  //initialize components of the twist command global variable
    g_twist_cmd.linear.x=0.0;
    g_twist_cmd.linear.y=0.0;    
    g_twist_cmd.linear.z=0.0;
    g_twist_cmd.angular.x=0.0;
    g_twist_cmd.angular.y=0.0;
    g_twist_cmd.angular.z=0.0;  
    
    //define initial position to be 0
    g_current_pose.position.x = 0.0;
    g_current_pose.position.y = 0.0;
    g_current_pose.position.z = 0.0;
    
    // define initial heading to be "0"
    g_current_pose.orientation.x = 0.0;
    g_current_pose.orientation.y = 0.0;
    g_current_pose.orientation.z = 0.0;
    g_current_pose.orientation.w = 1.0;
    
    // we declared g_twist_commander as global, but never set it up; do that now that we have a node handle
    g_twist_commander = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);    
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "example_action_server_node"); // name this node 
    ros::NodeHandle n;
    ros::Subscriber lidar_alarm_sub = n.subscribe("lidar_alarm", 1, alarmCb);
  
    // to clean up "main", do initializations in a separate function
    // a poor-man's class constructor
    do_inits(n); //pass in a node handle so this function can set up publisher with it
  
    ExampleActionServer as_object; // create an instance of the class "ExampleActionServer"
    
    ROS_INFO("going into spin");
    // from here, all the work is done in the action server, with the interesting stuff done within "executeCB()"
    // you will see 5 new topics under example_action: cancel, feedback, goal, result, status
    ros::spin();

    return 0;
}

