// timer_client: works together with action server called "timer_action"
// in source: example_action_server_lab4_w_fdbk.cpp
// this code could be written using classes instead (e.g. like the corresponding server)
//  see: http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Callback%20Based%20Simple%20Action%20Client

#include<ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include<example_action_server_lab4/demoAction.h> //reference action message in this package
#include <example_action_server_lab4/demoAction.h> // this message type is defined in the current package
#include <iostream>
#include <string>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h> // boolean message

using namespace std;

bool g_goal_active = false; //some global vars for communication with callbacks
int g_result_output = -1;
int g_fdbk = -1;
bool g_alarm = false;

// This function will be called once when the goal completes
// this is optional, but it is a convenient way to get access to the "result" message sent by the server
void doneCb(const actionlib::SimpleClientGoalState& state,
        const example_action_server_lab4::demoResultConstPtr& result) {
    ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
    ROS_INFO("got result output = %d",result->output);
    g_result_output= result->output;
    g_goal_active=false;
}

//this function wakes up every time the action server has feedback updates for this client
// only the client that sent the current goal will get feedback info from the action server
void feedbackCb(const example_action_server_lab4::demoFeedbackConstPtr& fdbk_msg) {
    ROS_INFO("feedback status = %d",fdbk_msg->fdbk);
    g_fdbk = fdbk_msg->fdbk; //make status available to "main()"
}

// Called once when the goal becomes active; not necessary, but could be useful diagnostic
void activeCb()
{
  ROS_INFO("Goal just went active");
  g_goal_active=true; //let main() know that the server responded that this goal is in process
}

/*void alarmCb(const std_msgs::Bool& alarm_msg) {
    g_alarm = alarm_msg.data;
    if (g_alarm) {
     ROS_INFO("alarm!!!");
     g_goal_active=false;
  }
}*/

geometry_msgs::Quaternion convertPlanarPhi2Quaternion(double phi) {
    geometry_msgs::Quaternion quaternion;
    quaternion.x = 0.0;
    quaternion.y = 0.0;
    quaternion.z = sin(phi / 2.0);
    quaternion.w = cos(phi / 2.0);
    return quaternion;
}

int main(int argc, char** argv) {
        ros::init(argc, argv, "example_action_client_node"); // name this node 
        ros::NodeHandle n;
        ros::Rate main_timer(1.0);
        ros::init(argc, argv, "commander"); 
        //ros::Publisher twist_commander = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
      
        geometry_msgs::Twist twist_cmd; //this is the message type required to send twist commands to STDR 
        // start with all zeros in the command message; should be the case by default, but just to be safe..
        twist_cmd.linear.x=0.0;
        twist_cmd.linear.y=0.0;    
        twist_cmd.linear.z=0.0;
        twist_cmd.angular.x=0.0;
        twist_cmd.angular.y=0.0;
        twist_cmd.angular.z=0.0;   
  
    double timer=0.0;
    //some "magic numbers"
    double sample_dt = 0.01; //specify a sample period of 10ms  
    double speed = 1.0; // 1m/s speed command
    double yaw_rate = 0.5; //0.5 rad/sec yaw rate command
    double time_3_sec = 3.0; // should move 3 meters or 1.5 rad in 3 seconds
    
    ros::Rate loop_timer(1/sample_dt); //create a ros object from the ros “Rate” class; set 100Hz rate   
    
        // here is a "goal" object compatible with the server, as defined in example_action_server_lab4/action
        example_action_server_lab4::demoGoal goal; 
        geometry_msgs::Quaternion quat;
        
        
        // use the name of our server, which is: timer_action (named in example_action_server_lab4_w_fdbk.cpp)
        // the "true" argument says that we want our new client to run as a separate thread (a good idea)
        actionlib::SimpleActionClient<example_action_server_lab4::demoAction> action_client("path_action", true);
        
        // attempt to connect to the server: need to put a test here, since client might launch before server
        ROS_INFO("attempting to connect to server: ");
        bool server_exists = action_client.waitForServer(ros::Duration(1.0)); // wait for up to 1 second
        // something odd in above: sometimes does not wait for specified seconds, 
        //  but returns rapidly if server not running; so we'll do our own version
        while (!server_exists) { // keep trying until connected
            ROS_WARN("could not connect to server; retrying...");
            server_exists = action_client.waitForServer(ros::Duration(1.0)); // retry every 1 second
        }
        ROS_INFO("connected to action server");  // if here, then we connected to the server;
        
        geometry_msgs::PoseStamped pose_stamped;
        geometry_msgs::Pose pose;
        std::vector<geometry_msgs::PoseStamped> plan;
        nav_msgs::Path paths;
        
        //start sending some zero-velocity commands, just to warm up communications with STDR
    for (int i=0;i<10;i++) {
      //twist_commander.publish(twist_cmd);
      ros::spinOnce();
      loop_timer.sleep();
    }
        
        while(ros::ok()) {
			
	    twist_cmd.angular.z=0.0; // do not spin 
        //twist_cmd.linear.x=speed; //command to move forward
		   
		while(!g_alarm){
		  //twist_commander.publish(twist_cmd);
          timer+=sample_dt;
			
           plan.clear();
           
           pose.position.x = 3.0; // say desired x-coord is 3
           pose.position.y = 0.0;
           pose.position.z = 0.0; // let's hope so!
           pose.orientation.x = 0.0; //always, for motion in horizontal plane
           pose.orientation.y = 0.0; // ditto
           pose.orientation.z = 0.0; // implies oriented at yaw=0, i.e. along x axis
           pose.orientation.w = 1.0; //sum of squares of all components of unit quaternion is 1
           pose_stamped.pose = pose;
           plan.push_back(pose_stamped);
           
           /*pose.position.x = 2.0; 
           pose.position.y = 2.0;
           pose.position.z = 0.0; // let's hope so!
           pose_stamped.pose = pose;
           plan.push_back(pose_stamped);
           
           pose.position.x = 3.0; 
           pose.position.y = 2.0;
           pose.position.z = 0.0; // let's hope so!
           pose_stamped.pose = pose;
           plan.push_back(pose_stamped);
           
           pose.position.x = -3.0; 
           pose.position.y = 3.0;
           pose.position.z = 0.0; // let's hope so
           pose_stamped.pose = pose;
           plan.push_back(pose_stamped);
           
           pose.position.x = -2.0; 
           pose.position.y = 0.0;
           pose.position.z = 0.0; 
           pose_stamped.pose = pose;
           plan.push_back(pose_stamped);
           
           pose.position.x = 0.0; 
           pose.position.y = 2.0;
           pose.position.z = 0.0; 
           pose_stamped.pose = pose;
           plan.push_back(pose_stamped);
           
           pose.position.x = -1.0; 
           pose.position.y = 3.0;
           pose.position.z = 0.0; // let's hope so!
           pose_stamped.pose = pose;
           plan.push_back(pose_stamped);
           
           pose.position.x = -1.8; 
           pose.position.y = 0.1;
           pose.position.z = 0.0; // let's hope so!
           pose_stamped.pose = pose;
           plan.push_back(pose_stamped);*/
           
           paths.poses = plan;
           goal.input = paths;
           action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
           ros::spinOnce();
           loop_timer.sleep();
	   }
	   
	   action_client.cancelGoal();
	   plan.clear();
	   
	   //here if got an alarm; turn CCW until alarm clears
        //twist_cmd.linear.x=0.0; //stop moving forward
       //twist_cmd.angular.z=yaw_rate; //and start spinning left
        
	   
	   pose.position.x = 0.0; 
       pose.position.y = 0.0;
       pose.position.z = 0.0; 
       pose_stamped.pose = pose;
       plan.push_back(pose_stamped);
           
       pose.position.x = 0.0; 
       pose.position.y = 0.0;
       pose.position.z = 0.0; 
       pose_stamped.pose = pose;
       plan.push_back(pose_stamped);
           
       paths.poses = plan;
       ROS_INFO("Stopping!");
       goal.input = paths;
       timer=0.0; //reset the timer
	   
	   /*
	   while(g_alarm){
		   
		   plan.clear();
		   //twist_commander.publish(twist_cmd);
           timer+=sample_dt;
         
		   ROS_INFO("Client entered alarm loop!");
		   pose.position.x = 0.0;
		   pose.position.y = 0.0;
		   pose.position.z = 0.0;
		   pose_stamped.pose = pose;
           
           paths.poses = plan;
           goal.input = paths;
           action_client.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
           
           ros::spinOnce();
           loop_timer.sleep();
		   
	   }*/
           
       }
    return 0;
}

