#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h>
#include <math.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void cloud_cb (const geometry_msgs::PoseStampedConstPtr& input){

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(input->pose.orientation, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    geometry_msgs::Quaternion quat_turned =  tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw + M_PI);

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = input->pose.position.x;
    goal.target_pose.pose.position.y = input->pose.position.y;
    goal.target_pose.pose.position.z = input->pose.position.z;
    goal.target_pose.pose.orientation.w = quat_turned.w;
    goal.target_pose.pose.orientation.x = quat_turned.x;
    goal.target_pose.pose.orientation.y = quat_turned.y;
    goal.target_pose.pose.orientation.z = quat_turned.z;

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);
}

int main (int argc, char** argv)
{
    ros::init (argc, argv, "send_goal");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe ("/move_base_simple/goal", 1, cloud_cb);

    ros::spin ();
}
