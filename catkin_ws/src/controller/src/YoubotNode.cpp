#include <iostream>
#include <time.h>
#include <numeric>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/date_time.hpp>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <std_msgs/Float64.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

#include "Youbot.h"

using namespace std;

#define foreach BOOST_FOREACH

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

ros::NodeHandle* node;
geometry_msgs::PoseStamped nav_goal;
ros::Publisher pub;
ros::Duration five_seconds(5, 0);
geometry_msgs::PoseArray objects;
geometry_msgs::PoseArray standardPoses;

boost::shared_ptr<MoveBaseClient> ac;

decision_making::TaskResult initialize(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {

    double roll = 0;
    double pitch = 0;
    double yaw = 0;

    geometry_msgs::Quaternion quat =  tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

//    geometry_msgs::Pose pose1;
//    pose1.position.x = -0.34;
//    pose1.position.y = 0;
//    pose1.position.z = 0;
//    pose1.orientation = quat;
//    standardPoses.poses.push_back(pose1);

//    geometry_msgs::Pose pose2;
//    pose2.position.x = 1.0;
//    pose2.position.y = -0.4;
//    pose2.position.z = 0;
//    pose2.orientation = quat;
//    standardPoses.poses.push_back(pose2);

//    geometry_msgs::Pose pose3;
//    pose3.position.x = 0.76;
//    pose3.position.y = 1.47;
//    pose3.position.z = 0;
//    pose3.orientation = quat;
//    standardPoses.poses.push_back(pose3);

    geometry_msgs::Pose pose4;
    pose4.position.x = -1.45;
    pose4.position.y = 0.42;
    pose4.position.z = 0;
    pose4.orientation = quat;
    standardPoses.poses.push_back(pose4);


    geometry_msgs::PoseWithCovarianceStamped initialpose;
    initialpose.header.frame_id = "/map";
    initialpose.header.stamp = ros::Time::now();
    initialpose.pose.pose.position.x = 0.00250330683775;
    initialpose.pose.pose.position.y = -1.47826063633;
    initialpose.pose.pose.position.z = 0.0;
    initialpose.pose.pose.orientation.x = 0.0;
    initialpose.pose.pose.orientation.y = 0.0;
    initialpose.pose.pose.orientation.z = 0.707477092743;
    initialpose.pose.pose.orientation.w = -0.706736326218;

    double covariance[36] = {0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942};
    for (int i = 0; i < 36; i++) {
        initialpose.pose.covariance[i] = 0.0;
    }

    pub.publish(initialpose);

    five_seconds.sleep();

    e.riseEvent("/DONE");
    return decision_making::TaskResult::SUCCESS();
}

decision_making::TaskResult analyzeScan(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {

    e.riseEvent("/NOTHING_FOUND");
    //    nav_goal.pose.position.x = -0.34;
    //    nav_goal.pose.position.y = 0;
    //    nav_goal.pose.position.z = 0;
    //    double roll = 0;
    //    double pitch = 0;
    //    double yaw = 0;

    //    geometry_msgs::Quaternion quat =  tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

    //    nav_goal.pose.orientation = quat;

    //    five_seconds.sleep();
    //    e.riseEvent("/TARGET_FOUND");
    return decision_making::TaskResult::SUCCESS();

}

void sendGoal(){

    move_base_msgs::MoveBaseGoal goal;

    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(nav_goal.pose.orientation, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    geometry_msgs::Quaternion quat_turned =  tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw + M_PI);

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position = nav_goal.pose.position;
    goal.target_pose.pose.orientation = quat_turned;

    ROS_INFO("Sending goal");
    ac->sendGoal(goal);
}

decision_making::TaskResult drive(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {

    sendGoal();

    ac->waitForResult();

    if(ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        e.riseEvent("/TARGET_REACHED");
        return decision_making::TaskResult::SUCCESS();
    }
    else{
        return decision_making::TaskResult::FAIL();
    }

    return decision_making::TaskResult::SUCCESS();
}

decision_making::TaskResult turnToTarget(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {

    tf::Quaternion quat;
    tf::quaternionMsgToTF(nav_goal.pose.orientation, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    geometry_msgs::Quaternion quat_turned =  tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw + M_PI);

    nav_goal.pose.orientation = quat_turned;

    sendGoal();

    ac->waitForResult();

    if(ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        e.riseEvent("/TURNED");
        return decision_making::TaskResult::SUCCESS();
    }
    else{
        return decision_making::TaskResult::FAIL();
    }

    return decision_making::TaskResult::SUCCESS();
}

decision_making::TaskResult pickUp(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {
    return decision_making::TaskResult::SUCCESS();
}

decision_making::TaskResult detection(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {
    return decision_making::TaskResult::SUCCESS();
}

decision_making::TaskResult driveAround(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {

    geometry_msgs::Pose pose;
    foreach (pose, standardPoses.poses) {
        nav_goal.pose.position = pose.position;
        nav_goal.pose.orientation = pose.orientation;

        sendGoal();

        while(ac->getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
            if(objects.poses.size() > 0){
                e.riseEvent("/NEW_OBJECT");
            }
        }
    }

    if(objects.poses.size() == 0){
        e.riseEvent("/TIMEOUT");
    }

    return decision_making::TaskResult::SUCCESS();
}

decision_making::TaskResult analyzeLoad(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {
    return decision_making::TaskResult::SUCCESS();
}

decision_making::TaskResult driveToTruck(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {
    return decision_making::TaskResult::SUCCESS();
}

decision_making::TaskResult placeToTruck(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {
    return decision_making::TaskResult::SUCCESS();
}

decision_making::TaskResult driveToContainer(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {
    return decision_making::TaskResult::SUCCESS();
}

decision_making::TaskResult placeToContainer(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {
    return decision_making::TaskResult::SUCCESS();
}

decision_making::TaskResult stop(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {
    return decision_making::TaskResult::SUCCESS();
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "youbot_node");
    ros_decision_making_init(argc, argv);

    ROS_INFO("Preparing youbot...");

    boost::posix_time::ptime epoch(boost::posix_time::min_date_time);
    boost::posix_time::ptime now(boost::posix_time::microsec_clock::local_time());
    unsigned int seed = (now - epoch).total_nanoseconds();
    srand(seed);
    node = new ros::NodeHandle();

    pub = node->advertise<geometry_msgs::PoseWithCovarianceStamped> ("/initialpose", 1);

    boost::shared_ptr<MoveBaseClient> temp(new MoveBaseClient("move_base", true));
    //wait for the action server to come up
    while(!temp->waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    ac = temp;

    RosEventQueue* q = new RosEventQueue();

    LocalTasks::registrate("Initialize", initialize);
    LocalTasks::registrate("Drive", drive);
    LocalTasks::registrate("AnalyzeScan", analyzeScan);
    LocalTasks::registrate("TurnToTarget", turnToTarget);
    LocalTasks::registrate("PickUp", pickUp);
    LocalTasks::registrate("Detection", detection);
    LocalTasks::registrate("DriveAround", driveAround);
    LocalTasks::registrate("AnalyzeLoad", analyzeLoad);
    LocalTasks::registrate("DriveToTruck", driveToTruck);
    LocalTasks::registrate("PlaceToTruck", placeToTruck);
    LocalTasks::registrate("DriveToContainer", driveToContainer);
    LocalTasks::registrate("PlaceToContainer", placeToContainer);
    LocalTasks::registrate("Stop", stop);

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ROS_INFO("Spinner started");
    FsmYoubot(NULL, q, "Youbot");

    delete node;
    return 0;
}