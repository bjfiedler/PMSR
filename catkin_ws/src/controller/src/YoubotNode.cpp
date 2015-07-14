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
#include <moveit/move_group_interface/move_group.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <brics_actuator/JointPositions.h>
#include "Youbot.h"
#include "tf/transform_listener.h"

using namespace std;

#define foreach BOOST_FOREACH

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

//Pose with tag whether it was processed
struct taged_pose{
    geometry_msgs::PoseStamped pose;
    //true: not yet processed false: processed, dont touch again
    bool tag;
};

struct barrel{
    string color;

    int danger_sign_id ; // = -1 wenn das Fass kein Gefahrenzeichen hat
};



ros::NodeHandle* node;

//Sleeptimes
ros::Duration five_seconds(5, 0);
ros::Duration three_seconds(3, 0);
ros::Duration one_second(1, 0);

// Publishers
ros::Publisher init_pub;
ros::Publisher brics_pub;
ros::Publisher cmd_pub;

geometry_msgs::PoseStamped nav_goal;
geometry_msgs::PoseArray standardPoses;
geometry_msgs::PoseWithCovarianceStamped initialpose;
vector<taged_pose> points_of_interest;
// Vector hat immer die Länge drei. Index sagt aus auf welcher Position der Ladefläche sich das Objekt befindet.
vactor<barrel> youbot_load_area(3);
taged_pose current_target;

boost::shared_ptr<MoveBaseClient> my_movebase;
boost::shared_ptr<moveit::planning_interface::MoveGroup> my_moveit;

int toPick = 0;
int next_place_on_robot = 0;
double distanceFromObject = -0.6;

bool moveToNamedTarget(string target){
    my_moveit->setNamedTarget(target);
    return my_moveit->move();
}

bool driveForward() {

    for (int i = 0; i < 3; ++i) {
        geometry_msgs::Twist forward;
        forward.linear.x = 0.1;

        cmd_pub.publish(forward);

        one_second.sleep();

    }

    return true;
}

void sendGoalToMovebase(){

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
    my_movebase->sendGoal(goal);
}

bool openGripper() {

    brics_actuator::JointPositions pos;
    brics_actuator::JointValue r;
    r.joint_uri = "gripper_finger_joint_r";
    r.unit = "m";
    r.value = 0.001;
    pos.positions.push_back(r);

    brics_actuator::JointValue l;
    l.joint_uri = "gripper_finger_joint_l";
    l.unit = "m";
    l.value = 0.001;
    pos.positions.push_back(l);

    brics_pub.publish(pos);

    three_seconds.sleep();

    return true;
}

bool closeGripper() {

    brics_actuator::JointPositions pos;
    brics_actuator::JointValue r;
    r.joint_uri = "gripper_finger_joint_r";
    r.unit = "m";
    r.value = 0.01;
    pos.positions.push_back(r);

    brics_actuator::JointValue l;
    l.joint_uri = "gripper_finger_joint_l";
    l.unit = "m";
    l.value = 0.01;
    pos.positions.push_back(l);

    brics_pub.publish(pos);

    three_seconds.sleep();

    return true;
}

int contains(vector<taged_pose> points_of_interest, geometry_msgs::Pose current){
    for (int i = 0; i < points_of_interest.size(); ++i) {
        taged_pose poi = points_of_interest[i];
        double distance = sqrt(pow(poi.pose.pose.position.x - current.position.x, 2) + pow(poi.pose.pose.position.y - current.position.y, 2));
        if(distance < 0.1){
            return i;
        }
    }
    return -1;
}

//TODO
bool out_of_map(geometry_msgs::Pose pose){

    float x = pose.position.x;
    float y = pose.position.y;

     float m1 = (2.41 - 1.77) / (-0.56 + 4.19);
     float m2 = (-1.78 - 2.41) / (0.19 + 0.56);
     float m3 = (-2.3 + 1.78) / (-3.47 - 0.19);
     float m4 = (1.77 + 2.3) / (-4.19 + 3.47);

     float b1 =  2.41 - (m1 * -0.56);
     float b2 = 2.41 - (m2 * -0.56);
     float b3 = -2.3 - (m3 * -3.47);
     float b4 = -2.3 - (m4 * -3.47);

     bool in_polygon = (y <= (m1 * x + b1) && y >= (m3 * x + b3) && x <= ((y - b2) / m2) && x >= ((y - b4) / m4);

     m1 = (0.21 - 0.42) / (-1.74 + 0.06);
     m2 = (0.9 - 0.21) / (-1.74 + 1.74);
     m3 = (1.15 + 1.74 ) / (-0.17 + 1.74);
     m4 = (0.42 - 1.15 )/ (-0.06 + 0.17);

     b1 = 0.21 - (m1 * -1.74);
     b2 = 0.21 - (m2 * -1.74);
     b3 = 1.15 - (m3 * -0.17) ;
     b4 = 1.15 - (m4 * -0.17);

     bool out_wall = (y >= (m2 * x + b1) && y <= (m4 * x + b4) && x <= ((y - b1) / m1) && x >= ((y - b2) / m2) );

     m1 = (-2.23 + 2.09) / (-2.15 + 1.35);
     m2 = (-1.83 + 2.23) / (-2.1 + 2.15);
     m3 = (-1.83 + 1.74 ) / (-2.1 + 1.39);
     m4 = (-2.09 + 1.74) / (-1.35 + 1.39);

     b1 = -2.23 - (m1 * -2.15);
     b2 = -2.23 - (m2 * -2.15);
     b3 = -1.75 - (m3 * -1.39) ;
     b4 = -1.75 - (m4 * -1.39);

      bool out_container = (y >= (m2 * x + b1) && y <= (m4 * x + b4) && x <= ((y - b1) / m1) && x >= ((y - b2) / m2) );

             return in_polygon && out_wall && out_container;
}

void poiCallback (const geometry_msgs::PoseArrayConstPtr& input){
    string frame = input->header.frame_id;
    foreach (geometry_msgs::Pose current, input->poses) {
        if(!out_of_map(current)){
            int index = contains(points_of_interest, current);
            taged_pose new_pose;
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = frame;
            pose.pose.position = current.position;
            new_pose.pose = pose;
            new_pose.tag = true;
            if(index != -1){
                points_of_interest[index] = new_pose;
            } else {
                points_of_interest.push_back(new_pose);
            }
        }
    }
}

int choose_next_target(){

   /* Die Funktion wählt das Objekt, das noch nicht angefahren
    * wurde und die kleinste Distanz zu (0, 0 ,0 ) hat.
    */

    geometry_msgs::PoseStamped object_pose;

     object_pose.header.frame_id = "/map";

    float distance = 100000000;
    float tem_distance = 0;
    float x ;
    float y;
    int index;

    for(int i = 0; i < points_of_interest.size(); i++){

         tf_listener.transformPose("/base_link", points_of_interest[i].pose, object_pose);

         x = object_pose.position.x;
         y = object_pose.position.y;

        if(points_of_interest[i].tag){
            tem_distance = sqrt(pow(x, 2) + pow(y,2));

            if(tem_distance < distance){
                distance = tem_distance;
                index = i;
            }
        }
    }
    return index;
}

decision_making::TaskResult initialize(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {

    init_pub.publish(initialpose);

    five_seconds.sleep();

    e.riseEvent("/DONE");
    return decision_making::TaskResult::SUCCESS();
}

decision_making::TaskResult analyzeScan(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {

    current_target = choose_next_target();
    geometry_msgs::PoseStamped target_map;
    geometry_msgs::PoseStamped target_base_link;
    geometry_msgs::PoseStamped target_robot_base_link;

    static tf::TransformListener tf_listener;
    ros::Time now = ros::Time::now();

    // Transform next map position into base_link
    target_map.pose.position.x = current_target.pose.pose.position.x;
    target_map.pose.position.y = current_target.pose.pose.position.y;
    target_map.pose.position.z = 0;
    target_map.pose.orientation.w = 1.0;
    target_map.header.frame_id = current_target.pose.header.frame_id;

    tf_listener.waitForTransform("/base_link", "/map", now, ros::Duration(3.0));
    tf_listener.transformPose("/base_link", target_map, target_base_link);

    // Calculate desired position for robot in base_link
    double alpha = atan(target_base_link.pose.position.y / target_base_link.pose.position.x);
    double x = target_base_link.pose.position.x - cos(alpha) * distanceFromObject;
    double y = target_base_link.pose.position.y - sin(alpha) * distanceFromObject;

    target_robot_base_link.pose.position.x = x;
    target_robot_base_link.pose.position.y = y;

    geometry_msgs::Quaternion quat =  tf::createQuaternionMsgFromRollPitchYaw(0, 0, alpha);

    target_robot_base_link.pose.orientation = quat;
    target_robot_base_link.header.frame_id = "/base_link";

    // Transform desired position for robot into map frame
    tf_listener.waitForTransform("/map", "/base_link", now, ros::Duration(3.0));
    tf_listener.transformPose("/map", target_robot_base_link, nav_goal);

    five_seconds.sleep();
    e.riseEvent("/TARGET_FOUND");
    return decision_making::TaskResult::SUCCESS();
}


decision_making::TaskResult drive(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {

    sendGoalToMovebase();

    my_movebase->waitForResult();

    if(my_movebase->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
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

    sendGoalToMovebase();

    my_movebase->waitForResult();

    if(my_movebase->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        e.riseEvent("/TURNED");
        return decision_making::TaskResult::SUCCESS();
    }
    else{
        return decision_making::TaskResult::FAIL();
    }

    return decision_making::TaskResult::SUCCESS();
}


decision_making::TaskResult detection(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {

    if(moveToNamedTarget("search_front")){

        // OBJECT DETECTION VON BJÖRN HIER

        e.riseEvent("/SAFE");
        return decision_making::TaskResult::SUCCESS();
    }
    else {
        return decision_making::TaskResult::FAIL();
    }

    five_seconds.sleep();
    return decision_making::TaskResult::FAIL();
}

decision_making::TaskResult driveAround(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {

    geometry_msgs::Pose pose;
    foreach (pose, standardPoses.poses) {
        nav_goal.pose.position = pose.position;
        nav_goal.pose.orientation = pose.orientation;

        sendGoalToMovebase();

        while(my_movebase->getState() != actionlib::SimpleClientGoalState::SUCCEEDED){
            if(points_of_interest.size() > 0){
                e.riseEvent("/NEW_OBJECT");
            }
        }
    }

    if(points_of_interest.size() == 0){
        e.riseEvent("/TIMEOUT");
    }

    return decision_making::TaskResult::SUCCESS();
}

decision_making::TaskResult moveItPick(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {

    if(moveToNamedTarget("safety_front")){
        one_second.sleep();
        if(moveToNamedTarget("pick_front")){
            one_second.sleep();
            if(moveToNamedTarget("pick_front")){
                one_second.sleep();
                driveForward();
                one_second.sleep();
                closeGripper();
                if(moveToNamedTarget("safety_front")){
                    if(moveToNamedTarget("safety_back")){
                        if(moveToNamedTarget("place_choose_" + boost::lexical_cast<std::string>(next_place_on_robot))){
                            one_second.sleep();
                            if(moveToNamedTarget("place_choose_" + boost::lexical_cast<std::string>(next_place_on_robot))){
                                one_second.sleep();
                                if(moveToNamedTarget("final" + boost::lexical_cast<std::string>(next_place_on_robot))){
                                    one_second.sleep();
                                    openGripper();
                                    if(moveToNamedTarget("place_choose_" + boost::lexical_cast<std::string>(next_place_on_robot))){
                                        if(moveToNamedTarget("safety_back")){
                                            next_place_on_robot++;
                                            current_target.tag = false;
                                            e.riseEvent("/PICKED");
                                            return decision_making::TaskResult::SUCCESS();
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    return decision_making::TaskResult::FAIL();
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

    //Set standardposes for driving around
    geometry_msgs::Pose pose1;
    pose1.position.x = -0.442;
    pose1.position.y = -1.275;
    pose1.orientation.z = -0.286;
    pose1.orientation.w = 0.958;
    standardPoses.poses.push_back(pose1);

    geometry_msgs::Pose pose2;
    pose2.position.x = -2.305;
    pose2.position.y = -1.468;
    pose2.orientation.z = -0.750;
    pose2.orientation.w = 0.662;
    standardPoses.poses.push_back(pose2);

    geometry_msgs::Pose pose3;
    pose3.position.x = -2.783;
    pose3.position.y = 0.841;
    pose3.orientation.z = 0.865;
    pose3.orientation.w = 0.501;
    standardPoses.poses.push_back(pose3);

    geometry_msgs::Pose pose4;
    pose4.position.x = -0.913;
    pose4.position.y = 1.489;
    pose4.orientation.z = 0.074;
    pose4.orientation.w = 0.997;
    standardPoses.poses.push_back(pose4);

    geometry_msgs::Pose pose5;
    pose5.position.x = -0.750;
    pose5.position.y = -0.012;
    pose5.orientation.z = 0.067;
    pose5.orientation.w = 0.998;
    standardPoses.poses.push_back(pose5);

    // Set initial pose for robot
    initialpose.header.frame_id = "/map";
    initialpose.pose.pose.position.x = 0.238;
    initialpose.pose.pose.position.y = 0.141;
    initialpose.pose.pose.orientation.z = 0.067;
    initialpose.pose.pose.orientation.w = 0.998;

    for (int i = 0; i < 36; i++) {
        initialpose.pose.covariance[i] = 0.0;
    }


    ros::init(argc, argv, "youbot_node");
    ros_decision_making_init(argc, argv);

    boost::posix_time::ptime epoch(boost::posix_time::min_date_time);
    boost::posix_time::ptime now(boost::posix_time::microsec_clock::local_time());
    unsigned int seed = (now - epoch).total_nanoseconds();
    srand(seed);
    node = new ros::NodeHandle();

    // Initialize MoveBase and MoveIt!
    my_movebase = boost::shared_ptr<MoveBaseClient>(new MoveBaseClient("move_base", true));
    while(!my_movebase->waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    moveit::planning_interface::MoveGroup::Options options("arm_1", "robot_description");
    my_moveit = boost::shared_ptr<moveit::planning_interface::MoveGroup>(new moveit::planning_interface::MoveGroup(options));

    // Register publishers
    init_pub = node->advertise<geometry_msgs::PoseWithCovarianceStamped> ("/initialpose", 1);
    brics_pub = node->advertise<brics_actuator::JointPositions> ("/arm_1/gripper_controller/position_command", 1);
    cmd_pub = node->advertise<geometry_msgs::Twist> ("/cmd_vel", 1);

    // Register subscribers
    ros::Subscriber poi_sub = node->subscribe ("/points_of_interest", 1, poiCallback);

    RosEventQueue* q = new RosEventQueue();

    LocalTasks::registrate("Initialize", initialize);
    LocalTasks::registrate("Drive", drive);
    LocalTasks::registrate("AnalyzeScan", analyzeScan);
    LocalTasks::registrate("TurnToTarget", turnToTarget);
    LocalTasks::registrate("Detection", detection);
    LocalTasks::registrate("MoveItPick", moveItPick);
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
