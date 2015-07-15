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
#include "opencv_detect_squares/GetObjects.h"
#include "cluster_pointcloud/get_barrels.h"

using namespace std;

#define foreach BOOST_FOREACH

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct taged_pose{
    geometry_msgs::PoseStamped pose;
    bool safe;
    int fail_counter;
};

struct object_on_robot{
    // 1: left, 2: middle, 3: right
    int position_on_robot;
    // red, yellow, green
    string color;
    // none, explosive, fire, toxic, unkown
    string ghs;
};

ros::NodeHandle* node;

// Sleeptimes
ros::Duration five_seconds(5, 0);
ros::Duration three_seconds(3, 0);
ros::Duration one_second(1, 0);

// Publishers
ros::Publisher init_pub;
ros::Publisher brics_pub;
ros::Publisher cmd_pub;
ros::Publisher test_pub;

// Services
ros::ServiceClient detection_client;
ros::ServiceClient laser_client;

// Poses
geometry_msgs::PoseStamped nav_goal;
geometry_msgs::PoseArray standardPoses;
geometry_msgs::PoseWithCovarianceStamped initialpose;
geometry_msgs::PoseStamped container_pose;
geometry_msgs::PoseStamped truck_pose;

// Vectors
vector<taged_pose> points_of_interest;
vector<object_on_robot> objects_on_robot;

boost::shared_ptr<MoveBaseClient> my_movebase;
boost::shared_ptr<moveit::planning_interface::MoveGroup> my_moveit;

int current_index;
int next_place_on_robot = 1;
int next_place_on_truck = 1;
int max_fail_counter = 3;
double distanceFromObject = 0.6;
double objectThreshold = 0.2;
double min_pick_distance = 0.8;
bool unloading = false;

// #####################################################################

bool moveToNamedTarget(string target){
    my_moveit->setNamedTarget(target);
    return my_moveit->move();
}

bool driveForward() {

    geometry_msgs::Twist forward;
    for (int i = 0; i < 2; ++i) {
        forward.linear.x = 0.1;

        cmd_pub.publish(forward);

        one_second.sleep();

    }

    forward.linear.x = 0.0;

    cmd_pub.publish(forward);

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
        if(distance < objectThreshold){
            return i;
        }
    }
    return -1;
}

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

    bool in_polygon = (y <= (m1 * x + b1) && y >= (m3 * x + b3) && x <= ((y - b2) / m2) && x >= ((y - b4) / m4));

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

int choose_next_target(){
    static tf::TransformListener tf_listener;
    ros::Time now = ros::Time::now();

    geometry_msgs::PoseStamped object_pose;

    object_pose.header.frame_id = "/base_footprint";
    object_pose.pose.orientation.w = 1.0;

    float distance = 100000000;
    float tem_distance = 0;
    float x ;
    float y;
    int index = -1;

    for(int i = 0; i < points_of_interest.size(); i++){

        tf_listener.waitForTransform("/base_footprint", "/map", now, ros::Duration(3.0));
        tf_listener.transformPose("/base_footprint", points_of_interest[i].pose, object_pose);

        x = object_pose.pose.position.x;
        y = object_pose.pose.position.y;

        if(points_of_interest[i].safe && points_of_interest[i].fail_counter < max_fail_counter){
            tem_distance = sqrt(pow(x, 2) + pow(y,2));

            // TODO
            // An object that could not get picked should not be tried immediately again
            if(tem_distance < distance && tem_distance > min_pick_distance){
                distance = tem_distance;
                index = i;
            }
        }
    }
    return index;
}

geometry_msgs::PoseStamped calculateDesiredPosition(double x_pre, double y_pre, bool turned){

    geometry_msgs::PoseStamped result;
    geometry_msgs::PoseStamped target_robot_base_link;

    static tf::TransformListener tf_listener;
    ros::Time now = ros::Time::now();

    double h = sqrt(pow(x_pre,2) + pow(y_pre,2)) - distanceFromObject;
    double alpha;
    if(x_pre > 0 && y_pre > 0){
        alpha = atan(y_pre / x_pre);
    } else if(x_pre > 0 && y_pre < 0){
        alpha = atan(y_pre / x_pre) + 2*M_PI;
    } else if(x_pre < 0) {
        alpha = atan(y_pre / x_pre) + M_PI;
    } else if(x_pre == 0 && y_pre > 0){
        alpha = M_PI / 2;
    } else if(x_pre == 0 && y_pre < 0){
        3*M_PI / 2;
    }
    double y = h * sin(alpha);
    double x = h * cos(alpha);

    target_robot_base_link.pose.position.x = x;
    target_robot_base_link.pose.position.y = y;

    if(turned){
        alpha += M_PI;
    }

    geometry_msgs::Quaternion quat =  tf::createQuaternionMsgFromRollPitchYaw(0, 0, alpha);

    target_robot_base_link.pose.orientation = quat;
    target_robot_base_link.header.frame_id = "/base_footprint_turned";

    tf_listener.waitForTransform("/map", "/base_footprint_turned", now, ros::Duration(3.0));
    tf_listener.transformPose("/map", target_robot_base_link, result);

    return result;
}

void refreshPOI(){
    one_second.sleep();

    cluster_pointcloud::get_barrels laser_srv;
    if(laser_client.call(laser_srv)){

        string frame = laser_srv.response.barrel_list.header.frame_id;
        foreach (geometry_msgs::Pose current, laser_srv.response.barrel_list.poses) {
            if(!out_of_map(current)){
                int index = contains(points_of_interest, current);
                taged_pose new_pose;
                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = frame;
                pose.pose.position = current.position;
                pose.pose.orientation.w = 1.0;
                new_pose.pose = pose;
                if(index != -1){
                    new_pose.safe = points_of_interest[index].safe;
                    points_of_interest[index] = new_pose;
                } else {
                    new_pose.safe = true;
                    points_of_interest.push_back(new_pose);
                }
            }
        }
    }

    one_second.sleep();

    geometry_msgs::PoseArray test;
    foreach (taged_pose cur, points_of_interest) {
        test.poses.push_back(cur.pose.pose);
    }
    test.header.frame_id = "/map";
    test_pub.publish(test);

    one_second.sleep();
}

// #####################################################################

decision_making::TaskResult initialize(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {

    init_pub.publish(initialpose);

    one_second.sleep();

    if(moveToNamedTarget("drive")){
        e.riseEvent("/DONE");
    }
    return decision_making::TaskResult::SUCCESS();
}

decision_making::TaskResult analyzeScan(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {

    refreshPOI();

    taged_pose current_target;
    geometry_msgs::PoseStamped target_map;
    geometry_msgs::PoseStamped target_base_link;

    current_index = choose_next_target();
    if(current_index < 0){
        // No object found in scan
        e.riseEvent("/NOTHING_FOUND");
        return decision_making::TaskResult::SUCCESS();
    }
    current_target = points_of_interest[current_index];

    static tf::TransformListener tf_listener;
    ros::Time now = ros::Time::now();

    // Transform next map position into base_footprint_turned
    target_map.pose.position.x = current_target.pose.pose.position.x;
    target_map.pose.position.y = current_target.pose.pose.position.y;
    target_map.pose.position.z = 0;
    target_map.pose.orientation.w = 1.0;
    target_map.header.frame_id = current_target.pose.header.frame_id;

    tf_listener.waitForTransform("/base_footprint_turned", "/map", now, ros::Duration(3.0));
    tf_listener.transformPose("/base_footprint_turned", target_map, target_base_link);

    nav_goal = calculateDesiredPosition(target_base_link.pose.position.x, target_base_link.pose.position.y, true);

    one_second.sleep();
    e.riseEvent("/TARGET_FOUND");
    return decision_making::TaskResult::SUCCESS();
}


decision_making::TaskResult drive(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {

    moveToNamedTarget("drive");
    sendGoalToMovebase();

    my_movebase->waitForResult();

    if(my_movebase->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        e.riseEvent("/TARGET_REACHED");
        return decision_making::TaskResult::SUCCESS();
    } else{
        points_of_interest[current_index].fail_counter++;
        e.riseEvent("/DRIVING_FAILED");
        return decision_making::TaskResult::SUCCESS();
    }
    return decision_making::TaskResult::FAIL();
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
        points_of_interest[current_index].fail_counter++;
        e.riseEvent("/TURNING_FAILED");
        return decision_making::TaskResult::SUCCESS();
    }

    return decision_making::TaskResult::FAIL();
}


decision_making::TaskResult detection(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {

    if(moveToNamedTarget("search_front")){
        opencv_detect_squares::GetObjects detection_srv;
        if(detection_client.call(detection_srv)){
            opencv_detect_squares::DetectedObject detected_object = detection_srv.response.result.objects[0];
            nav_goal = calculateDesiredPosition(detected_object.pose.position.x, detected_object.pose.position.y, false);
            object_on_robot cur;
            cur.color = detected_object.color;
            cur.ghs = detected_object.ghs;
            if(strcmp(cur.ghs.c_str(), "explosive") == 0){
                points_of_interest[current_index].safe = false;
                e.riseEvent("/DANGER");
                return decision_making::TaskResult::SUCCESS();
            }

            cur.position_on_robot = next_place_on_robot;
            objects_on_robot.push_back(cur);

            sendGoalToMovebase();

            my_movebase->waitForResult();

            if(my_movebase->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                e.riseEvent("/SAFE");
                return decision_making::TaskResult::SUCCESS();
            } else {
                points_of_interest[current_index].fail_counter++;
                e.riseEvent("/MOVE_CORRECTION_FAILED");
                return decision_making::TaskResult::SUCCESS();
            }
        }
    }
    return decision_making::TaskResult::FAIL();
}

decision_making::TaskResult driveAround(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {

    if(moveToNamedTarget("drive")){

        geometry_msgs::Pose pose;
        foreach (pose, standardPoses.poses) {
            nav_goal.pose.position = pose.position;
            nav_goal.pose.orientation = pose.orientation;

            sendGoalToMovebase();

            my_movebase->waitForResult();

            if(my_movebase->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                refreshPOI();
                if(points_of_interest.size() > 0){
                    e.riseEvent("/NEW_OBJECT");
                    return decision_making::TaskResult::SUCCESS();
                }
            } else {
                ROS_INFO("Could not reach standard pose for drive_around!");
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
                if(driveForward()){
                    one_second.sleep();
                    closeGripper();
                    if(moveToNamedTarget("safety_front")){
                        if(moveToNamedTarget("place_choose_"+ boost::lexical_cast<std::string>(next_place_on_robot))){
                            one_second.sleep();
                            if(moveToNamedTarget("place_choose_"+ boost::lexical_cast<std::string>(next_place_on_robot))){
                                one_second.sleep();
                                if(moveToNamedTarget("place_final_"+ boost::lexical_cast<std::string>(next_place_on_robot))){
                                    one_second.sleep();
                                    openGripper();
                                    if(moveToNamedTarget("place_choose_"+ boost::lexical_cast<std::string>(next_place_on_robot))){
                                        next_place_on_robot++;
                                        points_of_interest.erase(points_of_interest.begin()+current_index);
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
    return decision_making::TaskResult::FAIL();
}

decision_making::TaskResult analyzeLoad(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {
    if(objects_on_robot.size() == 3){
        unloading = true;
    } else if(objects_on_robot.size() == 0){
        unloading = false;
    }
    if(unloading){
        bool truck_only = true;
        foreach (object_on_robot c, objects_on_robot) {
            if(strcmp(c.ghs.c_str(), "none")){
                truck_only = false;
            }
        }
        if(truck_only){
            e.riseEvent("/GO_TRUCK");
        } else {
            e.riseEvent("/GO_CONTAINER");
        }
    } else {
        e.riseEvent("/EMPTY");
    }
    return decision_making::TaskResult::SUCCESS();

}

decision_making::TaskResult driveToTruck(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {

    if(moveToNamedTarget("drive")){

        nav_goal = truck_pose;

        sendGoalToMovebase();

        my_movebase->waitForResult();

        if(my_movebase->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            e.riseEvent("/TRUCK_REACHED");
            return decision_making::TaskResult::SUCCESS();
        } else {
            ROS_INFO("Could not reach truck pose!");
        }
    }
    return decision_making::TaskResult::FAIL();
}

decision_making::TaskResult placeToTruck(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {

    vector<int> to_remove;
    if(moveToNamedTarget("safety_back")){
        for (int i = 0; i < objects_on_robot.size(); i++) {
            object_on_robot cur = objects_on_robot[i];
            if(strcmp(cur.ghs.c_str(), "none") != 0){
                if(moveToNamedTarget("place_choose_"+ boost::lexical_cast<std::string>(cur.position_on_robot))){
                    one_second.sleep();
                    if(moveToNamedTarget("place_choose_"+ boost::lexical_cast<std::string>(cur.position_on_robot))){
                        one_second.sleep();
                        if(moveToNamedTarget("place_final_"+ boost::lexical_cast<std::string>(cur.position_on_robot))){
                            one_second.sleep();
                            closeGripper();
                            one_second.sleep();
                            if(moveToNamedTarget("place_truck_"+ boost::lexical_cast<std::string>(next_place_on_truck))){
                                next_place_on_truck++;
                                to_remove.push_back(i);
                                one_second.sleep();
                                openGripper();
                                one_second.sleep();
                            }
                        }
                    }
                }
            }
        }
    }
    for (int i = to_remove.size()-1; i >= 0; i--) {
        objects_on_robot.erase(objects_on_robot.begin()+i);
    }
    if(moveToNamedTarget("drive")){
        e.riseEvent("/PLACED_TRUCK");
        return decision_making::TaskResult::SUCCESS();
    }
    return decision_making::TaskResult::FAIL();
}

decision_making::TaskResult driveToContainer(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {

    if(moveToNamedTarget("drive")){
        nav_goal = container_pose;

        sendGoalToMovebase();

        my_movebase->waitForResult();

        if(my_movebase->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            e.riseEvent("/TRUCK_REACHED");
            return decision_making::TaskResult::SUCCESS();
        } else {
            ROS_INFO("Could not reach container pose!");
        }
    }
    return decision_making::TaskResult::FAIL();
}

decision_making::TaskResult placeToContainer(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {

    vector<int> to_remove;
    if(moveToNamedTarget("safety_back")){
        for (int i = 0; i < objects_on_robot.size(); i++) {
            object_on_robot cur = objects_on_robot[i];
            if(strcmp(cur.ghs.c_str(), "none") == 0){
                if(moveToNamedTarget("place_choose_"+ boost::lexical_cast<std::string>(cur.position_on_robot))){
                    one_second.sleep();
                    if(moveToNamedTarget("place_choose_"+ boost::lexical_cast<std::string>(cur.position_on_robot))){
                        one_second.sleep();
                        if(moveToNamedTarget("place_final_"+ boost::lexical_cast<std::string>(cur.position_on_robot))){
                            one_second.sleep();
                            closeGripper();
                            one_second.sleep();
                            if(strcmp(cur.color.c_str(), "red") == 0){
                                if(moveToNamedTarget("place_container_r")){
                                    one_second.sleep();
                                    openGripper();
                                    to_remove.push_back(i);
                                }
                            } else if(strcmp(cur.color.c_str(), "yellow") == 0){
                                if(moveToNamedTarget("place_container_m")){
                                    one_second.sleep();
                                    openGripper();
                                    to_remove.push_back(i);
                                }
                            } else if(strcmp(cur.color.c_str(), "green") == 0){
                                if(moveToNamedTarget("place_container_l")){
                                    one_second.sleep();
                                    openGripper();
                                    to_remove.push_back(i);
                                }
                            }
                            one_second.sleep();
                        }
                    }
                }
            }
        }
    }
    for (int i = to_remove.size()-1; i >= 0; i--) {
        objects_on_robot.erase(objects_on_robot.begin()+i);
    }
    if(moveToNamedTarget("drive")){
        e.riseEvent("/PLACED_CONTAINER");
    }
    return decision_making::TaskResult::SUCCESS();
}

decision_making::TaskResult stop(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {
    return decision_making::TaskResult::SUCCESS();
}

int main(int argc, char **argv) {

    // Set poses for container and truck
    container_pose.header.frame_id = "/map";
    container_pose.pose.position.x = -1.812;
    container_pose.pose.position.y = -1.442;
    container_pose.pose.orientation.z = -0.666;
    container_pose.pose.orientation.w = 0.746;

    truck_pose.header.frame_id = "/map";
    truck_pose.pose.position.x = -0.593;
    truck_pose.pose.position.y = -1.130;
    truck_pose.pose.orientation.z = -0.381;
    truck_pose.pose.orientation.w = 0.925;

    // Set standardposes for driving around
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
    test_pub = node->advertise<geometry_msgs::PoseArray> ("/test", 1);

    // Register services
    detection_client = node->serviceClient<opencv_detect_squares::GetObjects>("/getCVObjects");
    laser_client = node->serviceClient<cluster_pointcloud::get_barrels>("/get_barrel_list");

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
