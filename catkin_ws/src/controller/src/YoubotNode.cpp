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
#include "opencv_detect_squares/GetContainerRect.h"
#include "cluster_pointcloud/get_truck.h"
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

struct point{
    float x;
    float y;

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
ros::ServiceClient container_client;
ros::ServiceClient truck_client;

// Poses
geometry_msgs::PoseStamped nav_goal;
geometry_msgs::PoseArray standardPoses;
geometry_msgs::PoseWithCovarianceStamped initialpose;
geometry_msgs::PoseStamped container_pose;
geometry_msgs::PoseStamped truck_pose_1;
//geometry_msgs::PoseStamped truck_pose_2;

// Vectors
vector<taged_pose> points_of_interest;
object_on_robot* objects_on_robot[3];

boost::shared_ptr<MoveBaseClient> my_movebase;
boost::shared_ptr<moveit::planning_interface::MoveGroup> my_moveit;

int current_index;
int next_place_on_robot = 1;
int next_place_on_truck = 1;
int max_fail_counter = 3;
double distanceFromObject = 0.65;
double distanceFromTruck = 0.5;
double distanceFromContainer = -0.55;
double objectThreshold = 0.15;
double min_pick_distance = 0.8;
bool unloading = false;
bool last_target_failed = false;
bool truck_half_full = false;
bool last_unload = false;

// #####################################################################

bool moveToNamedTarget(string target){
    my_moveit->setNamedTarget(target);
    return my_moveit->move();
}

bool driveForward() {

    geometry_msgs::Twist forward;
    forward.linear.x = 0.05;

    ros::Time start = ros::Time::now();
    while(ros::Time::now() - start < ros::Duration(1.5)){
        cmd_pub.publish(forward);
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
    for (int i = 0; i < points_of_interest.size(); i++) {
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

    std::vector<point> polygon;

    point point1;
    point1.x = -4.19;
    point1.y = 1.77;

    point point2;
    point2.x = -0.56;
    point2.y = 2.41;

    point point3;
    point3.x = 0.19;
    point3.y = -1.78;

    point point4;
    point4.x = -2.3;
    point4.y = -3.47;

    polygon.push_back(point1);
    polygon.push_back(point2);
    polygon.push_back(point3);
    polygon.push_back(point4);

    int i;
    int j;
    bool in_polygon = false;
    bool out_wall = false;
    bool out_container = false;

    for (i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
        if ((polygon[i].y > y) != (polygon[j].y > y) &&
                (x < (polygon[j].x - polygon[i].x) * (y - polygon[i].y) / (polygon[j].y-polygon[i].y) + polygon[i].x)) {
            in_polygon = !in_polygon;
        }
    }

    polygon.clear();

    point1.x = -0.06;
    point1.y = 0.42;

    point2.x = -1.74;
    point2.y = 0.21;

    point3.x = -1.74;
    point3.y = 0.9;

    point4.x = -0.17;
    point4.y = 1.15;

    polygon.push_back(point1);
    polygon.push_back(point2);
    polygon.push_back(point3);
    polygon.push_back(point4);

    for (i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
        if ((polygon[i].y > y) != (polygon[j].y > y) &&
                (x < (polygon[j].x - polygon[i].x) * (y - polygon[i].y) / (polygon[j].y-polygon[i].y) + polygon[i].x)) {
            out_wall = !out_wall;
        }
    }

    out_wall = !out_wall;

    polygon.clear();

    point1.x = -1.35;
    point1.y = 2.09;

    point2.x = -2.15;
    point2.y = -2.23;

    point3.x = -2.1;
    point3.y = -1.83;

    point4.x = -1.39;
    point4.y = -1.74;

    polygon.push_back(point1);
    polygon.push_back(point2);
    polygon.push_back(point3);
    polygon.push_back(point4);

    for (i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
        if ((polygon[i].y > y) != (polygon[j].y > y) &&
                (x < (polygon[j].x - polygon[i].x) * (y - polygon[i].y) / (polygon[j].y-polygon[i].y) + polygon[i].x)) {
            out_container = !out_container;
        }
    }

    out_container = !out_container;

    polygon.clear();

    return !(in_polygon && out_container && out_wall);
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

            // If an object pick failed, try to drive further away to try later again from another position
            if(tem_distance < distance && (tem_distance > min_pick_distance || !last_target_failed)){
                distance = tem_distance;
                index = i;
            }
        }
    }
    last_target_failed = false;
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
                    new_pose.fail_counter = points_of_interest[index].fail_counter;
                    points_of_interest[index] = new_pose;
                } else {
                    new_pose.safe = true;
                    new_pose.fail_counter = 0;
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

void turnNavGoalAround(){

    tf::Quaternion quat;
    tf::quaternionMsgToTF(nav_goal.pose.orientation, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    geometry_msgs::Quaternion quat_turned =  tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw + M_PI);

    nav_goal.pose.orientation = quat_turned;

    sendGoalToMovebase();
}

void turnCurrentPoseAround(){

    geometry_msgs::PoseStamped current;
    current.header.frame_id = "/base_footprint_turned";
    geometry_msgs::Quaternion quat_turned =  tf::createQuaternionMsgFromRollPitchYaw(0, 0, M_PI);
    current.pose.orientation = quat_turned;

    static tf::TransformListener tf_listener;
    ros::Time now = ros::Time::now();
    tf_listener.waitForTransform("/map", "/base_footprint_turned", now, ros::Duration(3.0));
    tf_listener.transformPose("/map", current, nav_goal);

    sendGoalToMovebase();
}

// #####################################################################

decision_making::TaskResult initialize(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {

    init_pub.publish(initialpose);

    one_second.sleep();

    if(moveToNamedTarget("drive")){
        e.riseEvent("/DONE");
        return decision_making::TaskResult::SUCCESS();
    }
    return decision_making::TaskResult::FAIL();
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
        last_target_failed = true;
        e.riseEvent("/DRIVING_FAILED");
        return decision_making::TaskResult::SUCCESS();
    }
    return decision_making::TaskResult::FAIL();
}

decision_making::TaskResult turnToTarget(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {

    turnNavGoalAround();

    my_movebase->waitForResult();

    if(my_movebase->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        e.riseEvent("/TURNED");
        return decision_making::TaskResult::SUCCESS();
    }
    else{
        points_of_interest[current_index].fail_counter++;
        last_target_failed = true;
        e.riseEvent("/TURNING_FAILED");
        return decision_making::TaskResult::SUCCESS();
    }

    return decision_making::TaskResult::FAIL();
}


decision_making::TaskResult detection(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {

    opencv_detect_squares::GetObjects detection_srv;
    detection_srv.request.numberOfFrames = 0;
    if(moveToNamedTarget("search_front_top")){

        if(detection_client.call(detection_srv)){
            opencv_detect_squares::DetectedObject detected_object = detection_srv.response.result.objects[0];
            nav_goal = calculateDesiredPosition(detected_object.pose.position.x, detected_object.pose.position.y, false);

            sendGoalToMovebase();

            my_movebase->waitForResult();

            if(my_movebase->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                object_on_robot *pre = new object_on_robot();
                pre->color = detected_object.color;
                pre->ghs = detected_object.ghs;
                pre->position_on_robot = next_place_on_robot;
                if(moveToNamedTarget("search_front")){
                    if(detection_client.call(detection_srv)){
                        opencv_detect_squares::DetectedObject detected_object = detection_srv.response.result.objects[0];
                        nav_goal = calculateDesiredPosition(detected_object.pose.position.x, detected_object.pose.position.y, false);

                        if(strcmp(detected_object.ghs.c_str(), "explosive") == 0){
                            points_of_interest[current_index].safe = false;
                            e.riseEvent("/DANGER");
                            return decision_making::TaskResult::SUCCESS();
                        }

                        sendGoalToMovebase();

                        my_movebase->waitForResult();

                        if(my_movebase->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                            object_on_robot *cur = new object_on_robot();
                            cur->color = detected_object.color;
                            cur->ghs = detected_object.ghs;
                            cur->position_on_robot = next_place_on_robot;
                            objects_on_robot[next_place_on_robot - 1] = cur;
                            e.riseEvent("/SAFE");
                            return decision_making::TaskResult::SUCCESS();
                        } else {
                            points_of_interest[current_index].fail_counter++;
                            last_target_failed = true;
                            e.riseEvent("/MOVE_CORRECTION_FAILED");
                            return decision_making::TaskResult::SUCCESS();
                        }
                    } else {
                        objects_on_robot[next_place_on_robot-1] = pre;
                        e.riseEvent("/SAFE");
                        return decision_making::TaskResult::SUCCESS();
                    }
                }
            } else {
                points_of_interest[current_index].fail_counter++;
                last_target_failed = true;
                e.riseEvent("/MOVE_CORRECTION_FAILED");
                return decision_making::TaskResult::SUCCESS();
            }
        } else {
            points_of_interest[current_index].fail_counter++;
            last_target_failed = true;
            e.riseEvent("/MOVE_CORRECTION_FAILED");
            return decision_making::TaskResult::SUCCESS();
        }
        return decision_making::TaskResult::FAIL();
    }
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
            }
        }
    }

    if(points_of_interest.size() == 0){
        if(next_place_on_robot > 1){
            last_unload = true;
            e.riseEvent("/LAST_UNLOAD");
            return decision_making::TaskResult::SUCCESS();
        } else {
            e.riseEvent("/TIMEOUT");
            return decision_making::TaskResult::SUCCESS();
        }
    }

    return decision_making::TaskResult::FAIL();
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
                        if(moveToNamedTarget("place_final_"+ boost::lexical_cast<std::string>(next_place_on_robot))){
                            one_second.sleep();
                            openGripper();
                            if(moveToNamedTarget("place_choose_"+ boost::lexical_cast<std::string>(next_place_on_robot))){
                                next_place_on_robot++;
                                points_of_interest.erase(points_of_interest.begin()+current_index);
                                if(moveToNamedTarget("safety_back")){
                                    turnNavGoalAround();
                                    my_movebase->waitForResult();
                                    if(my_movebase->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                                        e.riseEvent("/PICKED");
                                        return decision_making::TaskResult::SUCCESS();
                                    }
                                    else{
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
    ROS_INFO("Next Place on Robot: %d", next_place_on_robot);
    if(next_place_on_robot > 3){
        unloading = true;
    } else if(next_place_on_robot == 1){
        unloading = false;
        last_unload = false;
    }
    if(unloading || last_unload){
        bool truck_only = true;
        for (int i = 0; i < 3; i++) {
            if(objects_on_robot[i] != 0){
                ROS_INFO(objects_on_robot[i]->ghs.c_str());
                if(strcmp(objects_on_robot[i]->ghs.c_str(), "none") == 0){
                    truck_only = false;
                }
            }
        }
        if(truck_only){
            e.riseEvent("/GO_TRUCK");
            return decision_making::TaskResult::SUCCESS();
        } else {
            e.riseEvent("/GO_CONTAINER");
            return decision_making::TaskResult::SUCCESS();
        }
    } else {
        e.riseEvent("/EMPTY");
        return decision_making::TaskResult::SUCCESS();
    }
    return decision_making::TaskResult::FAIL();
}

decision_making::TaskResult driveToTruck(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {

    if(moveToNamedTarget("drive")){

        if(truck_half_full){
            distanceFromTruck = 0.7;
        }

        nav_goal = truck_pose_1;

        sendGoalToMovebase();

        my_movebase->waitForResult();

        if(my_movebase->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){

            cluster_pointcloud::get_truck truck_srv;
            if(truck_client.call(truck_srv)){

                geometry_msgs::Pose l = truck_srv.response.truck_pose.poses[0];
                geometry_msgs::Pose r = truck_srv.response.truck_pose.poses[1];

                double x_pre = r.position.x - l.position.x;
                double y_pre = r.position.y - l.position.y;

                double x_vec = y_pre;
                double y_vec = -x_pre;
                double x_st = x_pre * 0.32 + l.position.x;
                double y_st = y_pre * 0.32 + l.position.y;

                // Normierung
                double norm = 1/(sqrt(pow(x_vec,2) + pow(y_vec,2)));
                double x = x_st + distanceFromTruck * x_vec * norm;
                double y = y_st + distanceFromTruck * y_vec * norm;

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

                nav_goal.pose.position.x = x;
                nav_goal.pose.position.y = y;

                geometry_msgs::Quaternion quat =  tf::createQuaternionMsgFromRollPitchYaw(0, 0, alpha+M_PI/2);
                nav_goal.pose.orientation = quat;

                sendGoalToMovebase();

                my_movebase->waitForResult();
                if(my_movebase->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                    e.riseEvent("/TRUCK_REACHED");
                    return decision_making::TaskResult::SUCCESS();
                } else {
                    nav_goal = truck_pose_1;

                    sendGoalToMovebase();

                    my_movebase->waitForResult();

                    if(my_movebase->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                        e.riseEvent("/TRUCK_REACHED");
                        return decision_making::TaskResult::SUCCESS();
                    } else {
                        turnCurrentPoseAround();
                        my_movebase->waitForResult();

                        if(my_movebase->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                            e.riseEvent("/TRY_TRUCK_AGAIN");
                            return decision_making::TaskResult::SUCCESS();
                        } else {
                            nav_goal.pose.position = standardPoses.poses[4].position;
                            nav_goal.pose.orientation = standardPoses.poses[4].orientation;

                            sendGoalToMovebase();

                            my_movebase->waitForResult();

                            if(my_movebase->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                                e.riseEvent("/TRY_TRUCK_AGAIN");
                                return decision_making::TaskResult::SUCCESS();
                            }
                        }
                    }
                }
            } else {
                e.riseEvent("/TRUCK_REACHED");
                return decision_making::TaskResult::SUCCESS();
            }


        } else {
            turnCurrentPoseAround();
            my_movebase->waitForResult();

            if(my_movebase->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                e.riseEvent("/TRY_TRUCK_AGAIN");
                return decision_making::TaskResult::SUCCESS();
            } else {
                nav_goal.pose.position = standardPoses.poses[4].position;
                nav_goal.pose.orientation = standardPoses.poses[4].orientation;

                sendGoalToMovebase();

                my_movebase->waitForResult();

                if(my_movebase->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                    e.riseEvent("/TRY_TRUCK_AGAIN");
                    return decision_making::TaskResult::SUCCESS();
                }
            }
        }
    }
    return decision_making::TaskResult::FAIL();
}

decision_making::TaskResult placeToTruck(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {

    if(moveToNamedTarget("safety_back")){
        for (int i = 0; i < 3; i++) {
            object_on_robot cur = *objects_on_robot[i];
            if(strcmp(cur.ghs.c_str(), "none") != 0){
                if(next_place_on_truck > 3){
                    next_place_on_truck = 1;
                    truck_half_full = true;
                    e.riseEvent("/CORRECT_TRUCK_POSITION");
                    return decision_making::TaskResult::SUCCESS();
                }
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
                                objects_on_robot[i] = 0;
                                next_place_on_robot--;
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
            if(moveToNamedTarget("search_container")){

                opencv_detect_squares::GetContainerRect container_srv;
                container_srv.request.numberOfFrames = 0;

                if(container_client.call(container_srv)){
                    geometry_msgs::PoseArray detected_array = container_srv.response.rect;
                    int smallest_x = -1;
                    int second_smallest_x = -1;
                    double x_cur = 100.0;
                    for (int i = 0; i < 4; ++i) {
                        if(detected_array.poses[i].position.x < x_cur){
                            second_smallest_x = smallest_x;
                            smallest_x = i;
                            x_cur = detected_array.poses[i].position.x;
                        }
                    }

                    geometry_msgs::Pose l;
                    geometry_msgs::Pose r;

                    if(detected_array.poses[smallest_x].position.y < detected_array.poses[second_smallest_x].position.y){
                        l = detected_array.poses[second_smallest_x];
                        r = detected_array.poses[smallest_x];
                    } else {
                        r = detected_array.poses[second_smallest_x];
                        l = detected_array.poses[smallest_x];
                    }

                    double x_pre = r.position.x - l.position.x;
                    double y_pre = r.position.y - l.position.y;

                    double x_vec = y_pre;
                    double y_vec = -x_pre;
                    double x_st = x_pre * 0.5 + l.position.x;
                    double y_st = y_pre * 0.5 + l.position.y;

                    // Normierung
                    double norm = 1/(sqrt(pow(x_vec,2) + pow(y_vec,2)));
                    double x = x_st + distanceFromContainer * x_vec * norm;
                    double y = y_st + distanceFromContainer * y_vec * norm;

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

                    geometry_msgs::Quaternion quat =  tf::createQuaternionMsgFromRollPitchYaw(0, 0, alpha-M_PI/2);
                    geometry_msgs::PoseStamped c;
                    c.pose.position.x = x;
                    c.pose.position.y = y;
                    c.pose.orientation = quat;
                    c.header.frame_id = "/base_footprint_turned";

                    static tf::TransformListener tf_listener;
                    ros::Time now = ros::Time::now();
                    tf_listener.waitForTransform("/map", "/base_footprint_turned", now, ros::Duration(3.0));
                    tf_listener.transformPose("/map", c, nav_goal);

                    sendGoalToMovebase();

                    my_movebase->waitForResult();

                    if(my_movebase->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                        e.riseEvent("/CONTAINER_REACHED");
                        return decision_making::TaskResult::SUCCESS();
                    } else {
                        nav_goal = container_pose;

                        sendGoalToMovebase();

                        my_movebase->waitForResult();

                        if(my_movebase->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                            e.riseEvent("/CONTAINER_REACHED");
                            return decision_making::TaskResult::SUCCESS();
                        } else {
                            turnCurrentPoseAround();
                            my_movebase->waitForResult();

                            if(my_movebase->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                                e.riseEvent("/TRY_CONTAINER_AGAIN");
                                return decision_making::TaskResult::SUCCESS();
                            } else {
                                nav_goal.pose.position = standardPoses.poses[4].position;
                                nav_goal.pose.orientation = standardPoses.poses[4].orientation;

                                sendGoalToMovebase();

                                my_movebase->waitForResult();

                                if(my_movebase->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                                    e.riseEvent("/TRY_CONTAINER_AGAIN");
                                    return decision_making::TaskResult::SUCCESS();
                                }
                            }
                        }
                    }
                } else {
                    e.riseEvent("/CONTAINER_REACHED");
                    return decision_making::TaskResult::SUCCESS();
                }
            }
        } else {
            turnCurrentPoseAround();
            my_movebase->waitForResult();

            if(my_movebase->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                e.riseEvent("/TRY_CONTAINER_AGAIN");
                return decision_making::TaskResult::SUCCESS();
            } else {
                nav_goal.pose.position = standardPoses.poses[4].position;
                nav_goal.pose.orientation = standardPoses.poses[4].orientation;

                sendGoalToMovebase();

                my_movebase->waitForResult();

                if(my_movebase->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                    e.riseEvent("/TRY_CONTAINER_AGAIN");
                    return decision_making::TaskResult::SUCCESS();
                }
            }
        }
    }
    return decision_making::TaskResult::FAIL();
}

decision_making::TaskResult placeToContainer(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {

    if(moveToNamedTarget("safety_back")){
        for (int i = 0; i < 3; i++) {
            object_on_robot cur = *objects_on_robot[i];
            if(strcmp(cur.ghs.c_str(), "none") == 0){
                if(moveToNamedTarget("place_choose_"+ boost::lexical_cast<std::string>(cur.position_on_robot))){
                    one_second.sleep();
                    if(moveToNamedTarget("place_choose_"+ boost::lexical_cast<std::string>(cur.position_on_robot))){
                        one_second.sleep();
                        if(moveToNamedTarget("place_final_"+ boost::lexical_cast<std::string>(cur.position_on_robot))){
                            one_second.sleep();
                            closeGripper();
                            one_second.sleep();
                            string target;
                            if(strcmp(cur.color.c_str(), "red") == 0){
                                target = "place_container_r";
                            } else if(strcmp(cur.color.c_str(), "yellow") == 0){
                                target = "place_container_m";
                            } else if(strcmp(cur.color.c_str(), "green") == 0){
                                target = "place_container_l";
                            }
                            if(moveToNamedTarget(target)){
                                one_second.sleep();
                                openGripper();
                                objects_on_robot[i] = 0;
                                next_place_on_robot--;
                            }
                            one_second.sleep();
                        }
                    }
                }
            }
        }
    }
    if(moveToNamedTarget("drive")){
        e.riseEvent("/PLACED_CONTAINER");
    }
    return decision_making::TaskResult::SUCCESS();
}

decision_making::TaskResult stop(std::string, const decision_making::FSMCallContext& c, decision_making::EventQueue& e) {
    if(moveToNamedTarget("folded")){
        return decision_making::TaskResult::SUCCESS();
    }
}

int main(int argc, char **argv) {

    // Set poses for container and truck
    container_pose.header.frame_id = "/map";
    container_pose.pose.position.x = -1.804;
    container_pose.pose.position.y = -1.271;
    container_pose.pose.orientation.z = -0.657;
    container_pose.pose.orientation.w = 0.754;

    truck_pose_1.header.frame_id = "/map";
    truck_pose_1.pose.position.x = -0.874;
    truck_pose_1.pose.position.y = -0.724;
    truck_pose_1.pose.orientation.z = 0.883;
    truck_pose_1.pose.orientation.w = 0.470;

//    truck_pose_2.header.frame_id = "/map";
//    truck_pose_2.pose.position.x = -0.509;
//    truck_pose_2.pose.position.y = -1.001;
//    truck_pose_2.pose.orientation.z = -0.411;
//    truck_pose_2.pose.orientation.w = 0.912;

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
    container_client = node->serviceClient<opencv_detect_squares::GetContainerRect>("/getContainerRect");
    truck_client = node->serviceClient<cluster_pointcloud::get_truck>("/get_truck");

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
