#ifndef YOUBOT_H
#define YOUBOT_H

#include <iostream>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>

#include <decision_making/FSM.h>
#include <decision_making/ROSTask.h>
#include <decision_making/DecisionMaking.h>

using namespace std;
using namespace decision_making;

#define foreach BOOST_FOREACH

FSM_HEADER(Youbot)

#endif // YOUBOT_H
