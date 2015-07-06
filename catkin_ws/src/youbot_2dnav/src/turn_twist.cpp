#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

ros::Publisher pub;

void
cloud_cb (const geometry_msgs::TwistConstPtr& input)
{
    geometry_msgs::Twist output;
    output.linear.x = - input->linear.x;
    output.linear.y = - input->linear.y;
    output.linear.z = input->linear.z;

    output.angular.x = input->angular.x;
    output.angular.y = input->angular.y;
    output.angular.z = input->angular.z;

    pub.publish(output);
}

int
main (int argc, char** argv)
{
    ros::init (argc, argv, "turn_twist");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe ("/cmd_vel_movebase", 1, cloud_cb);

    pub = nh.advertise<geometry_msgs::Twist> ("/cmd_vel", 1);

    ros::spin ();
}

