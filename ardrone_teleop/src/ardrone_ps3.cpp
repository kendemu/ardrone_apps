#include <ros/ros.h>
#include <geometry/Twist.h>
#include <sensor_msgs/Joy.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "ros/console.h"

class ARDroneTeleop
{
public:
    ARDroneTeleop();

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    void publish();

    ros::NodeHandle nh;

    double linearx, lineary, linearz, angular;
    double deadman_axis;
    int linx_axis, liny_axis, linz_axis, ang_axis, deadman_axis;
    int land_axis, emer_axis;
    ros::Publisher velPub, emerPub, landPub, flyPub;
    ros::Subscriber joy_sub;

    geometry_msgs::Twist last_published_;
    boost::mutex publish_mutex_;
    bool deadman_pressed_;
    bool land_pressed;
    bool emer_pressed;
    ros::Timer timer_;

    int state;
};

ARDroneTeleop::ARDroneTeleop():
    linearx(0), lineary(0), linearz(0),
    angular(0),
    linx_axis(0), liny_axis(1), linz_axis(3), ang_axis(2),
    land_axis(14), emer_axis(12),
    state(0),
    deadman_axis(10)
{
    velPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    emerPub = nh.advertise<geometry_msgs::Twist>("ardrone/reset", 1);
    landPub = nh.advertise<geometry_msgs::Twist>("ardrone/land", 1);
    flyPub = nh.advertise<geometry_msgs::Twist>("ardrone/takeoff", 1);

    timer_ = nh.createTimer(ros::Duration(0.03), boost::bind(&TurtlebotTeleop::publish, this));
}

void ARDroneTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    geometry_msgs::Twist vel;
    vel.angular.z = joy->axes[ang_axis];
    vel.linear.x = joy->axes[linx_axis];
    vel.linear.y = joy->axes[liny_axis];
    vel.linear.z = joy->axes[linz_axis];
    last_published_ = vel;
    deadman_pressed_ = joy->buttons[deadman_axis];
    land_pressed = joy->buttons[land_axis];
    emer_pressed = joy->buttons[emer_axis];
}

void ARDroneTeleop::publish()
{
    boost::mutex::scoped_lock lock(publish_mutex_);

    if (emer_pressed) {
        emerPub.publish(std_msgs::Empty());
        if (state == -1)
            state = 0;
        else
            state = -1;
    }
    if (deadman_pressed_) {
        if (land_pressed) {
            if (state == 0) {
                flyPub.publish(std_msgs::Empty());
                state = 1;
            } else if (state == 1) {
                landPub.publish(std_msgs::Empty());
                state = 0;
            }
        }
        vel_pub.publish(last_published_);
    }
} 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ardrone_teleop_joy");
    ARDroneTeleop ardrone_teleop;

    ros::spin();
}
