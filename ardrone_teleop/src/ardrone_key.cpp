/*
 * Copyright (c) 2012, Rachel Brindle
 * All rights reserved.
 *
 * Released under a BSD license
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42

class ARDroneTeleop
{
public:
    ARDroneTeleop();
    void keyLoop();
    void watchdog();

private:
    ros::NodeHandle nh;
    double linearx, lineary, linearz, angular;
    ros::Time firstPublish;
    ros::Time lastPublish;
    double lScale, aScale;
    char state; // -1 = emergency, 0 = landed, 1 = flying
    char linzplus, linzmin, angzplus, angzmin;
    ros::Publisher velPub;
    ros::Publisher emerPub;
    ros::Publisher landPub;
    ros::Publisher flyPub;
    void publish(double, double, double, double, char, char);
    boost::mutex publishMutex;
};

char GetRosParam(char *param, char defaultVal) {
    std::string name(param);
    char res, ret;
    ret = (ros::param::get(name, res)) ? res : defaultVal;
    ROS_DEBUG("SET %-30s: %02x", param, ret);
    return ret;
}

ARDroneTeleop::ARDroneTeleop():
    linearx(0),
    lineary(0),
    linearz(0),
    angular(0),
    state(0),
    lScale(1.0),
    aScale(1.0)
{
    linzplus = GetRosParam("~linear_z_control+", 'w');
    linzmin = GetRosParam("~linear_z_control-", 's');
    angzplus = GetRosParam("~linear_z_control+", 'd');
    angzmin = GetRosParam("~linear_z_control-", 'a');
    velPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    emerPub = nh.advertise<std_msgs::Empty>("ardrone/reset", 1);
    landPub = nh.advertise<std_msgs::Empty>("ardrone/land", 1);
    flyPub = nh.advertise<std_msgs::Empty>("ardrone/takeoff", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ardrone_teleop_key");
    ARDroneTeleop ardroneTeleop;
    ros::NodeHandle n;
    
    signal(SIGINT, quit);

    boost::thread arthread(boost::bind(&ARDroneTeleop::keyLoop, &ardroneTeleop));

    ros::Timer timer = n.createTimer(ros::Duration(0.03), boost::bind(&ARDroneTeleop::watchdog, &ardroneTeleop));
    ros::spin();

    arthread.interrupt();
    arthread.join();

    return(0);
}

void ARDroneTeleop::watchdog()
{
    boost::mutex::scoped_lock lock(publishMutex);
    if ((ros::Time::now() > lastPublish + ros::Duration(0.05)) &&
        (ros::Time::now() > firstPublish + ros::Duration(0.15)))
        publish(0, 0, 0, 0, 0, 0);
}

void ARDroneTeleop::keyLoop()
{
    char c;

    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // setting a new line, then end of file...
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move the AR Drone");

    while(ros::ok()) {
        if (read(kfd, &c, 1) < 0) {
            perror("read():");
            exit(-1);
        }

        linearx = lineary = linearz = angular = 0;
        ROS_DEBUG("value: 0x%02X\n", c);
        char pastState = state;

        switch (c) {
            case KEYCODE_L:
                ROS_DEBUG("LEFT");
                lineary=1.0;
                break;
            case KEYCODE_R:
                ROS_DEBUG("RIGHT");
                lineary=-1.0;
                break;
            case KEYCODE_U:
                ROS_DEBUG("FORWARD");
                linearx=1.0;
                break;
            case KEYCODE_D:
                ROS_DEBUG("BACK");
                linearx=-1.0;
                break;
            case 0x2e: // '.' dvorak...
                ROS_DEBUG("UP");
                linearz=1.0;
                break;
            case 0x65: // 'e' dvorak...
                ROS_DEBUG("DOWN");
                linearz=-1.0;
                break;
            case 0x6f: // 'o' dvorak...
                ROS_DEBUG("TURN LEFT");
                angular=-1.0;
                break;
            case 0x75: // 'u' dvorak...
                ROS_DEBUG("TURN RIGHT");
                angular=1.0;
                break;
            case 0x20:
                if (state == -1) {
                    ROS_DEBUG("EMERGENCY");
                    state = 0;
                } else if (state == 0) {
                    ROS_DEBUG("TAKE OFF");
                    state = 1;
                } else if (state == 1) { // flying
                    ROS_DEBUG("LAND");
                    state = 0;
                }
                break;
/*
            case 0x65:
                ROS_DEBUG("EMERGENCY");
                state = -1;
                break;
*/
        }
        //fprintf(stderr, "DEBUG: remove 'continue' that prevents messages from being sent.\n");
        boost::mutex::scoped_lock lock(publishMutex);
        if (ros::Time::now() > lastPublish + ros::Duration(1.0))
            firstPublish = ros::Time::now();
        lastPublish = ros::Time::now();
        publish(angular, linearx, lineary, linearz, state, pastState);
    }
    return;
}

void ARDroneTeleop::publish(double ang, double lix, double liy, double liz, char st, char past)
{
    if (st != past) {
        switch (st) {
            case -1:
                emerPub.publish(std_msgs::Empty());
                break;
            case 0:
                landPub.publish(std_msgs::Empty());
                break;
            case 1:
                flyPub.publish(std_msgs::Empty());
                break;
        }
    } else {
        geometry_msgs::Twist vel;
        vel.linear.x = lix;
        vel.linear.y = liy;
        vel.linear.z = liz;
        vel.angular.z = ang;

        velPub.publish(vel);
    }
    return;
}
