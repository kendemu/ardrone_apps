/*
 * Copyright (c) 2012, Rachel Brindle
 * All rights reserved.
 *
 * Released under a BSD license
 */

#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

#define DEBUG

#include <ardrone_autonomy/Navdata.h>
#include <ardrone_autonomy/Navdata2.h>

#define MSS_PER_GS 9.80

#define M_TAU M_PI * 2

char GetRosParam(char *param, char defaultVal) {
    std::string name(param);
    int res, ret;
    ret = (ros::param::get(name, res)) ? res : defaultVal;
    ROS_DEBUG("SET %-30s: %02x", param, ret);
    return (char)ret;
}

class ARDrone_Imu {
private:
    double rotx,roty,rotz;
    double linx,liny,linz;
    double velx,vely,velz;
    double accx,accy,accz;
    double magx,magy,magz;

    double alt;
    double time;
    ros::Time rtime;
    double dt;
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher imu_pub;
    ros::Publisher vo_pub;

public:
    ARDrone_Imu();
    void runloop(const ardrone_autonomy::Navdata2::ConstPtr &msg);
    void PubIMU();
};

void ARDrone_Imu::PubIMU()
{
    if (imu_pub.getNumSubscribers() == 0 && vo_pub.getNumSubscribers() == 0)
        return;

    sensor_msgs::Imu msg;
    msg.header.stamp = rtime;

    msg.angular_velocity_covariance[0] = -1;
    msg.linear_acceleration_covariance[0] = -1;

    geometry_msgs::Quaternion q;
    for (int i = 0; i < 9; i++)
        msg.orientation_covariance[i] = 0;
    msg.orientation_covariance[0] = 0.1;
    msg.orientation_covariance[4] = 0.1;
    msg.orientation_covariance[8] = 0.1;

    q.x = rotx;
    q.y = roty;
    q.z = rotz;
    q.w = 0;

    msg.orientation = q;

    geometry_msgs::Vector3 av;
    geometry_msgs::Vector3 la;

    la.x = accx;
    la.y = accy;
    la.z = accz;

    av.x = rotx / dt;
    av.y = roty / dt;
    av.z = rotz / dt;

    msg.angular_velocity = av;
    msg.linear_acceleration = la;


    nav_msgs::Odometry om;
    om.header.stamp = rtime;

    om.pose.pose.position.x = linx;
    om.pose.pose.position.y = liny;
    om.pose.pose.position.z = linz;
    om.pose.pose.orientation.x = magx;
    om.pose.pose.orientation.y = magy;
    om.pose.pose.orientation.z = magz;
    om.pose.pose.orientation.w = 0;

    double c = 6; // mag covarience..

    double x,y,z;
    x = y = 99999; // no idea where it is. :/
    z = 0.1;
    // todo: get a GPS device on here to fix that!

    for (int i = 0; i < 36; i++)
        om.pose.covariance[i] = 0;

    om.pose.covariance[0] = x;
    om.pose.covariance[7] = y;
    om.pose.covariance[14] = z;
    om.pose.covariance[21] = c;
    om.pose.covariance[28] = c;
    om.pose.covariance[35] = c;
    imu_pub.publish(msg);
    vo_pub.publish(om);
}

void ARDrone_Imu::runloop(const ardrone_autonomy::Navdata2::ConstPtr &msg)
{
    // hm.
    if (msg->tm < time) // drop the message, it's out of date.
        return;
    if (time == 0) {
        time = msg->tm;
        return;
    }
    rtime = ros::Time::now();
    dt = (msg->tm - time) / 1000000; // to seconds...
    double ts = dt * dt;
    time = msg->tm;
    // cm to m
    alt = (double)msg->altd / 100;

    velx = msg->vx / 1000;
    vely = msg->vy / 1000;
    velz = msg->vz / 1000;

    double gravity = MSS_PER_GS;

    accx = msg->ax * gravity;
    accy = msg->ay * gravity;
    accz = msg->az * gravity;

    rotx = msg->rotX;
    roty = msg->rotY;
    rotz = msg->rotZ;

    magx = msg->magX;
    magy = msg->magY;
    magz = msg->magZ;

    if (msg->state >= 3 && msg->state != 5) {
        linx += ((velx * dt) + (0.5 * ts * accx));
        liny += ((vely * dt) + (0.5 * ts * accy));
    //    linz += ((velz * dt) + (0.5 * ts * accz));
        linz = alt;
    } else if (msg->state != 0) { // flying.
        accx = 0;
        accy = 0;
        accz = gravity;
        rotx = 0;
        roty = 0;
        rotz = 0;
    }
    PubIMU();
}

ARDrone_Imu::ARDrone_Imu()
{
    linx = 0;
    liny = 0;
    linz = 0;
    rotx = 0;
    roty = 0;
    rotz = 0;

    velx = 0;
    vely = 0;
    velz = 0;

    accx = 0;
    accy = 0;
    accz = 0;

    time = 0;

    sub = n.subscribe("ardrone/navdata2", 1, &ARDrone_Imu::runloop, this);
    fprintf(stderr, "subscribed to ardrone/navdata2\n");
    imu_pub = n.advertise<sensor_msgs::Imu>("imu_data", 10);
    vo_pub = n.advertise<nav_msgs::Odometry>("vo", 10);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ardrone_imu");
    ARDrone_Imu imu = ARDrone_Imu();
    ros::spin();

    return 0;
}
