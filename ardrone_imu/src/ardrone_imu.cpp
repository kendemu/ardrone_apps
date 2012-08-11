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

#define DEBUG

#include <ardrone_autonomy/Navdata.h>
#include <ardrone_autonomy/Navdata2.h>

#define MSS_PER_GS 9.79

#define M_TAU M_PI * 2

inline double ssds(std::vector<double> xi, double x)
{
    if (xi.size() < 2)
        return 0;
    double sum = 0;
    for (std::vector<double>::iterator i = xi.begin(); i < xi.end(); i++) {
        double foo = *i - x;
        sum += (foo * foo);
    }
    return sum / (xi.size() - 1);
}

inline double ssd(std::vector<double> xi, double x)
{
    return sqrt(ssds(xi, x));
}

inline double mean(std::vector<double> xi)
{
    if (xi.size() == 0)
        return 0;
    double sum = 0;
    for (std::vector<double>::iterator i = xi.begin(); i < xi.end(); i++) {
        double d = *i;
        sum += d;
    }
    return sum / xi.size();
}

inline double fastSign(double d)
{
    if (d < 0)
        return -1;
    return 1;
}

inline double rse(std::vector<double> xi)
{
    //double sd = ssds(xi, normal);
    if (xi.size() == 0)
        return 1;
    double me = mean(xi);
    return ssd(xi, me);
}

inline double correctAccel(double a, double v, double dt)
{
    double expected = v / dt;
    double sum = expected + a;
    return sum / 2;
}

char GetRosParam(char *param, char defaultVal) {
    std::string name(param);
    int res, ret;
    ret = (ros::param::get(name, res)) ? res : defaultVal;
    ROS_DEBUG("SET %-30s: %02x", param, ret);
    return (char)ret;
}

class ARDrone_Imu {
private:
    double linx,liny,linz;
    double rotx,roty,rotz;
    double velx,vely,velz;
    double accx,accy,accz;
    double ervx,ervy,ervz;
    double erax,eray,eraz;

    double gravity;
    double training;
    double gtraining;
    unsigned int lastTrainingDataSize;

    double alt;
    double time;
    std::vector<double> tvx;
    std::vector<double> tvy;
    std::vector<double> tvz;
    std::vector<double> tax;
    std::vector<double> tay;
    std::vector<double> taz;
    ros::NodeHandle n;
    ros::Subscriber sub;

public:
    ARDrone_Imu();
    void setErrorVals();
    void runloop(const ardrone_autonomy::Navdata2::ConstPtr &msg);
};

void ARDrone_Imu::setErrorVals()
{
    assert(tvx.size() == tvy.size() && tvy.size() == tvz.size());
    if (lastTrainingDataSize == tvx.size())
        return;
    ervx = rse(tvx); // should be standing still...
    ervy = rse(tvy);
    ervz = rse(tvz);

    erax = rse(tax);
    eray = rse(tay);
    eraz = rse(taz);
    lastTrainingDataSize = tvx.size();
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
    double dt = (msg->tm - time) / 1000000; // to seconds...
    double ts = dt * dt;
    time = msg->tm;
    alt = (double)msg->altd;

    double vx = msg->vx / 1000; // converting millimeters to meters
    double vy = msg->vy / 1000;
    double vz = msg->vz / 1000;
    double ax = msg->ax;
    double ay = msg->ay;
    double az = msg->az;

    if (msg->state >= 3 && msg->state != 5) { // flying.
        setErrorVals();
        ax *= gravity;
        ay *= gravity;
        az *= gravity;
        az -= gravity;

        vx = vx / ervx;
        vy = vy / ervy;
        vz = vz / ervz;
        ax = ax / erax;
        ay = ay / eray;
        az = az / eraz;
        linx += ((vx * dt) + (0.5 * ts * ax));
        liny += ((vy * dt) + (0.5 * ts * ay));
        linz += ((vz * dt) + (0.5 * ts * az));

        fprintf(stderr, "\r%f\t,%f\t,%f\t", linx, liny, linz);
    } else if (msg->state != 0) { // training.
        double lx,ly,lz;
        lx = (vx * dt) + (0.5 * ts * ax);
        ly = (vy * dt) + (0.5 * ts * ay);
        lz = (vz * dt) + (0.5 * ts * az);
        /*
        double evx = abs(vx);
        double evy = abs(vy);
        double evz = abs(vz);
        double eax = abs(ax);
        double eay = abs(ay);
        double eaz = abs(az - gravity);
        */
        tvx.push_back(vx);
        tvy.push_back(vy);
        tvz.push_back(vz);
        tax.push_back(ax);
        tay.push_back(ay);
        taz.push_back(msg->az);
        training += 1;
        /*
        if ((gtraining + msg->az) > 0) {
            gtraining += msg->az;
            gravity = (gtraining / training) * MSS_PER_GS; // don't want to overflow...
        }
        */
        //fprintf(stderr, "Pos: %2.2f\t %2.2f\t %2.2f\t %2.2f\t %2.2f\t %2.2f\t %2.2f\t %2.2f\t %2.2f\t %2.2f\t\n", lx, ly, lz, vx, vy, vz, ax + erax, ay + eray, az + eraz, dt);
    }
    setErrorVals();
    fprintf(stderr, "Errors: %2.6f\t %2.6f\t %2.6f\t %2.6f\t\n", erax, eray, eraz, gravity);
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

    ervx = 0;
    ervy = 0;
    ervz = 0;

    erax = 0;
    eray = 0;
    eraz = 0;
    training = 0;
    gtraining = 0;

    gravity = 9.79;
    time = 0;
    lastTrainingDataSize = 0;

    tvx = std::vector<double>(100);
    tvy = std::vector<double>(100);
    tvz = std::vector<double>(100);

    tax = std::vector<double>(100);
    tay = std::vector<double>(100);
    taz = std::vector<double>(100);

    sub = n.subscribe("ardrone/navdata2", 1, &ARDrone_Imu::runloop, this);
    fprintf(stderr, "subscribed to ardrone/navdata2\n");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ardrone_imu");
    ARDrone_Imu imu = ARDrone_Imu();
    ros::spin();

    return 0;
}
