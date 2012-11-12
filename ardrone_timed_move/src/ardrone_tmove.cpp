#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <unistd.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>

#define DEBUG

#include <ardrone_autonomy/Navdata.h>

#define MSS_PER_GS 9.80

#define M_TAU (M_PI * 2)

typedef struct _v3 {
    double x;
    double y;
    double z;
} v3;

char GetRosParam(char *param, char defaultVal) {
    std::string name(param);
    int res, ret;
    ret = (ros::param::get(name, res)) ? res : defaultVal;
    ROS_DEBUG("SET %-30s: %02x", param, ret);
    return (char)ret;
}

enum ARState { ARStateLanded, ARStateFlying, ARStateEmergency };

class ARDrone_Waypoint {
private:
    ros::Time rtime;

    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Subscriber destination;
    ros::Publisher cmds; // controlling the ardrone
    ros::Publisher emerPub;
    ros::Publisher landPub;
    ros::Publisher flyPub;

    ARState State;
    double batteryState;

public:
    ARDrone_Waypoint();
    void navdata(const ardrone_autonomy::Navdata::ConstPtr &msg);
    void newDest(const geometry_msgs::Vector3::ConstPtr &dest);
    void signalEmergency(char *reason);

    void basicMove(v3 m);

    void CheckBatteryState();

    void land();
    void takeoff();
    void emergency(char *reason);
};

void ARDrone_Waypoint::signalEmergency(char *reason)
{
    fprintf(stderr, "Emergency! Reason: %s\n", reason);
}

void ARDrone_Waypoint::land()
{
    //if (State == ARStateFlying)
        landPub.publish(std_msgs::Empty());
}

void ARDrone_Waypoint::takeoff()
{
    if (State == ARStateLanded)
        flyPub.publish(std_msgs::Empty());
}

void ARDrone_Waypoint::emergency(char *reason)
{
    emerPub.publish(std_msgs::Empty());
    signalEmergency(reason);
}

void ARDrone_Waypoint::newDest(const geometry_msgs::Vector3::ConstPtr &dest)
{
    v3 hack;
    hack.x = dest->x;
    hack.y = dest->y;
    hack.z = dest->z;
    basicMove(hack);
}

void ARDrone_Waypoint::basicMove(v3 m)
{
    takeoff();
    sleep(4);
    double dx = 1.0;
    if (fabs(m.x) > dx)
        dx = fabs(m.x);
    if (fabs(m.y) > dx)
        dx = fabs(m.y);

    int timeToSleep = 1 + (int)sqrt(dx);

    geometry_msgs::Twist toSend;
    toSend.linear.x = m.x / dx;
    toSend.linear.y = m.y / dx;
    toSend.linear.z = 0;
    cmds.publish(toSend);
    sleep(timeToSleep);
    toSend.linear.x = 0;
    toSend.linear.y = 0;
    cmds.publish(toSend);
    land();
}

void ARDrone_Waypoint::CheckBatteryState()
{
    if (batteryState <= 15.0) {
        if (State == ARStateFlying) {
            land();
        } else if (State == ARStateLanded) {
            emergency("battery is below 15 percent");
        }
    }
}

void ARDrone_Waypoint::navdata(const ardrone_autonomy::Navdata::ConstPtr &msg)
{
    int foo = msg->state;
    batteryState = msg->batteryPercent;
    if (foo >= 3 && foo != 5)
        State = ARStateFlying;
    else if (foo != 0)
        State = ARStateLanded;
    else {
        if (State != ARStateEmergency)
            signalEmergency("Unknown");
        State = ARStateEmergency;
    }
    CheckBatteryState();
}

ARDrone_Waypoint::ARDrone_Waypoint()
{
    State = ARStateLanded;

    sub = n.subscribe("ardrone/navdata", 1, &ARDrone_Waypoint::navdata, this);
    cmds = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    emerPub = n.advertise<std_msgs::Empty>("ardrone/reset", 1);
    landPub = n.advertise<std_msgs::Empty>("ardrone/land", 1);
    flyPub = n.advertise<std_msgs::Empty>("ardrone/takeoff", 1);
    destination = n.subscribe("destination", 1, &ARDrone_Waypoint::newDest, this);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ardrone_waypoint");
    ARDrone_Waypoint waypoint = ARDrone_Waypoint();
    ros::spin();

    return 0;
}
