#include <cnoid/SimpleController>
#include <cnoid/Joystick>

#include "ros/node_handle.h"
#include "geometry_msgs/Twist.h"

using namespace std;
using namespace cnoid;

class RobotController : public SimpleController
{
    SimpleControllerIO* io;
    bool usePseudoContinousTrackMode;
    Link::ActuationMode turretActuationMode;
    Link* trackL;
    Link* trackR;
    Link* turretJoint[2];
    double qref[2];
    double qprev[2];
    double dt;

    ros::NodeHandle node;
    ros::Subscriber trackSubscriber;
    geometry_msgs::Twist msg_track;

    Joystick joystick;

public:
    
    virtual bool initialize(SimpleControllerIO* io) override
    {
        this->io = io;
        ostream& os = io->os();
        Body* body = io->body();

        usePseudoContinousTrackMode = true;
        turretActuationMode = Link::ActuationMode::JOINT_TORQUE;
        for(auto opt : io->options()){
            if(opt == "wheels"){
                usePseudoContinousTrackMode = false;
            }
            if(opt == "velocity"){
                turretActuationMode = Link::ActuationMode::JOINT_VELOCITY;
            }
        }

        if(usePseudoContinousTrackMode){
            trackL = body->link("TRACK_L");
            trackR = body->link("TRACK_R");

        } else {
            trackL = body->link("WHEEL_L0");
            trackR = body->link("WHEEL_R0");
        }

        if(!trackL || !trackR){
            os << "The tracks are not found." << endl;
            return false;
        }

        if(usePseudoContinousTrackMode){
            trackL->setActuationMode(Link::JOINT_SURFACE_VELOCITY);
            trackR->setActuationMode(Link::JOINT_SURFACE_VELOCITY);
        } else {
            trackL->setActuationMode(Link::JOINT_VELOCITY);
            trackR->setActuationMode(Link::JOINT_VELOCITY);
        }
        io->enableOutput(trackL);
        io->enableOutput(trackR);
        
        dt = io->timeStep();
    }

    virtual bool start() override
    {
        trackSubscriber = node.subscribe("cmd_vel", 1, &RobotController::trackCallback, this);
    }

    virtual bool control() override
    {
        double pos[2];
        for(int i=0; i < 2; ++i){
            pos[i] = 
                i==0 ? msg_track.angular.z : msg_track.linear.x;
            if(fabs(pos[i]) < 0.2){
                pos[i] = 0.0;
            }
        }

        // set the velocity of each tracks
        if(usePseudoContinousTrackMode){
            double k = 1.0;
            trackL->dq_target() = k * (pos[1] - pos[0]/2);
            trackR->dq_target() = k * (pos[1] + pos[0]/2);
        } else {
            double k = 4.0;
            trackL->dq_target() = k * (pos[1] - pos[0]/2);
            trackR->dq_target() = k * (pos[1] + pos[0]/2);
        }
    }

    void trackCallback(const geometry_msgs::Twist& msg)
    {
        msg_track = msg;
    }

};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(RobotController)
