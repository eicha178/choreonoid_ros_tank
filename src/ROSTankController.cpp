#include <cnoid/SimpleController>
#include <cnoid/Joystick>

#include "ros/node_handle.h"
#include "geometry_msgs/Twist.h"

using namespace std;
using namespace cnoid;

class ROSTankController : public SimpleController
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
    ros::Subscriber turretSubscriber;
    geometry_msgs::Twist msg_track;
    geometry_msgs::Twist msg_turret;

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
        
        turretJoint[0] = body->link("TURRET_Y");
        turretJoint[1] = body->link("TURRET_P");
        for(int i=0; i < 2; ++i){
            Link* joint = turretJoint[i];
            if(!joint){
                os << "Turret joint " << i << " is not found." << endl;
                return false;
            }
            qref[i] = qprev[i] = joint->q();
            joint->setActuationMode(turretActuationMode);
            io->enableIO(joint);
        }

        dt = io->timeStep();
    }

    virtual bool start() override
    {
        trackSubscriber = node.subscribe("cmd_vel", 1, &ROSTankController::trackCallback, this);
        turretSubscriber = node.subscribe("cmd_turret", 1, &ROSTankController::turretCallback, this);
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
            trackL->dq_target() = k * (-pos[1] - pos[0]);
            trackR->dq_target() = k * (-pos[1] + pos[0]);
        } else {
            double k = 4.0;
            trackL->dq_target() = k * (-pos[1] - pos[0]);
            trackR->dq_target() = k * (-pos[1] + pos[0]);
        }

        static const double P = 200.0;
        static const double D = 50.0;

        for(int i=0; i < 2; ++i){
            Link* joint = turretJoint[i];
            double pos = 
                i==0 ? msg_turret.angular.z : msg_turret.angular.y;

            if(fabs(pos) < 0.15){
                pos = 0.0;
            }

            if(turretActuationMode == Link::JOINT_VELOCITY){
                joint->dq_target() = -pos;

            } else if(turretActuationMode == Link::JOINT_TORQUE){
                double q = joint->q();
                double dq = (q - qprev[i]) / dt;
                double dqref = 0.0;
                double deltaq = 0.002 * -pos;
                qref[i] += deltaq;
                dqref = deltaq / dt;
                joint->u() = P * (qref[i] - q) + D * (dqref - dq);
                qprev[i] = q;
            }
        }
    }

    void trackCallback(const geometry_msgs::Twist& msg)
    {
        msg_track = msg;
    }

    void turretCallback(const geometry_msgs::Twist& msg)
    {
        msg_turret = msg;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(ROSTankController)
