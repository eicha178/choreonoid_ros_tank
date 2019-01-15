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
    Link::ActuationMode armActuationMode;
    Link* trackL;
    Link* trackR;
    Link* armJoint[2];
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

        // Crawler
        usePseudoContinousTrackMode = true;
        armActuationMode = Link::ActuationMode::JOINT_TORQUE;
        for(auto opt : io->options()){
            if(opt == "wheels"){
                usePseudoContinousTrackMode = false;
            }
            //if(opt == "velocity"){
            //    armActuationMode = Link::ActuationMode::JOINT_VELOCITY;
            //}
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
            os << "set JOINT_SURFACE_VELOCITY" << endl;
            trackL->setActuationMode(Link::JOINT_SURFACE_VELOCITY);
            trackR->setActuationMode(Link::JOINT_SURFACE_VELOCITY);
        } else {
            os << "JOINT_VELOCITY" << endl;
            trackL->setActuationMode(Link::JOINT_VELOCITY);
            trackR->setActuationMode(Link::JOINT_VELOCITY);
        }
        io->enableOutput(trackL);
        io->enableOutput(trackR);
        

        // Camera Arm
        armJoint[0] = body->link("CAMERA_ARM_LINK_1");
        armJoint[1] = body->link("CAMERA_ARM_LINK_2");

        for(int i=0; i<2; ++i) {
            Link* joint = armJoint[i];
            if(!joint) {
                os << "Arm joint " << i << " is not found." << endl;
            }
            joint->setActuationMode(armActuationMode);
            io->enableIO(joint);
            qref[i] = qprev[i] = joint->q();
        }

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

        // Arm
        //static const double P = 200.0;
        //static const double D = 50.0;

        static const double P = 50.0;
        static const double D = 10.0;

        for(int i=0; i<2; ++i) {
            Link* joint = armJoint[i];
            //double dummyQ = 0.0;
    
            //if(armActuationMode == Link::JOINT_VELOCITY) {
            //    joint->dq_target() = dummyQ;
            //} else if(armActuationMode == Link::JOINT_TORQUE) {
            if(armActuationMode == Link::JOINT_TORQUE) {
                double q = joint->q();
                double dq = (q - qprev[i]) / dt;
                double dqref = 0.0;
//                double deltaq = 0.002 * dummyQ;
//                qref[i] += deltaq;
//                dqref = deltaq / dt;
                joint->u() = P * (qref[i] - q) + D * (dqref - dq);
                qprev[i] = q;
            }
        }

    }

    void trackCallback(const geometry_msgs::Twist& msg)
    {
        msg_track = msg;
    }

};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(RobotController)
