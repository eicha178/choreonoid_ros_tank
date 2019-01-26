#include <cnoid/SimpleController>
#include <cnoid/Joystick>

#include <cnoid/EigenUtil>

#include "ros/ros.h"
//#include "ros/node_handle.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"

#include "choreonoid_ros_tank/SetJointPosition.h"

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

    ros::Subscriber armSubscriber;
    sensor_msgs::JointState msg_arm;

    double effort[2]={0.0, 0.0};
    double position[2]={0.0, 0.0};

    ros::ServiceServer service;

    //Joystick joystick;

    Body* body;
    SimpleControllerIO* _io;

public:
    virtual bool initialize(SimpleControllerIO* io) override
    {
        this->io = io;
        _io = io;
        ostream& os = io->os();
        //Body* body = io->body();
        body = io->body();

        // Crawler
        usePseudoContinousTrackMode = true;
        //armActuationMode = Link::ActuationMode::JOINT_TORQUE;
        armActuationMode = Link::ActuationMode::JOINT_ANGLE;

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
        armSubscriber   = node.subscribe("cmd_arm", 1, &RobotController::armCallback, this);

        service = node.advertiseService("set_joint_position", &RobotController::setJointPosition, this);
    }

    virtual bool control() override
    {
        double vel[2];
        for(int i=0; i < 2; ++i){
            vel[i] = 
                i==0 ? msg_track.angular.z : msg_track.linear.x;
            if(fabs(vel[i]) < 0.2){
                vel[i] = 0.0;
            }
        }

        // set the velocity of each tracks
        if(usePseudoContinousTrackMode){
            double k = 1.0;
            trackL->dq_target() = k * (vel[1] - vel[0]/2);
            trackR->dq_target() = k * (vel[1] + vel[0]/2);
        } else {
            double k = 4.0;
            trackL->dq_target() = k * (vel[1] - vel[0]/2);
            trackR->dq_target() = k * (vel[1] + vel[0]/2);
        }

        // Arm
        static const double P = 60.0;
        static const double D = 10.0;

        for(int i=0; i<2; ++i) {
            Link* joint = armJoint[i];

            if(armActuationMode == Link::JOINT_TORQUE) {
                double q = joint->q();
                double dq = (q - qprev[i]) / dt;
                double dqref = 0.0;
                double deltaq = 0.002 * effort[i];

                qref[i] += deltaq;
                dqref = deltaq / dt;
                clampTargetJointAngle(joint);
                joint->u() = P * (qref[i] - q) + D * (dqref - dq);
                qprev[i] = q;

            }
            else if (armActuationMode == Link::JOINT_ANGLE) {
                double q = joint->q();
                //double dq = (q - qprev[i]) / dt;
                double dqref = 0.0;
                double deltaq = 0.0;

                if(!fabs(position[i]-q) < 0.05){
                    double deltaq = 0.001 ;
                    if(position[i] < q){
                        deltaq = deltaq * -1;
                    }
                    qref[i] += deltaq;
                    clampTargetJointAngle(joint);
                    joint->q_target() = qref[i];
                    qprev[i] = q;
                }
            }
        }
    }

    void trackCallback(const geometry_msgs::Twist& msg)
    {
        msg_track = msg;
    }

    void armCallback(const sensor_msgs::JointState& msg)
    {
        msg_arm = msg;
        effort[0] = msg.effort[0];
        effort[1] = msg.effort[1];
        
        position[0] = msg.position[0];
        position[1] = msg.position[1];
    }

    void clampTargetJointAngle(Link* joint)
    {
        int jointId = joint->jointId();
        static const double maxerror = radian(3.0);
        double q_current = joint->q();
        double q_lower = std::max(q_current - maxerror, joint->q_lower());
        double q_upper = std::min(q_current + maxerror, joint->q_upper());

        if(qref[jointId] < q_lower){
            qref[jointId] = q_lower;
        } else if(qref[jointId] > q_upper){
            qref[jointId] = q_upper;
        }
    }

    // service
    bool setJointPosition(choreonoid_ros_tank::SetJointPosition::Request  &req,
                          choreonoid_ros_tank::SetJointPosition::Response &res)
    {
        int _jId = req.joint_id;

        position[_jId] = req.joint_position;
        
        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(RobotController)
