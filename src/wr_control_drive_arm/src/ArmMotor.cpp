#include "ArmMotor.hpp"
#include <iostream>
#include "ros/ros.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/UInt16.h"
#include "math.h"

#define Std_UInt32 std_msgs::UInt32::ConstPtr&

class ArmMotor{
    private:
        enum MotorState{
            STOP, MOVING, RUN_TO_TARGET
        };
        MotorState currState;
        std::string motorName;
        unsigned int controllerID;
        unsigned int motorID;
        ros::Subscriber encRead;
        ros::Publisher speedPub;
        int encoderVal;

        const int COUNTS_PER_ROTATION = 0;

        float radToEnc(float rads){
            return COUNTS_PER_ROTATION * rads / (2 * M_PI);
        }

        void storeEncoderVals(const Std_UInt32 msg){
            this->encoderVal = msg->data;
        }

    public:
        ArmMotor(std::string motorName, unsigned int controllerID, unsigned int motorID, ros::NodeHandle& n){
            if(controllerID > 3) throw ((std::string)"Controller ID ") + std::to_string(controllerID) + "is not valid on [0,3]";
            if(motorID > 1) throw ((std::string)"Motor ID ") + std::to_string(motorID) + "is not valid on [0,1]";
            
            this->motorName = motorName;
            this->controllerID = controllerID;
            this->motorID = motorID;
            this->currState = MotorState::STOP;

            std::string tpString = ((std::string)"/hsi/wroboclaw/aux") + std::to_string(controllerID);

            encRead = n.subscribe(tpString + "/enc/" + (motorID == 0 ? "left" : "right"), 1000, storeEncoderVals, this);
            speedPub = n.advertise<std_msgs::UInt16>(tpString + "/cmd/" + (motorID == 0 ? "left" : "right"), 1000);
        }

        // ~ArmMotor();
        
        int getEncoderCounts(){
            return encoderVal;
        }

        // void resetEncoder();
        void runToTarget(int targetCounts, float power){
            power = -1;
        }
        MotorState getMotorState();
        void setPower(float power){
            std_msgs::UInt16 msg;
            msg.data = power * INT16_MAX;
            speedPub.publish(msg);
            currState = power == 0.f ? MotorState::STOP : MotorState::MOVING;
        };
};