#include "FrequencyCharacteristics.hpp"
#include "XmlRpcValue.h"
#include <ros/ros.h>

FrequencyCharacteristics::FrequencyCharacteristics(double nominalFreq, double nominalTol) : nominalFrequency{nominalFreq}, nominalTolerance{nominalTol}{}

FrequencyCharacteristics FrequencyCharacteristics::makeFrequencyCharacteristics(XmlRpc::XmlRpcValue &rosParamSetting){
    
    if(rosParamSetting.getType() != XmlRpc::XmlRpcValue::TypeStruct){
        ROS_FATAL("Frequency Characteristics for Heartbeat must be a ROSParam dictionary.");
        throw std::logic_error("Frequency Characteristic has invalid format");
    }

    // Get Frequency
    if(!rosParamSetting.hasMember("nominalFrequency") || (rosParamSetting["nominalFrequency"].getType() != XmlRpc::XmlRpcValue::TypeDouble && rosParamSetting["nominalFrequency"].getType() != XmlRpc::XmlRpcValue::TypeInt)){
        ROS_FATAL("Parameter 'nominalFrequency' must exist as a ROSParam of type double in a Frequency Characteristic.");
        throw std::logic_error("Parameter 'nominalFrequency' is invalid.");
    }
    double nomFreq = 0;
    if(rosParamSetting["nominalFrequency"].getType() == XmlRpc::XmlRpcValue::TypeInt) 
        nomFreq = static_cast<int>(rosParamSetting["nominalFrequency"]);
    else
        nomFreq = static_cast<double>(rosParamSetting["nominalFrequency"]);

    // Get Frequency Tolerance
    if(!rosParamSetting.hasMember("nominalTolerance") || (rosParamSetting["nominalTolerance"].getType() != XmlRpc::XmlRpcValue::TypeDouble && rosParamSetting["nominalTolerance"].getType() != XmlRpc::XmlRpcValue::TypeInt)){
        ROS_FATAL("Parameter 'nominalTolerance' must exist as a ROSParam of type double in a Frequency Characteristic.");
        throw std::logic_error("Parameter 'nominalTolerance' is invalid.");
    }
    double nomTol = 0;
    if (rosParamSetting["nominalTolerance"].getType() == XmlRpc::XmlRpcValue::TypeInt) 
        nomTol = static_cast<int>(rosParamSetting["nominalTolerance"]);
    else
        nomTol = static_cast<double>(rosParamSetting["nominalTolerance"]);

    return {nomFreq, nomTol};
}

double FrequencyCharacteristics::getPeriod() const{
    return 1/this->nominalFrequency;
}
