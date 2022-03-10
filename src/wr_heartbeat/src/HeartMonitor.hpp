#ifndef HEARTMONITOR_GUARD
#define HEARTMONITOR_GUARD

#include "FrequencyCharacteristics.hpp"
#include "XmlRpcValue.h"
#include "ros/node_handle.h"
#include "ros/timer.h"
#include "std_srvs/Empty.h"
#include <string>

class HeartMonitor{
private:
    std::string name;
    ros::ServiceServer monitor;
    FrequencyCharacteristics freq;
    ros::Time lastHeartbeat;
    ros::Timer heartbeatChecker;
    std::atomic_bool monitorInitialized;
    std::atomic_bool initialHeartbeat;
    bool registerHeartBeat(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    void checkHeartbeat(const ros::TimerEvent& event);
    HeartMonitor(const std::string &monitorName, FrequencyCharacteristics &freq, ros::NodeHandle &n);
public:
    static std::unique_ptr<HeartMonitor> makeHeartMonitor(XmlRpc::XmlRpcValue &monitorRosParam, ros::NodeHandle &n);
};
#endif