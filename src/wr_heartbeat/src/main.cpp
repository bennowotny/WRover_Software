#include "XmlRpcValue.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include <ros/ros.h>
#include <unordered_set>
#include "HeartMonitor.hpp"
#include "HeartBeat.hpp"

int main(int argc, char **argv){
    ros::init(argc, argv, "wr_heartbeat");
    ros::NodeHandle n;
    ros::NodeHandle params{"~"};

    XmlRpc::XmlRpcValue monitorList;
    params.getParam("monitors", monitorList);

    if(monitorList.getType() != XmlRpc::XmlRpcValue::TypeArray){
        ROS_FATAL("Parameter 'monitors' must exist as a ROSParam array of Monitor definitions.");
        throw std::logic_error("Parameter 'monitors' is invalid.");
    }

    XmlRpc::XmlRpcValue heartbeatList;
    params.getParam("heartbeats", heartbeatList);

    if(heartbeatList.getType() != XmlRpc::XmlRpcValue::TypeArray){
        ROS_FATAL("Parameter 'heartbeats' must exist as a ROSParam array of Heartbeat definitions.");
        throw std::logic_error("Parameter 'heartbeats' is invalid.");
    }

    std::unordered_set<std::unique_ptr<HeartMonitor>> monitors;
    for(std::uint32_t i = 0; i < monitorList.size(); i++) monitors.insert(HeartMonitor::makeHeartMonitor(monitorList[i], n));

    std::unordered_set<std::unique_ptr<HeartBeat>> heartbeats;
    for(std::uint32_t i = 0; i < heartbeatList.size(); i++) heartbeats.insert(HeartBeat::makeHeartBeat(heartbeatList[i], n));

    ros::spin();
    return 0;
}