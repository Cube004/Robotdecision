#include "ros/ros.h"
#include "decision/DecisionStatus.h"


int main(int argc, char **argv){
    ros::init(argc, argv, "decision_websocket_service");
    ros::NodeHandle nh;
    ros::Subscriber decision_sub = nh.subscribe("/decision_status", 10, decision_status_cb);
    ros::spin();
    return 0;
}

void decision_status_cb(const decision::DecisionStatus::ConstPtr& msg){
    const std::string& jsonData = msg->jsonData;
}