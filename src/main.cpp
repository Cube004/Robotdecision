#include "ros/ros.h"
#include "decision/core/decisonCore.hpp"

int main(int argc, char **argv){
    ros::init(argc, argv, "decison");
    ros::NodeHandle n;
    n.setParam("/use_sim_time", true);
    DecisionConfig decision_config;
    setlocale(LC_ALL, "");
    decision_config.decision_config_path_ = "/home/cube/Documents/decision/rules/rule_debug.json";
    decision_config.referee_topic_ = "/referee";
    decision_config.frame_id_ = "odom";
    decision_config.child_frame_id_ = "base_footprint";

    DecisionCore decision(decision_config, &n);

    decision.run();

    while (ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}
