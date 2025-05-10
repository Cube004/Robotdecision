#include "ros/ros.h"
#include "decision/core/decisonCore.hpp"

int main(int argc, char **argv){
    ros::init(argc, argv, "decison");
    ros::NodeHandle n;
    setlocale(LC_ALL, "");


    std::string config_path = "rules";
    std::string config_file_name = "rule_debug.json";
    std::string referee_topic_ = "/referee";
    std::string frame_id_ = "odom";
    std::string child_frame_id_ = "base_footprint";
    bool use_sim_time = true;

    n.getParam("/decision/use_sim_time", use_sim_time);
    n.getParam("/decision/config_path", config_path);
    n.getParam("/decision/config_file_name", config_file_name);
    n.getParam("/decision/referee_topic", referee_topic_);
    n.getParam("/decision/frame_id", frame_id_);
    n.getParam("/decision/child_frame_id", child_frame_id_);

    n.setParam("/use_sim_time", use_sim_time);
    DecisionConfig decision_config;
    decision_config.decision_config_path_ = config_path + "/" + config_file_name;
    decision_config.referee_topic_ = referee_topic_;
    decision_config.frame_id_ = frame_id_;
    decision_config.child_frame_id_ = child_frame_id_;

    DecisionCore decision(decision_config, &n);

    decision.run();

    while (ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}
