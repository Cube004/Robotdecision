#ifndef GRAPH_H
#define GRAPH_H

#include <unordered_map>
#include <vector>
#include <iostream>
#include "ros/ros.h"
#include "decision/rules/rules.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "nlohmann/json.hpp"
#include <fstream>

namespace rules {
    enum NodeType {
        ROOT = 0,
        BRANCH = 1,
        TASK = 2
    };

    enum NodeMode {
        NAVIGATION,
        STAY,
        LIMIT
    };

    struct Edge {
        int edge_id;
        int nodeIn_id;
        int nodeOut_id;
        int weight;
        rules::DecisionRules rules;
    };

    struct NodeGroup {
        int group_id;
        std::vector<int> nodes;
        bool Loop;
        float ResetTime;
        bool Reverse;
    };

    struct Waypoint {
        int waypoint_id;
        std::string name;
        geometry_msgs::PoseStamped pose;
    };

    struct Area {
        int area_id;
        std::string name;
        geometry_msgs::PoseStamped leftTop;
        geometry_msgs::PoseStamped rightBottom;
    };

    struct Node {
        int node_id;
        NodeType type;
        NodeMode mode;
        geometry_msgs::PoseStamped waypoint;
        float resetTime;

        double limit_linear;
        double limit_angular;
        
        bool finish;
        ros::Time last_check_time;// 上一次检查该节点的时间
        ros::Time start_time;
        ros::Time end_time;
        bool get_finish(ros::NodeHandle *nh){
            if (this->type == NodeType::ROOT || this->type == NodeType::BRANCH){
                this->finish = true;
                return this->finish;
            }
            if (this->type == NodeType::TASK && this->mode == NodeMode::LIMIT){
                double limit_linear;
                double limit_angular;
                nh->getParam("/decision/limit_linear", limit_linear);
                nh->getParam("/decision/limit_angular", limit_angular);
                if (limit_linear == this->limit_linear && limit_angular == this->limit_angular){
                    this->finish = true;
                    return this->finish;
                }else{
                    ROS_ERROR("限制任务未完成: 当前 线速度%f 转速%f 预期线速度%f 预期转速%f", limit_linear, limit_angular, this->limit_linear, this->limit_angular);
                }
            }
            
            if (resetTime == -1) return finish;
            // 非一次性任务超过指定时间未到达该节点认为是冷数据, 一段时间后重置完成状态
            // (未到达该节点也可理解为任务决策在上一段时间偏离转而执行其他路线上的节点)
            if (finish && ros::Time::now() - last_check_time > ros::Duration(resetTime)) {
                std::cout << "reset finish: " << (ros::Time::now() - last_check_time).toSec() << std::endl;
                this->end_time = ros::Time(0);
                this->finish = false;
            }
            this->last_check_time = ros::Time::now();// 更新检查时间
            return this->finish;
        }
        std::vector<Edge> edges;
        std::vector<int> group_id;
    };

class Graph {
    public:
        std::unordered_map<int, Node> nodeList;
        std::unordered_map<int, Edge> edgeList;
        std::unordered_map<int, NodeGroup> nodeGroupList;
        std::unordered_map<int, Waypoint> waypointList;
        std::unordered_map<int, Area> areaList;

        std::string decisionJsonStr;
        int rootNode_id = -1;
        bool init(const std::string& decisionJsonStr);
        bool CheckFinish(int node_id, ros::NodeHandle *nh);
        void transformDecisionRule(std::string filePath);
    };
}

#endif // GRAPH_H
