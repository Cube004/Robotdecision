#include "graph.h"

namespace rules {

    rules::NodeType convertToNodeType(const std::string& nodeType){
        if (nodeType == "task") return rules::NodeType::TASK;
        if (nodeType == "branch") return rules::NodeType::BRANCH;
        if (nodeType == "root") return rules::NodeType::ROOT;
        return rules::NodeType::TASK;
    }

    rules::NodeMode convertToNodeMode(const std::string& mode) {
        if (mode == "Move") return rules::NodeMode::NAVIGATION;
        if (mode == "Stay") return rules::NodeMode::STAY;
        if (mode == "Limit") return rules::NodeMode::LIMIT;
        return rules::NodeMode::NAVIGATION;
    }

    bool Graph::CheckFinish(int id, ros::NodeHandle *nh){
        // 检查节点组是否完成, 如果完成则重置节点组
        for (auto& groupIndex : this->nodeList[id].group_id) {
            bool ALLfinish = true;
            std::stringstream ss;
            if(this->nodeGroupList[groupIndex].Loop == false) continue;
            for (auto& node_id : this->nodeGroupList[groupIndex].nodes) {
                ss << " " << node_id;
                ALLfinish = this->nodeList[node_id].get_finish(nh) && ALLfinish;
            }
            if (ALLfinish) {
                ROS_INFO("group_id: %d all nodes finish", groupIndex);
                ROS_INFO("now reset nodes: %s", ss.str().c_str());
                for (auto& node_id : this->nodeGroupList[groupIndex].nodes) {
                    this->nodeList[node_id].finish = false;
                }
            }
        }
        return this->nodeList[id].get_finish(nh);
    }

    // 初始化图
    bool Graph::init(const std::string& decisionJsonStr){
        try
        {
            std::cout << "decisionJsonStr: " << decisionJsonStr << std::endl;
            this->decisionJsonStr = decisionJsonStr;
            this->nodeList.clear();
            this->edgeList.clear();
            this->nodeGroupList.clear();
            this->waypointList.clear();
            this->areaList.clear();
            this->rootNode_id = -1;
            transformDecisionRule(decisionJsonStr);
        }
        catch(const std::exception& e)
        {
            std::cerr << "规则加载失败: " << e.what() << '\n';
            throw std::runtime_error("错误: " + std::string(e.what()));
        }
        return true;
    }


    void Graph::transformDecisionRule(std::string filePath) {
    try {
        std::ifstream file(filePath);
        if (!file.is_open()) {
            throw std::runtime_error("无法打开文件: " + filePath);
        }
        // 解析JSON
        nlohmann::json jsonData;
        file >> jsonData;
        file.close();
        // 解析基本信息
        std::string version = jsonData["version"];
        std::string createdAt = jsonData["createdAt"];
        std::cout << "version: " << version << std::endl;
        std::cout << "createdAt: " << createdAt << std::endl;


        // 初始化航点
        for (const auto& pointJson : jsonData["points"]) {
            this->waypointList[pointJson["id"]].name = pointJson["text"];
            this->waypointList[pointJson["id"]].pose.pose.position.x = pointJson["waypoint"]["x"];
            this->waypointList[pointJson["id"]].pose.pose.position.y = pointJson["waypoint"]["y"];
        }

        std::cout << "航点读取完成" << std::endl;

        // 初始化区域
        for (const auto& areaJson : jsonData["areas"]) {
            this->areaList[areaJson["id"]].name = areaJson["name"];
            this->areaList[areaJson["id"]].leftTop.pose.position.x = areaJson["leftTopWaypoint"]["x"];
            this->areaList[areaJson["id"]].leftTop.pose.position.y = areaJson["leftTopWaypoint"]["y"];
            this->areaList[areaJson["id"]].rightBottom.pose.position.x = areaJson["rightBottomWaypoint"]["x"];
            this->areaList[areaJson["id"]].rightBottom.pose.position.y = areaJson["rightBottomWaypoint"]["y"];
        }

        std::cout << "区域读取完成" << std::endl;

        // 初始化边
        std::cout << "edgeJson: ";
        for (const auto& edgeJson : jsonData["edges"]) {
            this->edgeList[edgeJson["id"]].edge_id = edgeJson["id"];
            std::cout << edgeJson;
            this->edgeList[edgeJson["id"]].nodeIn_id = edgeJson["sourceId"];
            this->edgeList[edgeJson["id"]].nodeOut_id = edgeJson["targetId"];
            for (const auto& conditionJson : edgeJson["conditions"]) {
                this->edgeList[edgeJson["id"]].rules.add_condition(rules::RuleCondition(
                    conditionJson["metricType"],
                    conditionJson["datetype"],
                    conditionJson["temporalScope"]["type"],
                    conditionJson["temporalScope"]["rollingWindow"],
                    conditionJson["max"],
                    conditionJson["min"]
                ));
            }
        }

        std::cout << "边读取完成" << std::endl;

        // 初始化节点
        for (const auto& nodeJson : jsonData["nodes"]) {
            this->nodeList[nodeJson["id"]].node_id = nodeJson["id"];
            this->nodeList[nodeJson["id"]].type = convertToNodeType(nodeJson["taskConfig"]["nodeType"]);
            this->nodeList[nodeJson["id"]].mode = convertToNodeMode(nodeJson["taskConfig"]["mode"]);
            if (this->nodeList[nodeJson["id"]].type == rules::NodeType::ROOT) {
                this->rootNode_id = nodeJson["id"];
            }
            if (nodeJson["taskConfig"]["waypointId"] != -1) {
                this->nodeList[nodeJson["id"]].waypoint = this->waypointList[nodeJson["taskConfig"]["waypointId"]].pose;
            }
            this->nodeList[nodeJson["id"]].resetTime = nodeJson["taskConfig"]["resetTime"];
            this->nodeList[nodeJson["id"]].limit_linear = nodeJson["taskConfig"]["linear"];
            this->nodeList[nodeJson["id"]].limit_angular = nodeJson["taskConfig"]["spin"];
            for (const auto& edgeJson : nodeJson["edges"]) {
                this->nodeList[nodeJson["id"]].edges.push_back(this->edgeList[edgeJson["value"]]);
            }
        }

        std::cout << "节点读取完成" << std::endl;

        // 初始化节点组
        for (const auto& groupJson : jsonData["nodeGroups"]) {
            this->nodeGroupList[groupJson["id"]].group_id = groupJson["id"];
            for (const auto& nodeId : groupJson["nodesId"]) {
                this->nodeGroupList[groupJson["id"]].nodes.push_back(nodeId);
                this->nodeList[nodeId].group_id.push_back(groupJson["id"]);
            }
            this->nodeGroupList[groupJson["id"]].Loop = groupJson["config"]["Loop"];
            this->nodeGroupList[groupJson["id"]].ResetTime = groupJson["config"]["ResetTime"];
            this->nodeGroupList[groupJson["id"]].Reverse = groupJson["config"]["Reverse"];
        }

        std::cout << "节点组读取完成" << std::endl;
        return;
    } catch (const std::exception& e) {
        throw std::runtime_error("错误: " + std::string(e.what()));
    } catch (...) {  // 捕获所有其他异常
        throw std::runtime_error("错误: 发生未知异常");
    }
} 

}