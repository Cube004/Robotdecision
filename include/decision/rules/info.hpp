#pragma once
#include "graph.h"

namespace rules {
    // 打印航点信息
    void printWaypoint(const rules::Waypoint& waypoint) {
        std::cout << "    名称: " << waypoint.name << std::endl;
        std::cout << "    位置: (" << waypoint.pose.pose.position.x 
                << ", " << waypoint.pose.pose.position.y 
                << ", " << waypoint.pose.pose.position.z << ")" << std::endl;
    }

    // 打印区域信息
    void printArea(const rules::Area& area) {
        std::cout << "    名称: " << area.name << std::endl;
        std::cout << "    左上角: (" << area.leftTop.pose.position.x 
                << ", " << area.leftTop.pose.position.y 
                << ", " << area.leftTop.pose.position.z << ")" << std::endl;
        std::cout << "    右下角: (" << area.rightBottom.pose.position.x 
                << ", " << area.rightBottom.pose.position.y 
                << ", " << area.rightBottom.pose.position.z << ")" << std::endl;
    }

    // 打印边信息
    void printEdge(const rules::Edge& edge) {
        std::cout << "    源节点ID: " << edge.nodeIn_id << std::endl;
        std::cout << "    目标节点ID: " << edge.nodeOut_id << std::endl;
        
        // 打印边的条件
        std::cout << "    条件数量: " << edge.rules.conditions.size() << std::endl;
        for (const auto& condition : edge.rules.conditions) {
            std::cout << "      条件: " << std::endl;
            std::cout << "        判断依据: " << condition.metric_type << std::endl;
            if (condition.metric_type == metricType::IN_REGION) {
                std::cout << "     数据 区域ID: " << condition.datatype << std::endl;
            }else if (condition.metric_type == metricType::NEAR_WAYPOINT) {
                std::cout << "     数据 航点ID: " << condition.datatype << std::endl;
            }else{
                std::cout << "     数据 数据类型: " << condition.datatype << std::endl;
            }
            std::cout << "        时间范围类型: " << condition.temporal_scope << std::endl;
            std::cout << "        滚动窗口: " << condition.scope_value << std::endl;
            std::cout << "        最大值: " << condition.max_value << std::endl;
            std::cout << "        最小值: " << condition.min_value << std::endl;
        }
    }

    // 打印节点组信息
    void printNodeGroup(const rules::NodeGroup& group) {
        std::cout << "    组ID: " << group.group_id << std::endl;
        std::cout << "    循环: " << (group.Loop ? "是" : "否") << std::endl;
        std::cout << "    重置时间: " << group.ResetTime << std::endl;
        std::cout << "    反向: " << (group.Reverse ? "是" : "否") << std::endl;
        
        // 打印节点组中的节点ID
        std::cout << "    节点ID列表: ";
        for (const auto& nodeId : group.nodes) {
            std::cout << nodeId << " ";
        }
        std::cout << std::endl;
    }

    // 打印节点信息
    void printNode(const rules::Node& node) {
        std::cout << "    节点ID: " << node.node_id << std::endl;
        
        // 打印节点类型
        std::string nodeType;
        switch (node.type) {
            case rules::NodeType::TASK: nodeType = "任务节点"; break;
            case rules::NodeType::BRANCH: nodeType = "分支节点"; break;
            case rules::NodeType::ROOT: nodeType = "根节点"; break;
            default: nodeType = "未知类型"; break;
        }
        std::cout << "    节点类型: " << nodeType << std::endl;
        
        // 打印节点模式
        std::string nodeMode;
        switch (node.mode) {
            case rules::NodeMode::NAVIGATION: nodeMode = "导航模式"; break;
            case rules::NodeMode::STAY: nodeMode = "停留模式"; break;
            case rules::NodeMode::LIMIT: nodeMode = "限制模式"; break;
            default: nodeMode = "未知模式"; break;
        }
        std::cout << "    节点模式: " << nodeMode << std::endl;
        
        // 打印航点信息
        if (node.waypoint.pose.position.x != 0 || node.waypoint.pose.position.y != 0) {
            std::cout << "    航点位置: (" << node.waypoint.pose.position.x 
                    << ", " << node.waypoint.pose.position.y 
                    << ", " << node.waypoint.pose.position.z << ")" << std::endl;
        }
        
        std::cout << "    重置时间: " << node.resetTime << std::endl;
        
        // 打印边的数量
        std::cout << "    边数量: " << node.edges.size() << std::endl;
        
        // 打印节点组ID
        std::cout << "    所属节点组ID: ";
        for (const auto& groupId : node.group_id) {
            std::cout << groupId << " ";
        }
        std::cout << std::endl;
    }

    // 打印整个图的信息
    void printGraph(const rules::Graph& graph) {
        

        std::cout << "\n===== 图信息 =====" << std::endl;
        
        // 打印根节点ID
        std::cout << "根节点ID: " << graph.rootNode_id << std::endl;
        
        // 打印航点列表
        std::cout << "\n航点列表 (数量: " << graph.waypointList.size() << "):" << std::endl;
        for (const auto& waypoint : graph.waypointList) {
            std::cout << "  航点ID: " << waypoint.first << std::endl;
            printWaypoint(waypoint.second);
        }
        
        // 打印区域列表
        std::cout << "\n区域列表 (数量: " << graph.areaList.size() << "):" << std::endl;
        for (const auto& area : graph.areaList) {
            std::cout << "  区域ID: " << area.first << std::endl;
            printArea(area.second);
        }
        
        // 打印边列表
        std::cout << "\n边列表 (数量: " << graph.edgeList.size() << "):" << std::endl;
        for (const auto& edge : graph.edgeList) {
            std::cout << "  边ID: " << edge.first << std::endl;
            printEdge(edge.second);
        }
        
        // 打印节点组列表
        std::cout << "\n节点组列表 (数量: " << graph.nodeGroupList.size() << "):" << std::endl;
        for (const auto& group : graph.nodeGroupList) {
            std::cout << "  节点组ID: " << group.first << std::endl;
            printNodeGroup(group.second);
        }
        
        // 打印节点列表
        std::cout << "\n节点列表 (数量: " << graph.nodeList.size() << "):" << std::endl;
        for (const auto& node : graph.nodeList) {
            std::cout << "  节点ID: " << node.first << std::endl;
            printNode(node.second);
        }
        
        std::cout << "\n===== 图信息结束 =====" << std::endl;
    }
}