#ifndef DECISION_H
#define DECISION_H

#include "roborts_msgs/driver.h"
#include "ostream"
#include <fstream>
#include <sstream>
#include <string>
#include "decision/plugin/TaskExecutor/MoveBaseManager.hpp"
#include "decision/rules/graph.h"
#include "decision/rules/info.hpp"
#include "decision/core/database.hpp"
#include "decision/DecisionStatus.h"
#include "nlohmann/json.hpp"

struct DecisionResult{
    std::vector<int> nodepath_;
    ros::Duration duration_;
    int target_id_;
};

struct DecisionConfig{
    std::string data_version_;
    std::string version_;
    std::string decisionJsonStr_;
    std::string decision_config_path_;
    std::string referee_topic_;
    std::string topic_cancel_goal_;
    std::string topic_send_goal_;
    std::string frame_id_;
    std::string child_frame_id_;
    bool is_debug_;
};

class DecisionCore{
public:
    // 构造函数
    DecisionCore(DecisionConfig decision_config, ros::NodeHandle *nh):decision_config_(decision_config), nh_(nh){
        this->move_base_manager_ = std::make_unique<move_base_manager>
        (nh, decision_config_.frame_id_, decision_config_.child_frame_id_);
        this->init();
    }

    ~DecisionCore(){
        this->run_decision_ = false;
        this->run_subsriber_ = false;
        this->threads.join_all();
    }
    
    // 初始化函数，读取规则文件并排序
    void init(){
        //读取规则文件
        this->readRules();
        // 初始化发布者
        this->decision_pub_ = this->nh_->advertise<decision::DecisionStatus>("/decision_status", 10);
        // 初始化订阅者
        this->threads.create_thread(boost::bind(&DecisionCore::initSubsriber, this));
    }

    // 读取规则文件的函数
    void readRules(){
        try {
            std::cout << "decision_config_.decision_config_path_: " << this->decision_config_.decision_config_path_ << std::endl;
            this->graph_ = std::make_shared<rules::Graph>();
            this->graph_->init(this->decision_config_.decision_config_path_);
            this->database_.bind_graph(this->graph_);

            rules::printGraph(*this->graph_);
        } catch (const std::exception& e) {
            throw std::runtime_error("Failed to parse JSON from file: " + decision_config_.decision_config_path_ + ". Error: " + e.what());
        }
    }

    
    // 运行函数
    void run(){
        // 决策
        this->run_decision_ = true;
        this->threads.create_thread(boost::bind(&DecisionCore::decision, this));
    }

    void stop(){
        this->run_decision_ = false;
    }

    void decision(){
        while(this->data_size_ < 10){
            ros::Rate(10).sleep();
        }
        while(this->run_decision_){
            this->decision_result_ = this->getTargetNode();
            std::cout << "target_node_id = " << this->decision_result_.target_id_ << std::endl;
            this->decision_result_.nodepath_.push_back(this->decision_result_.target_id_);
            {
                nlohmann::json json_data;
                std::string data_tabs = this->database_.get_data_tabs();
                json_data["data_tabs"] = nlohmann::json::parse(data_tabs);
                json_data["path"]["nodeids"] = this->decision_result_.nodepath_;
                json_data["path"]["duration"] = this->decision_result_.duration_.toSec();
                json_data["path"]["target_id"] = this->decision_result_.target_id_;
                decision::DecisionStatus decision_status;
                decision_status.jsonData = json_data.dump();
                this->decision_pub_.publish(decision_status);
            }

            auto& target_node = this->graph_->nodeList[this->decision_result_.target_id_];
            this->database_.update_task_id(target_node.node_id);
            // 根据决策结果执行动作
            if (target_node.type == rules::NodeType::ROOT)
            {
                // 取消所有目标
                this->move_base_manager_->cancelAllGoal();
            }
            if(target_node.mode == rules::NodeMode::NAVIGATION){
                auto result = this->move_base_manager_->createWaypointTask(target_node.node_id, target_node.waypoint);
                target_node.start_time = result.task_start_time;
                target_node.end_time = result.task_completed_time;
                target_node.finish = result.completed;
                std::cout << "正在执行导航" << std::endl;
            }else if(target_node.mode == rules::NodeMode::STAY){
                std::cout << "正在执行停留" << std::endl;
                // 停留, 暂时不实现
                this->move_base_manager_->cancelAllGoal();
            }else if(target_node.mode == rules::NodeMode::LIMIT){
                std::cout << "正在执行限制" << std::endl;
                // 限制, 暂时不实现
                nh_->setParam("/decision/limit_linear", target_node.limit_linear);
                nh_->setParam("/decision/limit_angular", target_node.limit_angular);
                target_node.finish = true;
            }
            ros::Rate(10).sleep();
        }
    }

    DecisionResult getTargetNode(){
        DecisionResult decision_result;
        
        ros::Time start_time = ros::Time::now();

        auto node_id = this->graph_->rootNode_id;
        auto node = this->graph_->nodeList[node_id];
        auto edges = &node.edges;
        bool result = 1;

        // 如果当前节点完成，则继续寻找下一节点, 直到找到未完成的节点
        while(this->graph_->CheckFinish(node_id, this->nh_) == true){
            decision_result.nodepath_.push_back(node_id);
            int edge_index = 0;
            for(auto edge : *edges){
                result = 1;
                // 遍历所有条件, 检测是否存在不满足的条件
                for(auto condition : edge.rules.conditions){
                    ros::Duration duration = ros::Duration(condition.scope_value);   
                    result = this->database_.check_data(condition.datatype, ros::Time::now(), 
                                    duration, 
                                    (rules::metricType)condition.metric_type,  
                                    condition.min_value, 
                                    condition.max_value);
                    std::cout << condition.datatype << "result = " << result << std::endl;
                    if(result == 0) break;// 如果有一个条件不满足，则跳出循环
                }

                // 如果所有条件都满足，且任务未完成或者目标是分支节点，则跳转到下一节点
                ROS_ERROR("node_id: %d, %d", node_id,this->graph_->CheckFinish(edge.nodeOut_id, this->nh_));
                if(result && 
                (this->graph_->CheckFinish(edge.nodeOut_id, this->nh_) == false || this->graph_->nodeList[edge.nodeOut_id].type == rules::NodeType::BRANCH)){
                    node_id = edge.nodeOut_id;
                    node = this->graph_->nodeList[node_id];
                    edges = &node.edges;
                    break;
                }
                
                // 如果已经遍历完所有节点，任然无法找到可跳转的下一节点，则返回当前节点
                else if(edge_index == edges->size() - 1){
                    std::cout << "遍历完所有节点，任然无法找到可跳转的下一节点，返回当前节点" << std::endl;
                    decision_result.duration_ = ros::Time::now() - start_time;
                    decision_result.target_id_ = node_id;
                    return decision_result;
                }
                edge_index++;
            }
            // 如果时间超过0.5秒，则返回当前节点, 可能有环
            if (ros::Time::now() - start_time > ros::Duration(0.5))
            {
                decision_result.duration_ = ros::Time::now() - start_time;
                decision_result.target_id_ = node_id;
                return decision_result;
            }
        }
        // 寻找到未完成的任务节点
        decision_result.duration_ = ros::Time::now() - start_time;
        decision_result.target_id_ = node_id;
        return decision_result;
    }

    void initSubsriber(){
        this->run_subsriber_ = true;
        while(this->run_subsriber_){
            this->referee_sub_ = this->nh_->subscribe(decision_config_.referee_topic_, 10, &DecisionCore::referee_msgs_cb, this);
            ros::spinOnce();
            ros::Duration(0.1).sleep();
        }
    }

    // 裁判系统消息回调函数
    void referee_msgs_cb(const roborts_msgs::driver::ConstPtr data){
        database_.update_data(*data);
        this->data_size_++;
    }

private:
    ros::NodeHandle *nh_;
    
    // 线程关闭信号
    std::atomic<bool> run_decision_;
    std::atomic<bool> run_subsriber_;

    std::atomic<int> data_size_;

    std::unique_ptr<move_base_manager> move_base_manager_;
    
    ros::Subscriber referee_sub_;
    ros::Publisher decision_pub_;

    DecisionConfig decision_config_;
    DecisionResult decision_result_;

    std::shared_ptr<rules::Graph> graph_;
    database::Database database_;

    boost::thread_group threads;
};





#endif // DECISION_H