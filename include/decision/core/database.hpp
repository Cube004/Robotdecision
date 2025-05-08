#ifndef DATABASE_H
#define DATABASE_H

#include <vector>
#include <unordered_map>
#include <atomic>
#include <shared_mutex>

#include <ros/ros.h>
#include <roborts_msgs/driver.h>
#include <geometry_msgs/PoseStamped.h>
#include "decision/rules/graph.h"

// 前向声明
namespace rules {
    class Graph;
}

// 容器预分配大小
#define container_size 300 * 30

namespace database
{
    struct data
    {
        std::string name_;
        std::vector<int> raw_data;        // 原始数据
        std::vector<int> prefix_inc;      // 预处理累计增加
        std::vector<int> prefix_dec;      // 预处理累计减少
        std::vector<ros::Time> timestamp; // 时间戳索引
        std::unordered_map<int, std::vector<ros::Time>> timestamp_map; // 记录每个数据对应的所有时间戳
        data(){
            raw_data.reserve(container_size);
            prefix_inc.reserve(container_size);
            prefix_dec.reserve(container_size);
            timestamp.reserve(container_size);
            timestamp_map.reserve(container_size);
        }
    };

    enum prefixType{
        INCREASE = 1,
        DECREASE = 2,
        SUM = 3
    };
    
    class Database{
        private:
            std::unordered_map<std::string, data> dataContainer_;
            geometry_msgs::PoseStamped robot_pose_;
            // 自旋锁 (使用std::atomic_flag实现)
            std::atomic_flag spinlock = ATOMIC_FLAG_INIT;
            std::shared_ptr<::rules::Graph> graph_;
        public:
            void update_data(roborts_msgs::driver msg);
            bool check_data(std::string datatype, ros::Time end, ros::Duration duration, int type, int min_value = 0, int max_value = 0);
            void update_task_id(int task_id);
            void bind_graph(std::shared_ptr<::rules::Graph> graph);
        private:
            bool check_in_area(std::string datatype);
            bool check_near_waypoint(std::string datatype);
            bool confirm_metric_compliance(std::string datatype, ros::Time end, ros::Duration duration, int type, int min_value = 0, int max_value = 0);
            int get_duration_data(data &data, ros::Time end, ros::Duration duration, prefixType type);
            int get_current_data(data &data);
            bool hasValueDuring(data &data, ros::Time end, ros::Duration duration, int min_value = 0, int max_value = 0);
            std::pair<int, int> Time_get_index(data &data, ros::Time end, ros::Duration duration);
            data &get_data_base(std::string datatype);
            void preprocess_data(std::string datatype, ros::Time stamp, int value);
    };

    data &Database::get_data_base(std::string datatype){
        // 获取数据
        return dataContainer_[datatype];
    }

    void Database::bind_graph(std::shared_ptr<::rules::Graph> graph){
        this->graph_ = graph;
    }

    void Database::preprocess_data(std::string datatype, ros::Time stamp, int value){
        // 预处理数据
        auto data = get_data_base(datatype);
        if (!data.raw_data.empty()){
            if (stamp <= data.timestamp.back()){
                std::cout << "stamp <= data.timestamp.back()" << std::endl;
                return;
            }
        }// 避免时间戳回退

        if (data.raw_data.empty()){
            data.prefix_dec.push_back(0);
            data.prefix_inc.push_back(0);
        }else if (value > data.raw_data.back()){
            data.prefix_inc.push_back(value - data.raw_data.back() + data.prefix_inc.back());
            data.prefix_dec.push_back(data.prefix_dec.back());
        }else if (value < data.raw_data.back()){
            data.prefix_dec.push_back(data.raw_data.back() - value + data.prefix_dec.back());
            data.prefix_inc.push_back(data.prefix_inc.back());
        }else{
            data.prefix_inc.push_back(data.prefix_inc.back());
            data.prefix_dec.push_back(data.prefix_dec.back());
        }
        
        data.raw_data.push_back(value);
        data.timestamp.push_back(stamp);
        data.timestamp_map[value].push_back(stamp);
    }


    
    enum metricType{
        CURRENT_VALUE = 1, // 当前值
        TOTAL_INCREMENT = 2, // 总增量
        TOTAL_DECREMENT = 3, // 总减量
        HISTORICAL_PRESENCE = 4, // 历史存在
        IN_REGION = 5, // 在区域内
        NEAR_WAYPOINT = 6 // 靠近航点
    };

    void Database::update_data(roborts_msgs::driver msg){
        // 使用自旋锁
        while (spinlock.test_and_set(std::memory_order_acquire));
        
        // 更新数据
        preprocess_data("game_progress", msg.header.stamp, msg.game_progress);
        preprocess_data("stage_remain_time", msg.header.stamp, msg.stage_remain_time);
        preprocess_data("own_robot_HP", msg.header.stamp, msg.own_robot_HP);
        preprocess_data("own_base_HP", msg.header.stamp, msg.own_base_HP);
        preprocess_data("enemy_base_HP", msg.header.stamp, msg.enemy_base_HP);
        preprocess_data("event_data", msg.header.stamp, msg.event_data);
        preprocess_data("current_HP", msg.header.stamp, msg.current_HP);
        preprocess_data("shoot_num", msg.header.stamp, msg.shoot_num);
        preprocess_data("vision_status", msg.header.stamp, msg.vision_status);
        preprocess_data("HP_deduction_reason", msg.header.stamp, msg.HP_deduction_reason);
        preprocess_data("armor_id", msg.header.stamp, msg.armor_id);
        preprocess_data("rfid_status", msg.header.stamp, msg.rfid_status);
        
        // 释放自旋锁
        spinlock.clear(std::memory_order_release);
    }

    void Database::update_task_id(int task_id){
        // 使用自旋锁
        while (spinlock.test_and_set(std::memory_order_acquire));
        // 更新任务id
        this->preprocess_data("last_task_id", ros::Time::now(), task_id);
        // 释放自旋锁
        spinlock.clear(std::memory_order_release);
    }

    bool Database::check_data(std::string datatype, ros::Time end, ros::Duration duration, int type, int min_value, int max_value){
        // 使用自旋锁
        while (spinlock.test_and_set(std::memory_order_acquire));
        bool result = confirm_metric_compliance(datatype, end, duration, type, min_value, max_value);
        // 释放自旋锁
        spinlock.clear(std::memory_order_release);
        return result;
    }
    
    bool Database::confirm_metric_compliance(std::string datatype, ros::Time end, ros::Duration duration, int type, int min_value, int max_value){
        try {
            data &data = get_data_base(datatype);
            if (data.raw_data.empty()) return false;
            if (type == CURRENT_VALUE)
            {
                int result = get_current_data(data);
                if (min_value <= result && result <= max_value)return true;
                else return false;
            }
            else if (type == TOTAL_INCREMENT)
            {
                int result = get_duration_data(data, end, duration, prefixType::INCREASE);
                if (min_value <= result && result <= max_value)return true;
                else return false;
            }
            else if (type == TOTAL_DECREMENT)
            {
                int result = get_duration_data(data, end, duration, prefixType::DECREASE);
                if (min_value <= result && result <= max_value)return true;
                else return false;
            }
            else if (type == HISTORICAL_PRESENCE)
            {
                return hasValueDuring(data, end, duration, min_value, max_value);
            }else if (type == IN_REGION)
            {
                return check_in_area(datatype);
            }else if (type == NEAR_WAYPOINT)
            {
                return check_near_waypoint(datatype);
            }
        } catch (const std::runtime_error& e) {
            std::cout << e.what() << std::endl;
            std::cout << "error: not found data" << std::endl;
        }
        return false;
    }

    bool Database::check_in_area(std::string datatype){
        int area_id = std::stoi(datatype);
        if (this->robot_pose_.pose.position.x > this->graph_->areaList[area_id].leftTop.pose.position.x && 
            this->robot_pose_.pose.position.y > this->graph_->areaList[area_id].leftTop.pose.position.y && 
            this->robot_pose_.pose.position.x < this->graph_->areaList[area_id].rightBottom.pose.position.x && 
            this->robot_pose_.pose.position.y < this->graph_->areaList[area_id].rightBottom.pose.position.y)
        {
            return true;
        }
        return false;   
    }

    bool Database::check_near_waypoint(std::string datatype){
        int waypoint_id = std::stoi(datatype);
        if (this->robot_pose_.pose.position.x > this->graph_->waypointList[waypoint_id].pose.pose.position.x - 0.5 && 
            this->robot_pose_.pose.position.x < this->graph_->waypointList[waypoint_id].pose.pose.position.x + 0.5 && 
            this->robot_pose_.pose.position.y > this->graph_->waypointList[waypoint_id].pose.pose.position.y - 0.5 && 
            this->robot_pose_.pose.position.y < this->graph_->waypointList[waypoint_id].pose.pose.position.y + 0.5)
        {
            return true;
        }
        return false;
    }

    int Database::get_duration_data(data &data, ros::Time end, ros::Duration duration, prefixType type){
        // 获取一段时间内数据的变化量
        if (data.raw_data.empty())throw std::runtime_error("error: raw_data is empty");
        if (end - duration < data.timestamp.front())throw std::runtime_error("error: duration is out of range");

        std::pair<int, int> index = Time_get_index(data, end, duration);
        int start_index = index.first;
        int end_index = index.second;

        if (type == INCREASE)return data.prefix_inc[end_index] - data.prefix_inc[start_index];
        else if (type == DECREASE)return data.prefix_dec[end_index] - data.prefix_dec[start_index];
        else if (type == SUM)return data.prefix_inc[end_index] - data.prefix_inc[start_index] + data.prefix_dec[end_index] - data.prefix_dec[start_index];
        else throw std::runtime_error("error: invalid type");
    }

    int Database::get_current_data(data &data){
        // 获取当前数据
        if (data.raw_data.empty())throw std::runtime_error("error: raw_data is empty");
        return data.raw_data.back();
    }

    bool Database::hasValueDuring(data &data, ros::Time end, ros::Duration duration, int min_value, int max_value){
        // 判断在一段时间内是否有数据
        if (data.raw_data.empty())throw std::runtime_error("error: raw_data is empty");
        if (end - duration < data.timestamp.front())throw std::runtime_error("error: duration is out of range");
        if (min_value == max_value)
        {// 利用哈希表快速查找
            std::vector<ros::Time> timestamps = data.timestamp_map[min_value];
            if (timestamps.empty())return false;
            for (auto &timestamp : timestamps){
                if (timestamp <= end && timestamp >= end - duration)return true;
            }
            return false;
        }else{
            std::pair<int, int> index = Time_get_index(data, end, duration);
            int start_index = index.first;
            int end_index = index.second;
            for (int i = start_index; i < end_index; i++){
                if (data.raw_data[i] >= min_value && data.raw_data[i] <= max_value)return true;
            }
            return false;
        }
    }

    std::pair<int, int> Database::Time_get_index(data &data, ros::Time end, ros::Duration duration){
        if (data.timestamp.empty()) throw std::runtime_error("error: timestamp is empty");
        
        // 默认使用最后一个索引
        int end_index = data.timestamp.size() - 1;
        
        // 如果end比最后时间戳大，但在可接受范围内，仍使用最后索引
        // 否则使用二分查找
        if (end - data.timestamp.back() > ros::Duration(0.1)){
            auto it = std::upper_bound(data.timestamp.begin(), data.timestamp.end(), end);
            if (it == data.timestamp.end()) {
                end_index = data.timestamp.size() - 1;  // 使用最后一个索引
            } else {
                end_index = it - data.timestamp.begin() - 1;  // 减1获取小于等于end的最大索引
            }
        }
        
        // 查找起始索引
        int start_index = std::lower_bound(data.timestamp.begin(), data.timestamp.end(), end - duration) - data.timestamp.begin();
        
        // 边界检查
        if (start_index > end_index) throw std::runtime_error("error: start_index is out of range");
        if (end_index >= data.timestamp.size()) throw std::runtime_error("error: end_index is out of range");
        
        return std::make_pair(start_index, end_index);
    }
} // namespace database

#endif // DATABASE_H