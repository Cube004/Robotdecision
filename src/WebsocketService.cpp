#include "ros/ros.h"
#include "decision/DecisionStatus.h"
#include <libwebsockets.h>
#include <vector>
#include <mutex>
#include <string>
#include <map>
#include "nlohmann/json.hpp"
#include "roborts_msgs/driver.h"
#include <fstream>
#include <boost/filesystem.hpp>
#include <sstream>
#include <chrono>
#include <ctime>


ros::Publisher driver_pub;

// WebSocket 相关全局变量
static struct lws_context *context;
static std::vector<struct lws *> clients;
static std::mutex clients_mutex;
static std::string latest_json_data;
static bool new_data_available = false;

// 消息缓冲区，用于存储分片消息
static std::map<struct lws*, std::string> message_buffers;
static std::mutex buffer_mutex;

// WebSocket 回调函数的前向声明
void decision_status_cb(const decision::DecisionStatus::ConstPtr& msg);
static int callback_websocket(struct lws *wsi, enum lws_callback_reasons reason, void *user, void *in, size_t len);
// 处理从WebSocket客户端接收的消息
void handle_websocket_message(struct lws *wsi, void *in, size_t len);
void process_complete_message(const std::string& message);

// WebSocket 协议定义
static const struct lws_protocols protocols[] = {
    {
        "decision-protocol",
        callback_websocket,
        0,
        4096,
    },
    { NULL, NULL, 0, 0 } // 终结器
};

// WebSocket 回调函数实现
static int callback_websocket(struct lws *wsi, enum lws_callback_reasons reason, void *user, void *in, size_t len) {
    switch (reason) {
        case LWS_CALLBACK_ESTABLISHED: {
            ROS_INFO("WebSocket连接已建立");
            std::lock_guard<std::mutex> lock(clients_mutex);
            clients.push_back(wsi);
            break;
        }
        case LWS_CALLBACK_CLOSED: {
            ROS_INFO("WebSocket连接已关闭");
            {
                std::lock_guard<std::mutex> buffer_lock(buffer_mutex);
                message_buffers.erase(wsi); // 清理该连接的消息缓冲
            }
            
            std::lock_guard<std::mutex> lock(clients_mutex);
            auto it = std::find(clients.begin(), clients.end(), wsi);
            if (it != clients.end()) {
                clients.erase(it);
            }
            break;
        }
        case LWS_CALLBACK_RECEIVE: {
            ROS_INFO("接收到WebSocket客户端数据，长度: %zu", len);
            handle_websocket_message(wsi, in, len);
            break;
        }
        case LWS_CALLBACK_SERVER_WRITEABLE: {
            if (new_data_available) {
                // 为LWS_PRE预留空间
                unsigned char *buf = new unsigned char[LWS_PRE + latest_json_data.length()];
                memcpy(&buf[LWS_PRE], latest_json_data.c_str(), latest_json_data.length());
                
                int n = lws_write(wsi, &buf[LWS_PRE], latest_json_data.length(), LWS_WRITE_TEXT);
                delete[] buf;
                
                if (n < (int)latest_json_data.length()) {
                    ROS_ERROR("WebSocket写入错误");
                    return -1;
                }
            }
            break;
        }
        default:
            break;
    }
    return 0;
}

// 广播JSON数据到所有连接的WebSocket客户端
void broadcast_json_data(const std::string& json_data) {
    // 更新最新的数据
    {
        std::lock_guard<std::mutex> lock(clients_mutex);
        latest_json_data = json_data;
        new_data_available = true;
    }
    
    // 对所有连接的客户端触发可写回调
    for (struct lws *client : clients) {
        lws_callback_on_writable(client);
    }
}

// ROS回调函数实现
void decision_status_cb(const decision::DecisionStatus::ConstPtr& msg){
    const std::string& jsonData = msg->jsonData;
    ROS_INFO("接收到决策状态数据: %s", jsonData.c_str());
    broadcast_json_data(jsonData);
}

// 处理从WebSocket客户端接收的消息
void handle_websocket_message(struct lws *wsi, void *in, size_t len) {
    // 将接收到的数据转换为字符串
    std::string fragment(static_cast<char*>(in), len);
    
    // 添加到该连接的消息缓冲区
    {
        std::lock_guard<std::mutex> lock(buffer_mutex);
        message_buffers[wsi] += fragment;
    }
    
    // 检查消息是否完整
    bool is_complete = false;
    std::string complete_message;
    
    {
        std::lock_guard<std::mutex> lock(buffer_mutex);
        std::string& buffer = message_buffers[wsi];
        
        // 检查JSON是否有效且完整
        try {
            auto json_test = nlohmann::json::parse(buffer);
            // 如果没有抛出异常，则JSON是完整的
            is_complete = true;
            complete_message = buffer;
            buffer.clear(); // 清空缓冲区
        } catch (const nlohmann::json::parse_error& e) {
            // JSON不完整，等待更多数据
            ROS_INFO("JSON不完整，等待更多数据. 当前缓冲区大小: %zu 字节", buffer.size());
            
            // 如果缓冲区太大，可能是无效数据，清除它
            if (buffer.size() > 10 * 1024 * 1024) { // 超过10MB
                ROS_WARN("缓冲区过大，清空缓冲区");
                buffer.clear();
            }
        }
    }
    
    // 如果消息完整，处理它
    if (is_complete && !complete_message.empty()) {
        ROS_INFO("接收到完整的JSON消息，长度: %zu 字节", complete_message.size());
        process_complete_message(complete_message);
    }
}

// 处理完整的WebSocket消息
void process_complete_message(const std::string& message) {
    // 解析JSON数据
    try {
        nlohmann::json root = nlohmann::json::parse(message);
        
        if (root.is_array()) {
            // 解析JSON数组
            roborts_msgs::driver driver_msg;
            driver_msg.header.stamp = ros::Time::now();
            for (const auto& item : root) {
                if (item.contains("label") && item.contains("data")) {
                    std::string label = item["label"];
                    int data = item["data"];
                    
                    // 处理不同类型的数据
                    if (label == "game_progress") {
                        driver_msg.game_progress = data;
                    } 
                    else if (label == "stage_remain_time") {
                        driver_msg.stage_remain_time = data;
                    }
                    else if (label == "own_robot_HP") {
                        driver_msg.own_robot_HP = data;
                    }
                    else if (label == "own_outpost_HP") {
                        driver_msg.own_outpost_HP = data;
                    }
                    else if (label == "own_base_HP") {
                        driver_msg.own_base_HP = data;
                    }
                    else if (label == "enemy_robot_HP") {
                        driver_msg.enemy_robot_HP = data;
                    }
                    else if (label == "enemy_outpost_HP") {
                        driver_msg.enemy_outpost_HP = data;
                    }
                    else if (label == "remaining_energy") {
                        driver_msg.remaining_energy = data;
                    }
                    else if (label == "enemy_base_HP") {
                        driver_msg.enemy_base_HP = data;
                    }
                    else if (label == "event_data") {
                        driver_msg.event_data = data;
                    }
                    else if (label == "current_HP") {
                        driver_msg.current_HP = data;
                    }
                    else if (label == "vision_status") {
                        driver_msg.vision_status = data;
                    }
                    else if (label == "maximum_HP") {
                        driver_msg.maximum_HP = data;
                    }
                    else if (label == "defence_buff") {
                        driver_msg.defence_buff = data;
                    }
                    else if (label == "vulnerability_buff") {
                        driver_msg.vulnerability_buff = data;
                    }
                    else if (label == "attack_buff") {
                        driver_msg.attack_buff = data;
                    }
                    else if (label == "armor_id") {
                        driver_msg.armor_id = data;
                    }
                    else if (label == "HP_deduction_reason") {
                        driver_msg.HP_deduction_reason = data;
                    }
                    else if (label == "shoot_num") {
                        driver_msg.shoot_num = data;
                    }
                    else if (label == "rfid_status") {
                        driver_msg.rfid_status = data;
                    }
                    else if (label == "remaining_energy") {
                        driver_msg.remaining_energy = data;
                    }
                    else if (label == "enemy_outpost_HP") {
                        driver_msg.enemy_outpost_HP = data;
                    }
                    else if (label == "own_outpost_HP") {
                        driver_msg.own_outpost_HP = data;
                    }
                    else if (label == "enemy_robot_HP") {
                        driver_msg.enemy_robot_HP = data;
                    }
                    else if (label == "maximum_HP") {
                        driver_msg.maximum_HP = data;
                    }
                    else if (label == "defence_buff") {
                        driver_msg.defence_buff = data;
                    }
                    else if (label == "vulnerability_buff") {
                        driver_msg.vulnerability_buff = data;
                    }
                    else if (label == "attack_buff") {
                        driver_msg.attack_buff = data;
                    }
                    else if (label == "obtain_shoot_num") {
                        driver_msg.obtain_shoot_num = data;
                    }
                }
            }
            driver_pub.publish(driver_msg);
            ROS_INFO("发布决策状态数据");
        } else if (root.is_object()) {
            // 检查是否是规则文件（包含特定字段）
            if (root.contains("version") && root.contains("createdAt") && root.contains("mapSettings")) {
                ROS_INFO("接收到规则文件，开始保存...");
                
                // 确保目录存在
                std::string rules_dir = "/home/cube/Documents/decision/rules";
                boost::filesystem::create_directories(rules_dir);
                
                // 生成文件名，使用当前时间戳
                auto now = std::chrono::system_clock::now();
                auto now_time_t = std::chrono::system_clock::to_time_t(now);
                std::stringstream filename;
                filename << rules_dir << "/rule_debug" << ".json";
                
                // 格式化JSON（美化输出）
                std::string formatted_json = root.dump(4); // 使用4个空格缩进
                
                // 写入文件
                std::ofstream file(filename.str());
                if (file.is_open()) {
                    file << formatted_json;
                    file.close();
                    ROS_INFO("规则文件已保存至: %s", filename.str().c_str());
                } else {
                    ROS_ERROR("无法打开文件进行写入: %s", filename.str().c_str());
                }
            } else {
                ROS_WARN("接收到未知格式的JSON对象");
            }
        } else {
            ROS_ERROR("接收到的JSON既不是数组也不是对象格式");
        }
    } catch (const nlohmann::json::parse_error& e) {
        ROS_ERROR("JSON解析失败: %s", e.what());
    }
}


// 主函数
int main(int argc, char **argv){
    ros::init(argc, argv, "decision_websocket_service");
    ros::NodeHandle nh;

    setlocale(LC_ALL, "");
    // 初始化WebSocket服务器
    struct lws_context_creation_info info;
    memset(&info, 0, sizeof(info));
    
    info.port = 9000;  // 设置WebSocket端口
    info.protocols = protocols;
    info.gid = -1;
    info.uid = -1;
    info.options = LWS_SERVER_OPTION_HTTP_HEADERS_SECURITY_BEST_PRACTICES_ENFORCE;
    
    context = lws_create_context(&info);
    if (!context) {
        ROS_ERROR("WebSocket上下文创建失败");
        return -1;
    }
    
    ROS_INFO("WebSocket服务器已启动，监听端口: %d", info.port);
    
    ros::Subscriber decision_sub = nh.subscribe("/decision_status", 10, decision_status_cb);
    driver_pub = nh.advertise<roborts_msgs::driver>("/referee", 10);
    // 同时处理ROS和WebSocket事件
    while (ros::ok()) {
        ros::spinOnce();
        lws_service(context, 50);  // 50ms超时
        ros::Duration(0.01).sleep();  // 10ms休眠
    }
    
    // 清理资源
    lws_context_destroy(context);
    return 0;
}