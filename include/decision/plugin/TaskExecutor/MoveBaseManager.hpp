#ifndef MOVE_BASE_MANAGER_H
#define MOVE_BASE_MANAGER_H

#include "ros/ros.h"

#include <thread>
#include <chrono>

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <geometry_msgs/PoseStamped.h> 
#include <std_msgs/Empty.h>
#include <actionlib/client/simple_action_client.h>
#include "move_base_msgs/MoveBaseAction.h"
#include "decision/core/robot_pose.h"

struct WaypointTask 
{
    int32_t task_id = -1;
    ros::Time task_start_time;
    ros::Time task_completed_time;
    geometry_msgs::PoseStamped waypoint_pose;
    geometry_msgs::PoseStamped LastRobot_pose;
    geometry_msgs::PoseStamped LastGoal;
    double LastPose_time_tolerance = 5; // 5 秒内没有更新机器人位置, 则认为加大目标点误差
    double LastPose_time_cancel = 10; // 10 秒内没有更新机器人位置, 则认为机器人已经到达目标点
    float tolerance_plus = 0.3;
    float tolerance = 0.3;
    bool completed = false;
};


class move_base_manager{

public:
    move_base_manager(ros::NodeHandle *nh, std::string frame_id, std::string child_frame_id): nh_(nh),running_(true){
        std::cout << "move_base_manager is created" << std::endl;
        this->frame_id_ = frame_id;
        this->child_frame_id_ = child_frame_id;
        this->run();
    }
    ~move_base_manager(){
        running_ = false;
        if (loop_thread && loop_thread->joinable()) {
            loop_thread->join();
        }
        printf("move_base_manager is destoryed\n");
    }

    void cancelAllGoal(){
        std::lock_guard<std::mutex> lock(mutex_);
        this->sendGoal(this->transform.transform.translation.x, this->transform.transform.translation.y);
        this->waypoint_task_.task_id = -1;
    }


    void sendGoal(float x, float y){
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.pose.position.x = x;
        goal.target_pose.pose.position.y = y;
        goal.target_pose.pose.position.z = 0;
        goal.target_pose.pose.orientation.w = 1;
        goal.target_pose.pose.orientation.x = 0;
        goal.target_pose.pose.orientation.y = 0;
        goal.target_pose.pose.orientation.z = 0;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        if ((abs(current_goal.pose.position.x - x) > 0.01 || abs(current_goal.pose.position.y - y) > 0.01))
        {
            this->client_->sendGoal(goal);
        }   
    }


    void updateRobotPose(){
        // 如果机器人位置发生变化, 则更新机器人位置
        if (abs(waypoint_task_.LastRobot_pose.pose.position.x - robot_pose_x) > 0.01 || 
            abs(waypoint_task_.LastRobot_pose.pose.position.y - robot_pose_y) > 0.01)
        {
            waypoint_task_.LastRobot_pose.pose.position.x = robot_pose_x;
            waypoint_task_.LastRobot_pose.pose.position.y = robot_pose_y;
            waypoint_task_.LastRobot_pose.header.stamp = ros::Time::now();
            waypoint_task_.LastRobot_pose.header.frame_id = frame_id_;
        }
    }

    void updateLastGoal(float x, float y){
        if (abs(waypoint_task_.LastGoal.pose.position.x - x) > 0.01 || 
            abs(waypoint_task_.LastGoal.pose.position.y - y) > 0.01)
        {
            waypoint_task_.LastGoal.pose.position.x = x;
            waypoint_task_.LastGoal.pose.position.y = y;
            waypoint_task_.LastGoal.header.stamp = ros::Time::now();
        }
    }

    WaypointTask createWaypointTask(int id, geometry_msgs::PoseStamped goal){
        if (this->waypoint_task_.task_id != id){ // 新任务
            this->waypoint_task_.task_id = id;
            this->waypoint_task_.task_start_time = ros::Time::now();
            this->waypoint_task_.task_completed_time = ros::Time(0);
            this->waypoint_task_.waypoint_pose = goal;
            this->waypoint_task_.completed = false;
        }
        {
            this->updateRobotPose();
            std::lock_guard<std::mutex> lock(mutex_);
            double plus_tolerance = 0;
            
            
            // if (ros::Time::now() - waypoint_task_.LastRobot_pose.header.stamp > ros::Duration(waypoint_task_.LastPose_time_tolerance)
            //     && ros::Time::now() - waypoint_task_.LastGoal.header.stamp > ros::Duration(waypoint_task_.LastPose_time_tolerance)
            // ){
            //     plus_tolerance = waypoint_task_.tolerance_plus;
            //     ROS_WARN("机器人太久没动，加大目标点误差");
            // }else if (ros::Time::now() - waypoint_task_.LastGoal.header.stamp > ros::Duration(waypoint_task_.LastPose_time_cancel)){
            //     ROS_ERROR("目标点太久没动，取消目标点");
            //     this->waypoint_task_.task_id = -1;
            //     this->waypoint_task_.task_completed_time = ros::Time::now();
            //     this->waypoint_task_.completed = true;
            //     return this->waypoint_task_;
            // }

            if (abs(transform.transform.translation.x - goal.pose.position.x) > this->waypoint_task_.tolerance + plus_tolerance|| 
                abs(transform.transform.translation.y - goal.pose.position.y) > this->waypoint_task_.tolerance + plus_tolerance)
            {   // 如果当前位置与目标位置的距离大于阈值, 且当前目标点与目标点不一致, 则发送目标点
                this->sendGoal(goal.pose.position.x, goal.pose.position.y);
                this->updateLastGoal(goal.pose.position.x, goal.pose.position.y);
                #ifdef DEBUG
                std::cout << "send goal" << "id: " << id << "  x: " << goal.pose.position.x << "y: " << goal.pose.position.y << std::endl;
                #endif
            }
            else{
                #ifdef DEBUG
                std::cout << "task completed" << "id: " << id << "  x: " << transform.transform.translation.x << "y: " << transform.transform.translation.y << std::endl;
                #endif
                this->waypoint_task_.task_completed_time = ros::Time::now();
                this->waypoint_task_.completed = true;
            }
        }
        return this->waypoint_task_;
    }

private:

    void run(){
        loop_thread = std::make_unique<std::thread>(&move_base_manager::loop, this); // 启动线程
    }

    void loop(){
        this->client_ = std::make_unique<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>(*nh_, "move_base"); // 初始化move_base客户端, 可能被阻塞
        this->tf_buffer = std::make_unique<tf2_ros::Buffer>();
        this->tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);
        sub_current_goal = std::make_unique<ros::Subscriber>(nh_->subscribe("/move_base/current_goal", 1, &move_base_manager::current_goal_callback, this));
        while(this->running_ && ros::ok()){
            {
                std::lock_guard<std::mutex> lock(mutex_);
                try{
                    transform = tf_buffer->lookupTransform(frame_id_, child_frame_id_, ros::Time(0));
                    robot_pose_x = transform.transform.translation.x;
                    robot_pose_y = transform.transform.translation.y;
                    robot_pose_z = transform.transform.translation.z;
                    ros::spinOnce();    
                }catch(tf2::TransformException &ex){
                    ROS_WARN("%s", ex.what());
                }
            }
            #ifdef DEBUG
            std::cout << "transform x: " << transform.transform.translation.x << " y: " << transform.transform.translation.y << std::endl;
            #endif
            ros::Duration(0.04).sleep();
        }
    }

    void current_goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
        current_goal = *msg;
        // #ifdef DEBUG
        std::cout << "current_goal x: " << current_goal.pose.position.x << " y: " << current_goal.pose.position.y << std::endl;
        // #endif
    }

private:
    WaypointTask waypoint_task_;
    std::atomic<bool> running_;
    std::mutex mutex_; // 用于保护共享资源的互斥锁
    ros::NodeHandle *nh_;

    std::unique_ptr<ros::Subscriber> sub_current_goal;
    std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> client_;
    std::unique_ptr<std::thread> loop_thread;


    std::string frame_id_;
    std::string child_frame_id_;

    // 用于获取机器人位置
    std::unique_ptr<tf2_ros::Buffer> tf_buffer;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener;
    geometry_msgs::TransformStamped transform;

    // 用于获取当前目标点
    geometry_msgs::PoseStamped current_goal;
};

#endif // MOVE_BASE_MANAGER_H